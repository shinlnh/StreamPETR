#!/usr/bin/env python3
"""Minimal TensorRT runner for the six-camera nuCarla StreamPETR engines.

Holds the temporal memory between frames so the two engines can be driven as a
stream: the encoder turns six images into features, the head consumes those
features plus the memory carried over from the previous frame and returns both
the detections and the memory for the next one.

Depends only on tensorrt, torch (for its CUDA allocator) and numpy, so it runs
without the training environment.
"""

from __future__ import annotations

from pathlib import Path
from typing import Dict, Sequence

import numpy as np
import tensorrt as trt
import torch

MEMORY_TENSORS = (
    "memory_embedding",
    "memory_reference_point",
    "memory_timestamp",
    "memory_egopose",
    "memory_velocity",
)


class Engine:
    """One TensorRT engine with persistent device buffers."""

    def __init__(self, path: str | Path, logger: trt.Logger):
        with open(path, "rb") as stream:
            self.engine = trt.Runtime(logger).deserialize_cuda_engine(stream.read())
        if self.engine is None:
            raise RuntimeError(f"could not deserialize {path}")
        self.context = self.engine.create_execution_context()

        self.input_names, self.output_names = [], []
        for index in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(index)
            if self.engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT:
                self.input_names.append(name)
            else:
                self.output_names.append(name)

        # Shapes are fixed at build time, so the buffers can be allocated once.
        self.buffers: Dict[str, torch.Tensor] = {}
        for name in self.input_names + self.output_names:
            shape = tuple(self.engine.get_tensor_shape(name))
            dtype = trt.nptype(self.engine.get_tensor_dtype(name))
            self.buffers[name] = torch.empty(
                shape, dtype=getattr(torch, np.dtype(dtype).name), device="cuda"
            )
            self.context.set_tensor_address(name, self.buffers[name].data_ptr())

    def __call__(self, feeds: Dict[str, np.ndarray]) -> Dict[str, np.ndarray]:
        missing = set(self.input_names) - set(feeds)
        if missing:
            raise KeyError(f"missing inputs: {sorted(missing)}")
        for name in self.input_names:
            value = np.ascontiguousarray(
                feeds[name], dtype=np.dtype(trt.nptype(self.engine.get_tensor_dtype(name)))
            )
            expected = tuple(self.buffers[name].shape)
            if value.shape != expected:
                raise ValueError(f"{name}: expected {expected}, got {value.shape}")
            self.buffers[name].copy_(torch.from_numpy(value))
        stream = torch.cuda.current_stream().cuda_stream
        if not self.context.execute_async_v3(stream_handle=stream):
            raise RuntimeError("TensorRT execution failed")
        torch.cuda.current_stream().synchronize()
        return {name: self.buffers[name].cpu().numpy() for name in self.output_names}


class StreamPETRRunner:
    """Drives the encoder and temporal head as a single streaming detector."""

    def __init__(
        self,
        encoder_engine: str | Path,
        head_engine: str | Path,
        verbosity: trt.Logger.Severity = trt.Logger.WARNING,
    ):
        logger = trt.Logger(verbosity)
        trt.init_libnvinfer_plugins(logger, "")
        self.encoder = Engine(encoder_engine, logger)
        self.head = Engine(head_engine, logger)
        self.memory: Dict[str, np.ndarray] = {}
        self.reset()

    def reset(self) -> None:
        """Clear the temporal state; call this when starting a new sequence."""
        self.memory = {
            f"pre_{name}": np.zeros(
                tuple(self.head.buffers[f"pre_{name}"].shape), dtype=np.float32
            )
            for name in MEMORY_TENSORS
        }
        self._seen_frame = False

    def __call__(
        self,
        images: np.ndarray,
        timestamp: float,
        ego_pose: np.ndarray,
        ego_pose_inv: np.ndarray,
    ) -> Dict[str, np.ndarray]:
        """Run one frame.

        Args:
            images: ``[1, 6, 3, 256, 704]`` float32, already normalized.
            timestamp: frame time in seconds.
            ego_pose: ``[1, 4, 4]`` lidar-to-global matrix.
            ego_pose_inv: ``[1, 4, 4]`` its inverse.
        """
        features = self.encoder({"images": images})["image_features"]

        feeds = {
            "image_features": features,
            "timestamp": np.asarray([timestamp], dtype=np.float32),
            "ego_pose": np.asarray(ego_pose, dtype=np.float32),
            "ego_pose_inv": np.asarray(ego_pose_inv, dtype=np.float32),
            "prev_exists": np.asarray(
                [1.0 if self._seen_frame else 0.0], dtype=np.float32
            ),
            **self.memory,
        }
        outputs = self.head(feeds)

        # Carry the refreshed memory into the next call.
        self.memory = {
            f"pre_{name}": outputs[f"post_{name}"] for name in MEMORY_TENSORS
        }
        self._seen_frame = True
        return {
            "class_logits": outputs["class_logits"],
            "bbox_predictions": outputs["bbox_predictions"],
        }
