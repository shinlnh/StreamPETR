#!/usr/bin/env python3
"""Export the trained CARLA four-camera StreamPETR model to ONNX.

The deployment is intentionally split into two fixed-shape networks:

* encoder: normalized four-camera images -> image features
* temporal head: image features + temporal state -> detections + next state

The position embedding and camera cone are baked from the validation rig. They
are invariant for the fixed four-camera CARLA calibration used by this model.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib
import json
import platform
from pathlib import Path
from typing import Any, Dict, Iterable, Tuple

import numpy as np
import onnx
import torch
import torch.nn as nn
from mmcv import Config
from mmcv.runner import load_checkpoint
from mmdet.models.utils.transformer import inverse_sigmoid
from mmdet3d.datasets import build_dataset
from mmdet3d.models import build_detector

from projects.mmdet3d_plugin.models.utils.misc import (
    topk_gather,
    transform_reference_points,
)
from projects.mmdet3d_plugin.models.utils.positional_encoding import (
    pos2posemb3d,
)


DEFAULT_CONFIG = (
    "projects/configs/StreamPETR/stream_petr_r50_carla_4cam_finetune.py"
)
DEFAULT_CHECKPOINT = (
    "work_dirs/stream_petr_r50_carla_4cam_finetune/iter_16125.pth"
)
DEFAULT_OUTPUT_DIR = (
    "work_dirs/stream_petr_r50_carla_4cam_finetune/deployment"
)

ENCODER_INPUT_NAMES = ["images"]
ENCODER_OUTPUT_NAMES = ["image_features"]
HEAD_INPUT_NAMES = [
    "image_features",
    "timestamp",
    "ego_pose",
    "ego_pose_inv",
    "prev_exists",
    "pre_memory_embedding",
    "pre_memory_reference_point",
    "pre_memory_timestamp",
    "pre_memory_egopose",
    "pre_memory_velocity",
]
HEAD_OUTPUT_NAMES = [
    "class_logits",
    "bbox_predictions",
    "post_memory_embedding",
    "post_memory_reference_point",
    "post_memory_timestamp",
    "post_memory_egopose",
    "post_memory_velocity",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=DEFAULT_CONFIG)
    parser.add_argument("--checkpoint", default=DEFAULT_CHECKPOINT)
    parser.add_argument("--output-dir", default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--opset", type=int, default=17)
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--sample-index", type=int, default=0)
    return parser.parse_args()


def unwrap_data_container(value: Any) -> Any:
    while isinstance(value, (list, tuple)):
        value = value[0]
    if hasattr(value, "data"):
        value = value.data
    while isinstance(value, (list, tuple)):
        value = value[0]
    return value


def add_batch_dim(value: torch.Tensor, device: torch.device) -> torch.Tensor:
    return value.detach().to(device=device).unsqueeze(0)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def tensor_spec(tensor: torch.Tensor) -> Dict[str, Any]:
    return {
        "shape": list(tensor.shape),
        "dtype": str(tensor.dtype).removeprefix("torch."),
    }


def save_raw_tensors(
    tensor_dir: Path,
    names: Iterable[str],
    tensors: Iterable[torch.Tensor],
) -> None:
    tensor_dir.mkdir(parents=True, exist_ok=True)
    for name, tensor in zip(names, tensors):
        array = tensor.detach().cpu().contiguous().numpy()
        array.tofile(tensor_dir / f"{name}.bin")


class CarlaStreamPETREncoder(nn.Module):
    """Backbone and neck without the detector's batch-one in-place squeeze."""

    def __init__(self, detector: nn.Module) -> None:
        super().__init__()
        self.img_backbone = detector.img_backbone
        self.img_neck = detector.img_neck
        self.position_level = detector.position_level

    def forward(self, images: torch.Tensor) -> torch.Tensor:
        batch, cameras, channels, height, width = images.shape
        flat_images = images.reshape(
            batch * cameras, channels, height, width
        )
        image_features = self.img_backbone(flat_images)
        if isinstance(image_features, dict):
            image_features = list(image_features.values())
        image_features = self.img_neck(image_features)[self.position_level]
        _, feature_channels, feature_height, feature_width = (
            image_features.shape
        )
        return image_features.reshape(
            batch,
            cameras,
            feature_channels,
            feature_height,
            feature_width,
        )


class CarlaStreamPETRTemporalHead(nn.Module):
    """StreamPETR temporal head with explicit fixed-size recurrent state."""

    def __init__(
        self,
        head: nn.Module,
        position_embedding: torch.Tensor,
        cone: torch.Tensor,
    ) -> None:
        super().__init__()
        self.head = head
        self.memory_len = int(head.memory_len)
        self.num_propagated = int(head.num_propagated)
        self.topk_proposals = int(head.topk_proposals)
        self.register_buffer(
            "fixed_position_embedding", position_embedding.detach().clone()
        )
        self.register_buffer("fixed_cone", cone.detach().clone())

    @staticmethod
    def _select_topk(tensor: torch.Tensor, indexes: torch.Tensor) -> torch.Tensor:
        return topk_gather(tensor, indexes)

    def _pre_update_memory(
        self,
        timestamp: torch.Tensor,
        ego_pose_inv: torch.Tensor,
        prev_exists: torch.Tensor,
        memory_embedding: torch.Tensor,
        memory_reference_point: torch.Tensor,
        memory_timestamp: torch.Tensor,
        memory_egopose: torch.Tensor,
        memory_velocity: torch.Tensor,
    ) -> Tuple[torch.Tensor, ...]:
        head = self.head
        exists = prev_exists.reshape(-1, 1, 1)

        memory_timestamp = (
            memory_timestamp[:, : self.memory_len]
            + timestamp.reshape(-1, 1, 1)
        )
        memory_egopose = (
            ego_pose_inv.unsqueeze(1)
            @ memory_egopose[:, : self.memory_len]
        )
        memory_reference_point = transform_reference_points(
            memory_reference_point[:, : self.memory_len],
            ego_pose_inv,
            reverse=False,
        )

        memory_embedding = (
            memory_embedding[:, : self.memory_len] * exists
        )
        memory_reference_point = memory_reference_point * exists
        memory_timestamp = memory_timestamp * exists
        memory_egopose = memory_egopose * exists.unsqueeze(-1)
        memory_velocity = memory_velocity[:, : self.memory_len] * exists

        if self.num_propagated > 0:
            reset = 1.0 - exists
            pseudo = head.pseudo_reference_points.weight
            pseudo = (
                pseudo * (head.pc_range[3:6] - head.pc_range[0:3])
                + head.pc_range[0:3]
            ).unsqueeze(0)
            ref_front = (
                memory_reference_point[:, : self.num_propagated]
                + reset * pseudo
            )
            memory_reference_point = torch.cat(
                [
                    ref_front,
                    memory_reference_point[:, self.num_propagated :],
                ],
                dim=1,
            )

            identity = torch.eye(
                4,
                dtype=memory_egopose.dtype,
                device=memory_egopose.device,
            ).reshape(1, 1, 4, 4)
            ego_front = (
                memory_egopose[:, : self.num_propagated]
                + reset.unsqueeze(-1) * identity
            )
            memory_egopose = torch.cat(
                [
                    ego_front,
                    memory_egopose[:, self.num_propagated :],
                ],
                dim=1,
            )

        return (
            memory_embedding,
            memory_reference_point,
            memory_timestamp,
            memory_egopose,
            memory_velocity,
        )

    def _post_update_memory(
        self,
        timestamp: torch.Tensor,
        ego_pose: torch.Tensor,
        class_logits: torch.Tensor,
        bbox_predictions: torch.Tensor,
        decoder_output: torch.Tensor,
        memory_embedding: torch.Tensor,
        memory_reference_point: torch.Tensor,
        memory_timestamp: torch.Tensor,
        memory_egopose: torch.Tensor,
        memory_velocity: torch.Tensor,
    ) -> Tuple[torch.Tensor, ...]:
        proposal_scores = class_logits.sigmoid().max(
            dim=-1, keepdim=True
        ).values
        _, topk_indexes = torch.topk(
            proposal_scores, self.topk_proposals, dim=1
        )

        new_reference = self._select_topk(
            bbox_predictions[..., :3], topk_indexes
        )
        new_velocity = self._select_topk(
            bbox_predictions[..., -2:], topk_indexes
        )
        new_embedding = self._select_topk(
            decoder_output, topk_indexes
        )
        new_timestamp = torch.zeros_like(
            proposal_scores, dtype=memory_timestamp.dtype
        )
        new_timestamp = self._select_topk(new_timestamp, topk_indexes)

        batch = class_logits.shape[0]
        query_count = class_logits.shape[1]
        new_egopose = torch.eye(
            4,
            dtype=ego_pose.dtype,
            device=ego_pose.device,
        ).reshape(1, 1, 4, 4).repeat(batch, query_count, 1, 1)
        new_egopose = self._select_topk(new_egopose, topk_indexes)

        memory_embedding = torch.cat(
            [new_embedding, memory_embedding], dim=1
        )[:, : self.memory_len]
        memory_reference_point = torch.cat(
            [new_reference, memory_reference_point], dim=1
        )[:, : self.memory_len]
        memory_timestamp = torch.cat(
            [new_timestamp, memory_timestamp], dim=1
        )[:, : self.memory_len]
        memory_egopose = torch.cat(
            [new_egopose, memory_egopose], dim=1
        )[:, : self.memory_len]
        memory_velocity = torch.cat(
            [new_velocity, memory_velocity], dim=1
        )[:, : self.memory_len]

        memory_reference_point = transform_reference_points(
            memory_reference_point, ego_pose, reverse=False
        )
        memory_timestamp = (
            memory_timestamp - timestamp.reshape(-1, 1, 1)
        )
        memory_egopose = ego_pose.unsqueeze(1) @ memory_egopose

        return (
            memory_embedding,
            memory_reference_point,
            memory_timestamp,
            memory_egopose,
            memory_velocity,
        )

    def forward(
        self,
        image_features: torch.Tensor,
        timestamp: torch.Tensor,
        ego_pose: torch.Tensor,
        ego_pose_inv: torch.Tensor,
        prev_exists: torch.Tensor,
        pre_memory_embedding: torch.Tensor,
        pre_memory_reference_point: torch.Tensor,
        pre_memory_timestamp: torch.Tensor,
        pre_memory_egopose: torch.Tensor,
        pre_memory_velocity: torch.Tensor,
    ) -> Tuple[torch.Tensor, ...]:
        head = self.head
        (
            memory_embedding,
            memory_reference_point,
            memory_timestamp,
            memory_egopose,
            memory_velocity,
        ) = self._pre_update_memory(
            timestamp,
            ego_pose_inv,
            prev_exists,
            pre_memory_embedding,
            pre_memory_reference_point,
            pre_memory_timestamp,
            pre_memory_egopose,
            pre_memory_velocity,
        )

        # temporal_alignment reads the state through the original head.
        head.memory_embedding = memory_embedding
        head.memory_reference_point = memory_reference_point
        head.memory_timestamp = memory_timestamp
        head.memory_egopose = memory_egopose
        head.memory_velo = memory_velocity

        batch, cameras, channels, height, width = image_features.shape
        token_count = cameras * height * width
        image_memory = image_features.permute(
            0, 1, 3, 4, 2
        ).reshape(batch, token_count, channels)
        image_memory = head.memory_embed(image_memory)
        image_memory = head.spatial_alignment(
            image_memory, self.fixed_cone
        )
        position_embedding = head.featurized_pe(
            self.fixed_position_embedding, image_memory
        )

        reference_points = head.reference_points.weight.unsqueeze(0).repeat(
            batch, 1, 1
        )
        query_position = head.query_embedding(
            pos2posemb3d(reference_points)
        )
        target = torch.zeros_like(query_position)
        (
            target,
            query_position,
            reference_points,
            temporal_memory,
            temporal_position,
            _,
        ) = head.temporal_alignment(
            query_position, target, reference_points
        )

        decoder_outputs, _ = head.transformer(
            image_memory,
            target,
            query_position,
            position_embedding,
            None,
            temporal_memory,
            temporal_position,
        )
        final_decoder_output = decoder_outputs[-1]
        reference_logits = inverse_sigmoid(reference_points)
        class_logits = head.cls_branches[-1](final_decoder_output)
        bbox_predictions = head.reg_branches[-1](final_decoder_output)
        bbox_predictions[..., 0:3] = (
            bbox_predictions[..., 0:3] + reference_logits[..., 0:3]
        ).sigmoid()
        bbox_predictions[..., 0:3] = (
            bbox_predictions[..., 0:3]
            * (head.pc_range[3:6] - head.pc_range[0:3])
            + head.pc_range[0:3]
        )

        next_state = self._post_update_memory(
            timestamp,
            ego_pose,
            class_logits,
            bbox_predictions,
            final_decoder_output,
            memory_embedding,
            memory_reference_point,
            memory_timestamp,
            memory_egopose,
            memory_velocity,
        )
        return (class_logits, bbox_predictions, *next_state)


def prepare_model_and_sample(
    config_path: Path,
    checkpoint_path: Path,
    sample_index: int,
    device: torch.device,
) -> Tuple[nn.Module, Dict[str, Any], Dict[str, torch.Tensor]]:
    cfg = Config.fromfile(str(config_path))
    importlib.import_module("projects.mmdet3d_plugin")
    cfg.model.pretrained = None
    cfg.model.train_cfg = None
    cfg.data.test.test_mode = True

    dataset = build_dataset(cfg.data.test)
    sample = dataset[sample_index]
    metadata = unwrap_data_container(sample["img_metas"])
    tensors = {
        key: unwrap_data_container(sample[key])
        for key in [
            "img",
            "lidar2img",
            "intrinsics",
            "extrinsics",
            "timestamp",
            "ego_pose",
            "ego_pose_inv",
        ]
    }
    tensors["img"] = add_batch_dim(tensors["img"], device).float()
    for key in [
        "lidar2img",
        "intrinsics",
        "extrinsics",
        "ego_pose",
        "ego_pose_inv",
    ]:
        tensors[key] = add_batch_dim(tensors[key], device).float()
    tensors["timestamp"] = (
        tensors["timestamp"].reshape(1).to(device=device).float()
    )

    model = build_detector(cfg.model, test_cfg=cfg.get("test_cfg"))
    load_checkpoint(model, str(checkpoint_path), map_location="cpu")
    model = model.to(device=device).float().eval()
    model.pts_bbox_head.with_dn = False
    return model, metadata, tensors


def make_export_inputs(
    model: nn.Module,
    metadata: Dict[str, Any],
    sample: Dict[str, torch.Tensor],
) -> Tuple[nn.Module, nn.Module, Tuple[torch.Tensor, ...], Tuple[torch.Tensor, ...]]:
    encoder = CarlaStreamPETREncoder(model).eval()
    with torch.no_grad():
        image_features = encoder(sample["img"])
        location = model.prepare_location(
            [metadata], img_feats=image_features
        )
        position_data = {
            key: sample[key]
            for key in [
                "lidar2img",
                "intrinsics",
                "extrinsics",
                "timestamp",
                "ego_pose",
                "ego_pose_inv",
            ]
        }
        position_embedding, cone = (
            model.pts_bbox_head.position_embeding(
                position_data, location, None, [metadata]
            )
        )

    head = CarlaStreamPETRTemporalHead(
        model.pts_bbox_head, position_embedding, cone
    ).eval()
    memory_len = int(model.pts_bbox_head.memory_len)
    embed_dims = int(model.pts_bbox_head.embed_dims)
    device = sample["img"].device
    dtype = sample["img"].dtype
    batch = 1

    encoder_inputs = (sample["img"],)
    head_inputs = (
        image_features,
        sample["timestamp"],
        sample["ego_pose"],
        sample["ego_pose_inv"],
        torch.zeros(batch, dtype=dtype, device=device),
        torch.zeros(
            batch, memory_len, embed_dims, dtype=dtype, device=device
        ),
        torch.zeros(batch, memory_len, 3, dtype=dtype, device=device),
        torch.zeros(batch, memory_len, 1, dtype=dtype, device=device),
        torch.zeros(
            batch, memory_len, 4, 4, dtype=dtype, device=device
        ),
        torch.zeros(batch, memory_len, 2, dtype=dtype, device=device),
    )
    return encoder, head, encoder_inputs, head_inputs


def validate_wrapper_against_native_head(
    model: nn.Module,
    metadata: Dict[str, Any],
    sample: Dict[str, torch.Tensor],
    image_features: torch.Tensor,
    wrapper: nn.Module,
    wrapper_inputs: Tuple[torch.Tensor, ...],
) -> Dict[str, float]:
    native_head = model.pts_bbox_head
    native_head.reset_memory()
    native_data = {
        key: sample[key]
        for key in [
            "lidar2img",
            "intrinsics",
            "extrinsics",
            "timestamp",
            "ego_pose",
            "ego_pose_inv",
        ]
    }
    native_data["img_feats"] = image_features
    native_data["prev_exists"] = wrapper_inputs[4]
    native_location = model.prepare_location(
        [metadata], img_feats=image_features
    )
    with torch.inference_mode():
        native_outputs = native_head(
            native_location, [metadata], None, **native_data
        )
        wrapper_outputs = wrapper(*wrapper_inputs)
    native_head.reset_memory()

    class_diff = (
        native_outputs["all_cls_scores"][-1] - wrapper_outputs[0]
    ).abs()
    bbox_diff = (
        native_outputs["all_bbox_preds"][-1] - wrapper_outputs[1]
    ).abs()
    result = {
        "class_logits_max_abs": float(class_diff.max().item()),
        "class_logits_mean_abs": float(class_diff.mean().item()),
        "bbox_predictions_max_abs": float(bbox_diff.max().item()),
        "bbox_predictions_mean_abs": float(bbox_diff.mean().item()),
    }
    print(
        "[native-check] "
        f"class max={result['class_logits_max_abs']:.3e}, "
        f"bbox max={result['bbox_predictions_max_abs']:.3e}"
    )
    return result


def export_onnx(
    module: nn.Module,
    inputs: Tuple[torch.Tensor, ...],
    output_path: Path,
    input_names: Iterable[str],
    output_names: Iterable[str],
    opset: int,
) -> None:
    print(f"[export] {output_path}")
    with torch.inference_mode():
        torch.onnx.export(
            module,
            inputs,
            str(output_path),
            input_names=list(input_names),
            output_names=list(output_names),
            opset_version=opset,
            do_constant_folding=True,
            dynamo=False,
            verbose=False,
        )
    onnx.checker.check_model(str(output_path), full_check=True)
    print(
        f"[verified] {output_path.name}: "
        f"{output_path.stat().st_size / (1024 ** 2):.1f} MiB"
    )


def main() -> None:
    args = parse_args()
    repo_root = Path.cwd().resolve()
    config_path = (repo_root / args.config).resolve()
    checkpoint_path = (repo_root / args.checkpoint).resolve()
    output_dir = (repo_root / args.output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    if not checkpoint_path.is_file():
        raise FileNotFoundError(checkpoint_path)
    device = torch.device(args.device)
    if device.type == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("CUDA was requested but is unavailable")
    if hasattr(torch.backends, "mha"):
        torch.backends.mha.set_fastpath_enabled(False)
    torch.manual_seed(0)
    np.random.seed(0)

    print(f"[load] checkpoint: {checkpoint_path}")
    model, metadata, sample = prepare_model_and_sample(
        config_path, checkpoint_path, args.sample_index, device
    )
    encoder, head, encoder_inputs, head_inputs = make_export_inputs(
        model, metadata, sample
    )

    with torch.inference_mode():
        encoder_output = encoder(*encoder_inputs)
    native_check = validate_wrapper_against_native_head(
        model,
        metadata,
        sample,
        encoder_output,
        head,
        head_inputs,
    )
    with torch.inference_mode():
        head_outputs = head(*head_inputs)
    print(f"[shape] encoder output: {tuple(encoder_output.shape)}")
    print(
        "[shape] head outputs: "
        + ", ".join(str(tuple(item.shape)) for item in head_outputs)
    )

    # Name the artefacts after the rig actually exported: the same script now
    # serves the four-camera CARLA model and the six-camera nuCarla one.
    camera_count = encoder_inputs[0].shape[1]
    prefix = f"stream_petr_{camera_count}cam"
    encoder_path = output_dir / f"{prefix}_encoder.onnx"
    head_path = output_dir / f"{prefix}_temporal_head.onnx"
    export_onnx(
        encoder,
        encoder_inputs,
        encoder_path,
        ENCODER_INPUT_NAMES,
        ENCODER_OUTPUT_NAMES,
        args.opset,
    )
    # PyTorch's legacy ONNX constant-folding pass requires every parameter and
    # constant to share a device. The temporal graph contains CPU-created
    # positional constants in upstream MMCV, so trace this half on CPU, as the
    # NVIDIA StreamPETR conversion path does. This does not affect the exported
    # device-agnostic ONNX graph or the TensorRT engine that consumes it.
    head = head.cpu()
    head_export_inputs = tuple(tensor.cpu() for tensor in head_inputs)
    export_onnx(
        head,
        head_export_inputs,
        head_path,
        HEAD_INPUT_NAMES,
        HEAD_OUTPUT_NAMES,
        args.opset,
    )

    save_raw_tensors(
        output_dir / "test_inputs", ENCODER_INPUT_NAMES, encoder_inputs
    )
    save_raw_tensors(
        output_dir / "test_inputs", HEAD_INPUT_NAMES, head_inputs
    )
    save_raw_tensors(
        output_dir / "reference_outputs",
        ENCODER_OUTPUT_NAMES,
        (encoder_output,),
    )
    save_raw_tensors(
        output_dir / "reference_outputs",
        HEAD_OUTPUT_NAMES,
        head_outputs,
    )
    input_specs = {
        "encoder": {
            name: tensor_spec(tensor)
            for name, tensor in zip(ENCODER_INPUT_NAMES, encoder_inputs)
        },
        "temporal_head": {
            name: tensor_spec(tensor)
            for name, tensor in zip(HEAD_INPUT_NAMES, head_inputs)
        },
    }
    output_specs = {
        name: tensor_spec(tensor)
        for name, tensor in zip(HEAD_OUTPUT_NAMES, head_outputs)
    }
    manifest = {
        "format_version": 1,
        "config": str(config_path.relative_to(repo_root)),
        "checkpoint": str(checkpoint_path.relative_to(repo_root)),
        "checkpoint_sha256": sha256_file(checkpoint_path),
        "onnx_opset": args.opset,
        "classes": [
            "car",
            "truck",
            "bus",
            "motorcycle",
            "bicycle",
            "pedestrian",
        ],
        "input_preprocessing": {
            "camera_count": 4,
            "shape_nchw": [3, 256, 704],
            "color": "RGB",
            "mean": [123.675, 116.28, 103.53],
            "std": [58.395, 57.12, 57.375],
            "note": "Use the deterministic test pipeline from the config.",
        },
        "temporal_state": {
            "memory_length": int(model.pts_bbox_head.memory_len),
            "timestamp_dtype": "float32",
            "first_frame_prev_exists": 0.0,
            "subsequent_frame_prev_exists": 1.0,
            "note": (
                "Feed every post_memory_* output into the matching "
                "pre_memory_* input on the next frame."
            ),
        },
        "fixed_calibration": {
            "sample_index": args.sample_index,
            "scene_token": metadata.get("scene_token"),
            "note": (
                "Position embedding and camera cone are baked for the "
                "four-camera CARLA rig used by this dataset."
            ),
        },
        "inputs": input_specs,
        "outputs": {"temporal_head": output_specs},
        "pytorch_native_wrapper_check": native_check,
        "environment": {
            "python": platform.python_version(),
            "pytorch": torch.__version__,
            "cuda_runtime": torch.version.cuda,
            "gpu": (
                torch.cuda.get_device_name(device)
                if device.type == "cuda"
                else None
            ),
        },
    }
    manifest_path = output_dir / "deployment_manifest.json"
    manifest_path.write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(f"[done] manifest: {manifest_path}")


if __name__ == "__main__":
    main()
