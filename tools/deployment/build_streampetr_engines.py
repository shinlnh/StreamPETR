#!/usr/bin/env python3
"""Build the StreamPETR TensorRT engines from the exported ONNX graphs.

Uses the TensorRT Python API rather than a local trtexec binary so the engines
are serialized by the same library that will deserialize them: a pip-installed
``tensorrt`` of the same version can load the result on another machine.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import tensorrt as trt


def build(onnx_path: Path, engine_path: Path, workspace_gb: int, fp16: bool) -> None:
    logger = trt.Logger(trt.Logger.WARNING)
    trt.init_libnvinfer_plugins(logger, "")
    builder = trt.Builder(logger)
    network = builder.create_network(
        1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH)
    )
    parser = trt.OnnxParser(network, logger)
    with open(onnx_path, "rb") as stream:
        if not parser.parse(stream.read()):
            for index in range(parser.num_errors):
                print(f"  [onnx] {parser.get_error(index)}")
            raise RuntimeError(f"failed to parse {onnx_path}")

    config = builder.create_builder_config()
    config.set_memory_pool_limit(
        trt.MemoryPoolType.WORKSPACE, workspace_gb * (1 << 30)
    )
    config.builder_optimization_level = 5
    if fp16:
        if not builder.platform_has_fast_fp16:
            raise RuntimeError("this GPU has no fast FP16 support")
        config.set_flag(trt.BuilderFlag.FP16)

    print(f"[build] {onnx_path.name} -> {engine_path.name}")
    serialized = builder.build_serialized_network(network, config)
    if serialized is None:
        raise RuntimeError(f"engine build failed for {onnx_path}")
    engine_path.write_bytes(serialized)
    size_mib = engine_path.stat().st_size / (1024**2)
    print(f"[done]  {engine_path.name}: {size_mib:.1f} MiB")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--deployment-dir",
        default="work_dirs/stream_petr_r50_nucarla_town04/deployment",
    )
    parser.add_argument("--cameras", type=int, default=6)
    parser.add_argument("--fp32", action="store_true", help="build without FP16")
    args = parser.parse_args()

    directory = Path(args.deployment_dir)
    prefix = f"stream_petr_{args.cameras}cam"
    precision = "fp32" if args.fp32 else "fp16"
    print(f"TensorRT {trt.__version__}")
    for stem, workspace in (("encoder", 4), ("temporal_head", 6)):
        build(
            directory / f"{prefix}_{stem}.onnx",
            directory / f"{prefix}_{stem}_{precision}.engine",
            workspace,
            not args.fp32,
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
