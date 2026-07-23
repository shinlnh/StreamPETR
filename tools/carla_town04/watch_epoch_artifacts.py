#!/usr/bin/env python3
"""Persist manifests for rolling checkpoints and archive completed epochs."""

import argparse
import hashlib
import json
import math
import os
import re
import shutil
import time
from datetime import datetime, timezone
from pathlib import Path


IGNORED_METRIC_KEYS = {"mode", "epoch", "iter"}
CHECKPOINT_RE = re.compile(r"^iter_(\d+)\.pth$")


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--work-dir", type=Path, required=True)
    parser.add_argument("--iters-per-epoch", type=int, required=True)
    parser.add_argument("--epochs", type=int, required=True)
    parser.add_argument("--checkpoint-interval", type=int, required=True)
    parser.add_argument("--poll-seconds", type=int, default=30)
    parser.add_argument("--once", action="store_true")
    return parser.parse_args()


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(8 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_train_records(work_dir):
    records = []
    for log_path in sorted(work_dir.glob("*.log.json")):
        try:
            with log_path.open("r", encoding="utf-8") as handle:
                for line in handle:
                    try:
                        record = json.loads(line)
                    except json.JSONDecodeError:
                        continue
                    if record.get("mode") == "train" and "iter" in record:
                        records.append(record)
        except OSError:
            continue
    return records


def summarize_metrics(records, lower_iteration, upper_iteration):
    selected = [
        record
        for record in records
        if lower_iteration < int(record.get("iter", -1)) <= upper_iteration
    ]
    values = {}
    for record in selected:
        for key, value in record.items():
            if key in IGNORED_METRIC_KEYS or not isinstance(value, (int, float)):
                continue
            if not math.isfinite(float(value)):
                continue
            values.setdefault(key, []).append(float(value))

    summary = {}
    for key, samples in sorted(values.items()):
        summary[key] = {
            "count": len(samples),
            "mean": sum(samples) / len(samples),
            "min": min(samples),
            "max": max(samples),
            "last": samples[-1],
        }
    return selected, summary


def atomic_json_dump(path, payload):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, sort_keys=True, allow_nan=False)
        handle.write("\n")
    os.replace(str(temporary), str(path))


def atomic_copy(source, destination):
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_suffix(destination.suffix + ".tmp")
    shutil.copy2(str(source), str(temporary))
    os.replace(str(temporary), str(destination))


def checkpoint_is_stable(path):
    try:
        first = path.stat()
        if first.st_size <= 0 or time.time() - first.st_mtime < 10:
            return False
        time.sleep(2)
        second = path.stat()
        return first.st_size == second.st_size and first.st_mtime == second.st_mtime
    except OSError:
        return False


def checkpoint_paths(work_dir):
    checkpoints = []
    for path in work_dir.glob("iter_*.pth"):
        match = CHECKPOINT_RE.match(path.name)
        if match:
            checkpoints.append((int(match.group(1)), path))
    return sorted(checkpoints)


def checkpoint_manifest(args, records, iteration, checkpoint):
    epoch_number = (iteration - 1) // args.iters_per_epoch + 1
    epoch_iteration = iteration - (epoch_number - 1) * args.iters_per_epoch
    lower = max(0, iteration - args.checkpoint_interval)
    selected, metrics = summarize_metrics(records, lower, iteration)
    return {
        "kind": "rolling_checkpoint",
        "iteration": iteration,
        "epoch": epoch_number,
        "epoch_iteration": epoch_iteration,
        "epoch_progress": min(1.0, epoch_iteration / args.iters_per_epoch),
        "checkpoint": checkpoint.name,
        "checkpoint_bytes": checkpoint.stat().st_size,
        "checkpoint_sha256": sha256(checkpoint),
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "metrics_window": {
            "lower_iteration_exclusive": lower,
            "upper_iteration_inclusive": iteration,
            "log_points": len(selected),
            "metrics": metrics,
        },
        "contents": {
            "model": True,
            "optimizer": True,
            "fp16_loss_scaler": False,
            "training_precision": "fp32",
            "runner_metadata": True,
            "config_metadata": True,
        },
    }


def archive_epoch(args, records, iteration, checkpoint, rolling_payload):
    if iteration % args.iters_per_epoch:
        return

    epoch = iteration // args.iters_per_epoch
    archive_dir = args.work_dir / "epochs"
    archive = archive_dir / "epoch_{:02d}_iter_{}.pth".format(epoch, iteration)
    if not archive.is_file() or archive.stat().st_size != checkpoint.stat().st_size:
        atomic_copy(checkpoint, archive)

    lower = (epoch - 1) * args.iters_per_epoch
    selected, metrics = summarize_metrics(records, lower, iteration)
    payload = dict(rolling_payload)
    payload.update(
        {
            "kind": "completed_epoch",
            "epoch": epoch,
            "epoch_iteration": args.iters_per_epoch,
            "epoch_progress": 1.0,
            "checkpoint": str(archive.relative_to(args.work_dir)),
            "checkpoint_bytes": archive.stat().st_size,
            "checkpoint_sha256": sha256(archive),
            "epoch_metrics": {
                "lower_iteration_exclusive": lower,
                "upper_iteration_inclusive": iteration,
                "log_points": len(selected),
                "metrics": metrics,
            },
        }
    )
    manifest = archive_dir / "epoch_{:02d}_manifest.json".format(epoch)
    atomic_json_dump(manifest, payload)
    print(
        "[checkpoint-artifacts] archived epoch={} checkpoint={} sha256={}".format(
            epoch, archive.name, payload["checkpoint_sha256"]
        ),
        flush=True,
    )


def build_available_manifests(args):
    records = load_train_records(args.work_dir)
    newest_payload = None
    for iteration, checkpoint in checkpoint_paths(args.work_dir):
        manifest = args.work_dir / "checkpoint_iter_{:06d}_manifest.json".format(
            iteration
        )
        if manifest.is_file():
            try:
                payload = json.loads(manifest.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError):
                payload = None
        else:
            payload = None

        if payload is None:
            if not checkpoint_is_stable(checkpoint):
                continue
            payload = checkpoint_manifest(args, records, iteration, checkpoint)
            atomic_json_dump(manifest, payload)
            print(
                "[checkpoint-artifacts] iteration={} checkpoint={} sha256={}".format(
                    iteration, checkpoint.name, payload["checkpoint_sha256"]
                ),
                flush=True,
            )

        archive_epoch(args, records, iteration, checkpoint, payload)
        if newest_payload is None or iteration > newest_payload["iteration"]:
            newest_payload = payload

    if newest_payload is not None:
        atomic_json_dump(args.work_dir / "latest_checkpoint.json", newest_payload)

    completed_epochs = len(list((args.work_dir / "epochs").glob("epoch_*_manifest.json")))
    return completed_epochs


def main():
    args = parse_args()
    if (
        args.iters_per_epoch <= 0
        or args.epochs <= 0
        or args.checkpoint_interval <= 0
    ):
        raise ValueError("Epoch and checkpoint intervals must be positive")
    if args.iters_per_epoch % args.checkpoint_interval:
        raise ValueError(
            "checkpoint_interval must divide iters_per_epoch so every epoch is saved"
        )
    args.work_dir.mkdir(parents=True, exist_ok=True)

    while True:
        completed_epochs = build_available_manifests(args)
        if args.once or completed_epochs >= args.epochs:
            break
        time.sleep(args.poll_seconds)


if __name__ == "__main__":
    main()
