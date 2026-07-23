#!/usr/bin/env python3
"""Render CARLA Town04 ground truth and StreamPETR predictions.

Each output PNG contains the six original camera views, a bird's-eye view and
a perspective 3D view. Ground truth is green; predictions are magenta/red.
"""

import argparse
import html
import pickle
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image, ImageDraw


CAMERA_LAYOUT = (
    "CAM_FRONT_LEFT",
    "CAM_FRONT",
    "CAM_FRONT_RIGHT",
    "CAM_BACK_LEFT",
    "CAM_BACK",
    "CAM_BACK_RIGHT",
)
BOX_EDGES = (
    (0, 1), (1, 2), (2, 3), (3, 0),
    (4, 5), (5, 6), (6, 7), (7, 4),
    (0, 4), (1, 5), (2, 6), (3, 7),
)
GT_COLOR = "#33ff66"
PRED_COLOR = "#ff3366"
OCCLUSION_MASK_SCALE = 0.125

# Classes that the Town04 collector can obtain from CARLA actors. Keeping the
# nuScenes-only classes in a zero-shot display (barrier, trailer, cone, ...)
# adds a large amount of noise.
CARLA_ACTOR_CLASSES = frozenset(
    ("car", "truck", "bus", "motorcycle", "bicycle", "pedestrian")
)

# Broad physical sanity limits in metres. They deliberately leave room for
# variation between CARLA blueprints and reject only clearly impossible boxes.
CLASS_DIMENSION_LIMITS = {
    "car": ((2.0, 7.5), (1.2, 3.2), (0.8, 3.0)),
    "truck": ((3.0, 18.0), (1.4, 4.5), (1.3, 5.5)),
    "bus": ((5.0, 18.0), (1.7, 4.5), (2.0, 5.5)),
    "motorcycle": ((0.8, 4.0), (0.25, 1.8), (0.5, 3.0)),
    "bicycle": ((0.8, 3.5), (0.25, 1.8), (0.5, 3.0)),
    "pedestrian": ((0.2, 1.5), (0.2, 1.5), (0.8, 2.7)),
}


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("infos", help="Town04 validation info PKL")
    parser.add_argument("predictions", help="prediction PKL produced by infer.py")
    parser.add_argument("output_dir", help="directory for rendered PNG files")
    parser.add_argument(
        "--indices",
        type=int,
        nargs="+",
        default=[0, 10, 20, 39, 40, 50, 60, 79],
        help="validation indices to render",
    )
    parser.add_argument(
        "--score-threshold",
        type=float,
        default=0.40,
        help="minimum prediction confidence (0.40 is recommended for zero-shot)",
    )
    parser.add_argument("--max-predictions", type=int, default=20)
    parser.add_argument("--range", dest="view_range", type=float, default=55.0)
    parser.add_argument(
        "--min-visible-ratio",
        type=float,
        default=0.08,
        help=(
            "minimum projected cuboid area not covered by a nearer cuboid; "
            "set 0 to disable camera occlusion filtering"
        ),
    )
    parser.add_argument(
        "--raw-predictions",
        action="store_true",
        help="disable CARLA class/size/range and duplicate prediction filters",
    )
    parser.add_argument("--dpi", type=int, default=130)
    return parser.parse_args()


def load_pickle(path):
    with Path(path).open("rb") as handle:
        return pickle.load(handle)


def box_corners(box, z_is_bottom=False):
    """Return eight corners for [x, y, z, dx, dy, dz, yaw, ...]."""
    x, y, z, dx, dy, dz, yaw = np.asarray(box[:7], dtype=np.float64)
    if z_is_bottom:
        z += dz * 0.5
    local = np.array(
        [
            [dx / 2, dy / 2, -dz / 2],
            [dx / 2, -dy / 2, -dz / 2],
            [-dx / 2, -dy / 2, -dz / 2],
            [-dx / 2, dy / 2, -dz / 2],
            [dx / 2, dy / 2, dz / 2],
            [dx / 2, -dy / 2, dz / 2],
            [-dx / 2, -dy / 2, dz / 2],
            [-dx / 2, dy / 2, dz / 2],
        ],
        dtype=np.float64,
    )
    cosine, sine = np.cos(yaw), np.sin(yaw)
    rotation = np.array([[cosine, -sine], [sine, cosine]])
    local[:, :2] = local[:, :2] @ rotation.T
    local += np.array([x, y, z])
    return local


def camera_projection(corners, camera):
    rotation = np.asarray(camera["sensor2lidar_rotation"], dtype=np.float64)
    translation = np.asarray(camera["sensor2lidar_translation"], dtype=np.float64)
    intrinsic = np.asarray(camera["cam_intrinsic"], dtype=np.float64)
    camera_points = (rotation.T @ (corners - translation).T).T
    depth = camera_points[:, 2]
    pixels_h = (intrinsic @ camera_points.T).T
    pixels = np.full((len(corners), 2), np.nan, dtype=np.float64)
    valid = depth > 0.1
    pixels[valid] = pixels_h[valid, :2] / depth[valid, None]
    return pixels, valid, depth


def draw_projected_box(
    ax, corners, camera, image_size, color, text, linewidth, alpha
):
    pixels, valid, _ = camera_projection(corners, camera)
    drew_edge = False
    for start, end in BOX_EDGES:
        if valid[start] and valid[end]:
            ax.plot(
                pixels[[start, end], 0],
                pixels[[start, end], 1],
                color=color,
                linewidth=linewidth,
                alpha=alpha,
            )
            drew_edge = True
    if drew_edge:
        center = corners.mean(axis=0, keepdims=True)
        center_px, center_valid, _ = camera_projection(center, camera)
        x, y = center_px[0]
        width, height = image_size
        if center_valid[0] and 0 <= x < width and 0 <= y < height:
            ax.text(
                x,
                y,
                text,
                color=color,
                fontsize=5.5,
                bbox=dict(facecolor="black", alpha=0.45, edgecolor="none", pad=0.6),
                clip_on=True,
            )


def draw_bev_box(ax, corners, color, text, alpha, linewidth):
    footprint = corners[[0, 1, 2, 3, 0]]
    ax.plot(footprint[:, 1], footprint[:, 0], color=color, alpha=alpha, linewidth=linewidth)
    center = corners.mean(axis=0)
    front = (corners[0] + corners[1]) * 0.5
    ax.plot([center[1], front[1]], [center[0], front[0]], color=color, alpha=alpha)
    ax.text(center[1], center[0], text, color=color, fontsize=5.5, alpha=alpha)


def draw_3d_box(ax, corners, color, alpha, linewidth):
    for start, end in BOX_EDGES:
        segment = corners[[start, end]]
        ax.plot(
            segment[:, 0],
            segment[:, 1],
            segment[:, 2],
            color=color,
            alpha=alpha,
            linewidth=linewidth,
        )


def convex_hull(points):
    """Return the 2D convex hull of a small point set."""
    points = sorted(set(map(tuple, np.asarray(points, dtype=np.float64))))
    if len(points) <= 1:
        return np.asarray(points, dtype=np.float64)

    def cross(origin, first, second):
        return (
            (first[0] - origin[0]) * (second[1] - origin[1])
            - (first[1] - origin[1]) * (second[0] - origin[0])
        )

    lower = []
    for point in points:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 0:
            lower.pop()
        lower.append(point)
    upper = []
    for point in reversed(points):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 0:
            upper.pop()
        upper.append(point)
    return np.asarray(lower[:-1] + upper[:-1], dtype=np.float64)


def projected_box_mask(
    corners, camera, image_size, mask_scale=OCCLUSION_MASK_SCALE
):
    """Rasterize a projected cuboid silhouette and return mask plus depth."""
    pixels, valid, depths = camera_projection(corners, camera)
    if np.count_nonzero(valid) < 3:
        return None
    points = convex_hull(pixels[valid])
    if len(points) < 3 or not np.all(np.isfinite(points)):
        return None

    width, height = image_size
    x1, y1 = points.min(axis=0)
    x2, y2 = points.max(axis=0)
    if x2 < 0 or y2 < 0 or x1 >= width or y1 >= height:
        return None

    mask_width = max(1, int(np.ceil(width * mask_scale)))
    mask_height = max(1, int(np.ceil(height * mask_scale)))
    # Clipping prevents a cuboid crossing the camera plane from producing huge
    # polygon coordinates inside Pillow.
    points[:, 0] = np.clip(points[:, 0], -width, 2 * width)
    points[:, 1] = np.clip(points[:, 1], -height, 2 * height)
    scaled = [tuple(point * mask_scale) for point in points]
    image = Image.new("1", (mask_width, mask_height), 0)
    ImageDraw.Draw(image).polygon(scaled, fill=1)
    mask = np.asarray(image, dtype=bool)
    if not np.any(mask):
        return None
    return mask, float(np.median(depths[valid]))


def visible_box_indices(corners_list, camera, image_size, min_visible_ratio):
    """Approximate camera visibility with a near-to-far cuboid z-buffer.

    RGB-only legacy captures do not contain depth maps. A low-resolution
    projected-cuboid buffer still removes boxes fully hidden behind a nearer
    labelled object while retaining partially visible objects.
    """
    if min_visible_ratio <= 0:
        return list(range(len(corners_list)))

    projected = []
    for index, corners in enumerate(corners_list):
        item = projected_box_mask(corners, camera, image_size)
        if item is not None:
            mask, depth = item
            projected.append((depth, index, mask))
    if not projected:
        return []

    occupied = np.zeros_like(projected[0][2], dtype=bool)
    visible = []
    for _, index, mask in sorted(projected, key=lambda item: item[0]):
        area = int(np.count_nonzero(mask))
        unoccluded = int(np.count_nonzero(mask & ~occupied))
        center = corners_list[index].mean(axis=0, keepdims=True)
        center_pixels, center_valid, _ = camera_projection(center, camera)
        center_occluded = False
        if center_valid[0]:
            center_x = int(center_pixels[0, 0] * OCCLUSION_MASK_SCALE)
            center_y = int(center_pixels[0, 1] * OCCLUSION_MASK_SCALE)
            if 0 <= center_x < occupied.shape[1] and 0 <= center_y < occupied.shape[0]:
                center_occluded = bool(occupied[center_y, center_x])
        # Two low-resolution pixels avoid retaining a box due solely to
        # rasterisation around an occluder's edge. Requiring a visible centre
        # also handles nested projected boxes where the cuboid hull sticks out
        # although the actual actor is completely hidden.
        if (
            not center_occluded
            and unoccluded >= 2
            and unoccluded / area >= min_visible_ratio
        ):
            visible.append(index)
        occupied |= mask
    return sorted(visible)


def dimensions_are_plausible(box, class_name):
    limits = CLASS_DIMENSION_LIMITS.get(class_name)
    if limits is None:
        return True
    dimensions = np.asarray(box[3:6], dtype=np.float64)
    return all(
        low <= value <= high
        for value, (low, high) in zip(dimensions, limits)
    )


def select_predictions(
    record, class_names, threshold, maximum, view_range, raw=False
):
    boxes = np.asarray(record["boxes_3d"])
    scores = np.asarray(record["scores_3d"])
    labels = np.asarray(record["labels_3d"])
    candidates = np.argsort(scores)[::-1]
    candidates = candidates[scores[candidates] >= threshold]
    stats = {
        "above_threshold": len(candidates),
        "class": 0,
        "size": 0,
        "range": 0,
        "duplicate": 0,
        "limit": 0,
    }
    if raw:
        stats["limit"] = max(0, len(candidates) - maximum)
        return candidates[:maximum], stats

    selected = []
    for index in candidates:
        label = int(labels[index])
        name = class_names[label] if 0 <= label < len(class_names) else str(label)
        box = boxes[index]
        if name not in CARLA_ACTOR_CLASSES:
            stats["class"] += 1
            continue
        if not np.all(np.isfinite(box[:7])) or not dimensions_are_plausible(box, name):
            stats["size"] += 1
            continue
        if np.linalg.norm(box[:2]) > view_range:
            stats["range"] += 1
            continue

        duplicate = False
        for kept in selected:
            if labels[kept] != label:
                continue
            kept_box = boxes[kept]
            suppression_distance = max(
                0.75,
                0.25
                * min(max(box[3], box[4]), max(kept_box[3], kept_box[4])),
            )
            if np.linalg.norm(box[:2] - kept_box[:2]) < suppression_distance:
                duplicate = True
                break
        if duplicate:
            stats["duplicate"] += 1
            continue
        selected.append(index)
        if len(selected) == maximum:
            break

    stats["limit"] = max(
        0,
        len(candidates)
        - stats["class"]
        - stats["size"]
        - stats["range"]
        - stats["duplicate"]
        - len(selected),
    )
    return np.asarray(selected, dtype=np.int64), stats


def resolve_image_path(path, infos_path):
    candidate = Path(path)
    if candidate.exists():
        return candidate
    relative = Path.cwd() / candidate
    if relative.exists():
        return relative
    return Path(infos_path).resolve().parent / candidate.name


def render_frame(info, prediction, class_names, infos_path, output, args):
    prediction_indices, filter_stats = select_predictions(
        prediction,
        class_names,
        args.score_threshold,
        args.max_predictions,
        args.view_range,
        raw=args.raw_predictions,
    )
    gt_boxes = np.asarray(info["gt_boxes"])
    gt_names = np.asarray(info["gt_names"])
    pred_boxes = np.asarray(prediction["boxes_3d"])[prediction_indices]
    pred_scores = np.asarray(prediction["scores_3d"])[prediction_indices]
    pred_labels = np.asarray(prediction["labels_3d"])[prediction_indices]

    gt_corners = [box_corners(box, z_is_bottom=False) for box in gt_boxes]
    pred_corners = [box_corners(box, z_is_bottom=True) for box in pred_boxes]

    figure = plt.figure(figsize=(20, 14), facecolor="#101216")
    grid = figure.add_gridspec(3, 6, height_ratios=[1.0, 1.0, 1.25])
    for camera_index, camera_name in enumerate(CAMERA_LAYOUT):
        row, column = divmod(camera_index, 3)
        ax = figure.add_subplot(grid[row, column * 2 : column * 2 + 2])
        camera = info["cams"][camera_name]
        image_path = resolve_image_path(camera["data_path"], infos_path)
        camera_image = Image.open(image_path).convert("RGB")
        image_size = camera_image.size
        ax.imshow(camera_image)
        visible_gt = visible_box_indices(
            gt_corners, camera, image_size, args.min_visible_ratio
        )
        visible_pred = visible_box_indices(
            pred_corners, camera, image_size, args.min_visible_ratio
        )
        for gt_index in visible_gt:
            draw_projected_box(
                ax,
                gt_corners[gt_index],
                camera,
                image_size,
                GT_COLOR,
                str(gt_names[gt_index]),
                0.8,
                0.8,
            )
        for pred_index in visible_pred:
            corners = pred_corners[pred_index]
            score = pred_scores[pred_index]
            label = pred_labels[pred_index]
            name = class_names[int(label)] if int(label) < len(class_names) else str(label)
            draw_projected_box(
                ax,
                corners,
                camera,
                image_size,
                PRED_COLOR,
                f"{name} {score:.2f}",
                1.0,
                0.9,
            )
        ax.set_title(
            f"{camera_name}  |  visible GT {len(visible_gt)}/{len(gt_boxes)} · "
            f"pred {len(visible_pred)}/{len(pred_boxes)}",
            color="white",
            fontsize=9,
            pad=3,
        )
        ax.set_xlim(0, image_size[0])
        ax.set_ylim(image_size[1], 0)
        ax.axis("off")

    bev = figure.add_subplot(grid[2, :3], facecolor="#171a20")
    for radius in (10, 30, 50):
        circle = plt.Circle((0, 0), radius, fill=False, color="#4c566a", linewidth=0.6)
        bev.add_patch(circle)
    for corners, name in zip(gt_corners, gt_names):
        draw_bev_box(bev, corners, GT_COLOR, str(name), 0.75, 1.0)
    for corners, score, label in zip(pred_corners, pred_scores, pred_labels):
        name = class_names[int(label)] if int(label) < len(class_names) else str(label)
        draw_bev_box(bev, corners, PRED_COLOR, f"{name} {score:.2f}", 0.9, 1.2)
    bev.arrow(0, 0, 0, 4, width=0.35, head_width=1.4, color="#55ccff")
    view_range = args.view_range
    bev.set_xlim(-view_range, view_range)
    bev.set_ylim(-view_range, view_range)
    bev.set_aspect("equal")
    bev.grid(color="#343b48", linewidth=0.35)
    bev.tick_params(colors="white", labelsize=7)
    bev.set_xlabel("lateral y [m]  (right ← | → left)", color="white")
    bev.set_ylabel("longitudinal x [m]  (front ↑)", color="white")
    bev.set_title("Bird's-eye view", color="white")

    view_3d = figure.add_subplot(grid[2, 3:], projection="3d", facecolor="#171a20")
    for corners in gt_corners:
        draw_3d_box(view_3d, corners, GT_COLOR, 0.65, 0.8)
    for corners in pred_corners:
        draw_3d_box(view_3d, corners, PRED_COLOR, 0.9, 1.0)
    for camera_name, camera in info["cams"].items():
        rotation = np.asarray(camera["sensor2lidar_rotation"])
        position = np.asarray(camera["sensor2lidar_translation"])
        direction = rotation[:, 2]
        view_3d.quiver(*position, *(direction * 3.0), color="#55ccff", linewidth=0.8)
    view_3d.scatter([0], [0], [0], marker="^", s=35, color="#55ccff", label="ego/cameras")
    view_3d.set_xlim(-view_range, view_range)
    view_3d.set_ylim(-view_range, view_range)
    view_3d.set_zlim(-2, 8)
    view_3d.set_box_aspect((2, 2, 0.5))
    view_3d.view_init(elev=28, azim=-120)
    view_3d.set_xlabel("x forward [m]", color="white", fontsize=8)
    view_3d.set_ylabel("y left [m]", color="white", fontsize=8)
    view_3d.set_zlabel("z [m]", color="white", fontsize=8)
    view_3d.tick_params(colors="white", labelsize=6)
    view_3d.set_title("3D ego-coordinate view", color="white")

    figure.suptitle(
        f"{info['token']}  |  GT: {len(gt_boxes)}  |  predictions: "
        f"{len(pred_boxes)}/{filter_stats['above_threshold']} @ score ≥ "
        f"{args.score_threshold:.2f}\n"
        f"GT = green   prediction = magenta   ego/cameras = cyan   ·   "
        f"camera visibility: centre + ≥ {args.min_visible_ratio:.0%} area",
        color="white",
        fontsize=13,
    )
    figure.tight_layout(rect=(0, 0, 1, 0.95))
    figure.savefig(output, dpi=args.dpi, facecolor=figure.get_facecolor())
    plt.close(figure)
    print(f"Wrote {output}")


def main():
    args = parse_args()
    if not 0.0 <= args.min_visible_ratio <= 1.0:
        raise ValueError("--min-visible-ratio must be between 0 and 1")
    info_payload = load_pickle(args.infos)
    prediction_payload = load_pickle(args.predictions)
    infos = info_payload["infos"]
    class_names = tuple(prediction_payload["class_names"])
    predictions = {item["index"]: item for item in prediction_payload["predictions"]}
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    rendered = []
    for index in args.indices:
        if index < 0 or index >= len(infos):
            raise IndexError(f"validation index {index} outside [0, {len(infos) - 1}]")
        if index not in predictions:
            raise KeyError(f"prediction for validation index {index} is missing")
        output = output_dir / f"frame_{index:03d}_{infos[index]['token']}.png"
        render_frame(
            infos[index], predictions[index], class_names, args.infos, output, args
        )
        rendered.append(output)

    cards = "\n".join(
        f'<a href="{html.escape(path.name)}"><img src="{html.escape(path.name)}" '
        f'alt="{html.escape(path.stem)}"><span>{html.escape(path.stem)}</span></a>'
        for path in rendered
    )
    index_html = f"""<!doctype html>
<html lang="en"><meta charset="utf-8">
<title>StreamPETR Town04 visualization</title>
<style>
body {{ background:#101216; color:#eee; font-family:sans-serif; margin:24px; }}
.legend {{ margin-bottom:18px; }}
.grid {{ display:grid; grid-template-columns:repeat(auto-fit,minmax(420px,1fr)); gap:18px; }}
a {{ color:#ddd; text-decoration:none; }}
img {{ display:block; width:100%; border:1px solid #3a3f4b; margin-bottom:6px; }}
</style>
<h1>StreamPETR Town04: labels vs predictions</h1>
<div class="legend">GT = green · prediction = magenta · ego/cameras = cyan ·
score ≥ {args.score_threshold:.2f} · camera boxes need
visible centre + {args.min_visible_ratio:.0%} unoccluded area</div>
<div class="grid">{cards}</div>
</html>
"""
    index_path = output_dir / "index.html"
    index_path.write_text(index_html, encoding="utf-8")
    print(f"Wrote {index_path}")


if __name__ == "__main__":
    main()
