"""Shared projection path for browser previews, snapshots and video exports."""

import json
import shutil
import subprocess
from pathlib import Path

import cv2
import numpy as np

from .calibration import export_settings, optical_transform, project_points

MODES = ("original_overlay", "recorded_adjusted", "overlay_only")


def overlay(image, points, settings, width):
    transform = optical_transform(settings)
    camera = points @ transform[:3, :3].T + transform[:3, 3]
    camera = camera[np.isfinite(camera).all(axis=1) & (camera[:, 2] > 0.3)]
    ih, iw = image.shape[:2]
    height = max(2, round(width * ih / iw))
    result = cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)
    if not len(camera):
        return result, 0
    uv = project_points(camera, settings["intrinsics"])
    valid = (
        np.isfinite(uv).all(axis=1)
        & (uv[:, 0] >= 0)
        & (uv[:, 0] < iw)
        & (uv[:, 1] >= 0)
        & (uv[:, 1] < ih)
    )
    xy = np.rint(uv[valid] * (width / iw)).astype(np.int32)
    depth = camera[valid, 2]
    valid = (xy[:, 0] >= 0) & (xy[:, 0] < width) & (xy[:, 1] >= 0) & (xy[:, 1] < height)
    xy, depth = xy[valid], depth[valid]
    if not len(xy):
        return result, 0
    spacing = settings["point_spacing"]
    cells = (xy[:, 1] // spacing) * ((width + spacing - 1) // spacing) + xy[:, 0] // spacing
    order = np.lexsort((depth, cells))
    xy, depth, cells = xy[order], depth[order], cells[order]
    first = np.r_[True, cells[1:] != cells[:-1]]
    xy, depth = xy[first], depth[first]
    colors = cv2.applyColorMap(
        np.uint8(np.clip((depth - 1) / 49, 0, 1) * 255), cv2.COLORMAP_TURBO
    ).reshape(-1, 3)
    layer = result.copy()
    mask = np.zeros((height, width), dtype=bool)
    for dx, dy in ((0, 0), (-1, 0), (1, 0), (0, -1), (0, 1)):
        x, y = xy[:, 0] + dx, xy[:, 1] + dy
        keep = (x >= 0) & (x < width) & (y >= 0) & (y < height)
        layer[y[keep], x[keep]] = colors[keep]
        mask[y[keep], x[keep]] = True
    alpha = settings["opacity"]
    result[mask] = cv2.addWeighted(layer, alpha, result, 1 - alpha, 0)[mask]
    return result, len(xy)


def panel(image, label):
    result = cv2.copyMakeBorder(image, 34, 0, 0, 0, cv2.BORDER_CONSTANT, value=(20, 22, 25))
    cv2.putText(result, label, (12, 23), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (240, 240, 240), 1)
    return result


def render(project, settings, frame, mode="original_overlay", width=960):
    if mode not in MODES or width not in (720, 960, 1440):
        raise ValueError("Unsupported display mode or preview width.")
    rid, cid = settings["recording_id"], settings["camera_id"]
    image, cloud, stamp, offsets = project.load(rid, cid, frame)
    adjusted, count = overlay(image, cloud, settings, width)
    baseline = project.settings(rid, cid)
    label = "Manual preview"
    if mode == "original_overlay":
        result = np.hstack(
            [
                panel(cv2.resize(image, (width, adjusted.shape[0])), "Original image"),
                panel(adjusted, label),
            ]
        )
    elif mode == "recorded_adjusted":
        original, _ = overlay(image, cloud, baseline, width)
        result = np.hstack([panel(original, "Baseline calibration"), panel(adjusted, label)])
    else:
        result = panel(adjusted, label)
    offset = (stamp - project.frames[rid][cid][0]["image_stamp_ns"]) / 1e9
    return result, {
        "camera_id": cid,
        "recording_id": rid,
        "image_stamp_ns": str(stamp),
        "offset_seconds": offset,
        "scan_offsets_ms": offsets,
        "points": count,
        "input_points": len(cloud),
        "motion_compensated": False,
    }


def write_video(project, settings, target):
    """Resample by acquisition time so variable-rate input does not change playback speed."""
    if shutil.which("ffmpeg") is None:
        raise ValueError("Install FFmpeg to enable MP4 export.")
    target = Path(target)
    if target.exists():
        raise FileExistsError("Video output already exists.")
    rid, cid = settings["recording_id"], settings["camera_id"]
    rows = project.frames[rid][cid]
    stamps = np.array([row["image_stamp_ns"] for row in rows], dtype=np.int64)
    relative = (stamps - stamps[0]) / 1e9
    fps = float(project.recordings[rid].get("fps", 10))
    if not np.isfinite(fps) or not 0 < fps <= 60:
        raise ValueError("Video frame rate must be between 0 and 60.")
    count = max(1, round(relative[-1] * fps) + 1)
    if count > 36000:
        raise ValueError("Limit exports to 36,000 output frames; split a longer recording.")
    width = 960
    first, _ = render(project, settings, 0, "overlay_only", width)
    height = first.shape[0] + first.shape[0] % 2
    target.parent.mkdir(parents=True, exist_ok=True)
    command = [
        "ffmpeg",
        "-hide_banner",
        "-loglevel",
        "error",
        "-n",
        "-f",
        "rawvideo",
        "-pixel_format",
        "bgr24",
        "-video_size",
        f"{width}x{height}",
        "-framerate",
        str(fps),
        "-i",
        "pipe:0",
        "-an",
        "-c:v",
        "libx264",
        "-preset",
        "fast",
        "-crf",
        "20",
        "-threads",
        "2",
        "-pix_fmt",
        "yuv420p",
        "-movflags",
        "+faststart",
        str(target),
    ]
    sidecar = target.with_suffix(".json")
    selected = []
    with target.with_suffix(".log").open("wb") as log:
        process = subprocess.Popen(command, stdin=subprocess.PIPE, stderr=log)
        try:
            for tick in range(count):
                time = tick / fps
                right = min(int(np.searchsorted(relative, time)), len(rows) - 1)
                left = max(0, right - 1)
                index = min((left, right), key=lambda i: abs(relative[i] - time))
                image, _ = render(project, settings, index, "overlay_only", width)
                if image.shape[0] < height:
                    image = cv2.copyMakeBorder(image, 0, 1, 0, 0, cv2.BORDER_CONSTANT)
                process.stdin.write(image.tobytes())
                selected.append(index)
            process.stdin.close()
            if process.wait(timeout=120):
                raise RuntimeError("FFmpeg failed; inspect the export log.")
        except BaseException:
            if process.poll() is None:
                process.kill()
            process.wait()
            if process.stdin and not process.stdin.closed:
                process.stdin.close()
            target.unlink(missing_ok=True)
            raise
    sidecar.write_text(
        json.dumps(
            {
                "settings": export_settings(settings),
                "fps": fps,
                "source_frame_indices": selected,
                "motion_compensated": False,
            },
            indent=2,
        )
        + "\n"
    )
