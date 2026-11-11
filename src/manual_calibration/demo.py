"""A deterministic synthetic scene; contains no recorded images or sensor parameters."""

import json
from pathlib import Path

import cv2
import numpy as np

from .calibration import IDENTITY_MOUNT, optical_transform, project_points


def create_demo(destination):
    root = Path(destination).resolve()
    root.mkdir(parents=True, exist_ok=False)
    (root / "images").mkdir()
    (root / "clouds").mkdir()
    camera = {
        "id": "camera",
        "label": "Synthetic camera",
        "frame": "camera_body",
        "camera_mount": {**IDENTITY_MOUNT, "z": 1.5},
        "intrinsics": {
            "camera_model": "plumb_bob",
            "fx_fy_cx_cy": [500, 500, 400, 250],
            "distortion": [0, 0, 0, 0, 0],
        },
    }
    lidars = [{"id": "lidar", "label": "Synthetic LiDAR", "mount": dict(IDENTITY_MOUNT)}]
    # Grid, two signboards and a ground plane, all in the synthetic reference frame.
    rng = np.random.default_rng(7)
    ground = np.column_stack([rng.uniform(3, 25, 6000), rng.uniform(-8, 8, 6000), np.zeros(6000)])
    wall = np.array([[12, y, z] for y in np.linspace(-4, 4, 120) for z in np.linspace(0, 4, 60)])
    points = np.vstack([ground, wall])
    frames = []
    for i in range(12):
        moving = points.copy()
        moving[:, 0] -= i * 0.08
        optical = optical_transform(camera)
        p = moving @ optical[:3, :3].T + optical[:3, 3]
        pixels = project_points(p, camera["intrinsics"])
        image = np.full((500, 800, 3), (36, 33, 30), np.uint8)
        for index in np.flatnonzero((p[:, 2] > 0) & np.isfinite(pixels).all(axis=1)):
            u, v = np.rint(pixels[index]).astype(int)
            if 0 <= u < 800 and 0 <= v < 500:
                cv2.circle(image, (u, v), 1, (120, 135, 140), -1)
        cv2.putText(
            image,
            "SYNTHETIC DEMO - no recorded data",
            (24, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (230, 230, 230),
            1,
        )
        cv2.imwrite(str(root / f"images/{i:04}.png"), image)
        np.save(root / f"clouds/{i:04}.npy", moving.astype(np.float32))
        frames.append(
            {
                "image": f"images/{i:04}.png",
                "image_stamp_ns": i * 100000000,
                "clouds": {"lidar": {"path": f"clouds/{i:04}.npy", "stamp_ns": i * 100000000}},
            }
        )
    project = {
        "version": 1,
        "reference_frame": "base_link",
        "cameras": [camera],
        "lidars": lidars,
        "recordings": [
            {"id": "demo", "label": "Synthetic scene", "fps": 10, "frames": {"camera": frames}}
        ],
    }
    (root / "project.json").write_text(json.dumps(project, indent=2) + "\n")
    return root / "project.json"
