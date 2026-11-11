"""Read-only portable image/point-cloud projects, independent of ROS installation."""

import copy
import json
import re
from collections import OrderedDict
from pathlib import Path
from threading import RLock

import cv2
import numpy as np

from .calibration import BODY_TO_OPTICAL, lens_parameters, mount_matrix


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def asset_path(root, relative):
    path = (root / relative).resolve()
    if not path.is_relative_to(root) or not path.is_file():
        raise ValueError(f"Dataset asset must be a file within the project: {relative}")
    return path


def load_cloud(path):
    if path.suffix.lower() == ".npy":
        xyz = np.load(path, allow_pickle=False)
    else:
        raise ValueError("Point clouds must be NumPy .npy arrays with shape N-by-3 or N-by-4.")
    if xyz.ndim != 2 or xyz.shape[1] not in (3, 4) or not np.issubdtype(xyz.dtype, np.number):
        raise ValueError("Point clouds must contain numeric XYZ (and optional intensity).")
    xyz = np.asarray(xyz[:, :3], dtype=np.float64)
    return xyz[np.isfinite(xyz).all(axis=1)]


class Project:
    def __init__(self, filename):
        self.filename = Path(filename).resolve()
        self.root = self.filename.parent
        self.config = read_json(self.filename)
        if self.config.get("version") != 1:
            raise ValueError("Project version must be 1.")
        self.reference = self.config["reference_frame"]
        self.cameras = self._catalog("cameras")
        self.lidars = self._catalog("lidars")
        self.recordings = self._catalog("recordings")
        self.cache = OrderedDict()
        self.lock = RLock()
        for camera in self.cameras.values():
            mount_matrix(camera["camera_mount"])
            lens_parameters(camera["intrinsics"])
        for lidar in self.lidars.values():
            mount_matrix(lidar["mount"])
        self.frames = {}
        for rid, recording in self.recordings.items():
            self.frames[rid] = {}
            for cid in self.cameras:
                rows = recording["frames"].get(cid, [])
                if not rows:
                    raise ValueError(f"Recording {rid} has no frames for camera {cid}.")
                stamps = []
                for row in rows:
                    stamp = row["image_stamp_ns"]
                    if isinstance(stamp, bool) or not isinstance(stamp, int):
                        raise ValueError("image_stamp_ns must be an integer nanosecond timestamp.")
                    stamps.append(stamp)
                    asset_path(self.root, row["image"])
                    if set(row["clouds"]) != set(self.lidars):
                        raise ValueError("Every frame must contain all configured LiDARs.")
                    for cloud in row["clouds"].values():
                        asset_path(self.root, cloud["path"])
                        cloud_stamp = cloud["stamp_ns"]
                        if isinstance(cloud_stamp, bool) or not isinstance(cloud_stamp, int):
                            raise ValueError("stamp_ns must be an integer nanosecond timestamp.")
                if any(a >= b for a, b in zip(stamps, stamps[1:])):
                    raise ValueError("Camera timestamps must increase strictly.")
                self.frames[rid][cid] = rows

    def _catalog(self, key):
        items = self.config[key]
        result = {}
        for item in items:
            identifier = item["id"]
            if not isinstance(identifier, str) or not re.fullmatch(
                r"[A-Za-z0-9_-]{1,64}", identifier
            ):
                raise ValueError(f"Invalid {key} ID; use letters, digits, underscores or hyphens.")
            if identifier in result:
                raise ValueError(f"Duplicate {key} ID: {identifier}")
            result[identifier] = item
        if not result:
            raise ValueError(f"At least one {key} entry is required.")
        return result

    def settings(self, rid, cid):
        camera = self.cameras[cid]
        self.frames[rid][cid]
        return {
            "camera_id": cid,
            "camera_frame": camera["frame"],
            "camera_label": camera.get("label", cid),
            "recording_id": rid,
            "reference_frame": self.reference,
            "camera_mount": copy.deepcopy(camera["camera_mount"]),
            "intrinsics": copy.deepcopy(camera["intrinsics"]),
            "body_to_optical": camera.get("body_to_optical", BODY_TO_OPTICAL.tolist()),
            "image_pipeline": camera.get("image_pipeline", {}),
            "lidar_mounts": {
                key: copy.deepcopy(value["mount"]) for key, value in self.lidars.items()
            },
            "opacity": 0.78,
            "point_spacing": 4,
        }

    def configuration(self, rid, cid):
        rows = self.frames[rid][cid]
        return {
            "settings": self.settings(rid, cid),
            "frame_count": len(rows),
            "duration_s": (rows[-1]["image_stamp_ns"] - rows[0]["image_stamp_ns"]) / 1e9,
            "cameras": [
                {"id": key, "label": row.get("label", key)} for key, row in self.cameras.items()
            ],
            "recordings": [
                {"id": key, "label": row.get("label", key)} for key, row in self.recordings.items()
            ],
            "lidar_count": len(self.lidars),
        }

    def load(self, rid, cid, frame):
        if (
            isinstance(frame, bool)
            or not isinstance(frame, int)
            or not 0 <= frame < len(self.frames[rid][cid])
        ):
            raise ValueError("Frame index is outside the selected recording.")
        key = (rid, cid, frame)
        with self.lock:
            if key in self.cache:
                self.cache.move_to_end(key)
                return self.cache[key]
            row = self.frames[rid][cid][frame]
            image = cv2.imread(str(asset_path(self.root, row["image"])))
            if image is None:
                raise ValueError("Could not decode camera image.")
            chunks = []
            offsets = {}
            for identifier, source in row["clouds"].items():
                points = load_cloud(asset_path(self.root, source["path"]))
                transform = mount_matrix(self.lidars[identifier]["mount"])
                chunks.append(points @ transform[:3, :3].T + transform[:3, 3])
                offsets[identifier] = (int(source["stamp_ns"]) - row["image_stamp_ns"]) / 1e6
            value = (image, np.concatenate(chunks), row["image_stamp_ns"], offsets)
            self.cache[key] = value
            while len(self.cache) > 4:
                self.cache.popitem(last=False)
            return value
