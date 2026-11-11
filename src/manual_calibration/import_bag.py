"""Sample standard ROS image/PointCloud2 messages into a portable project."""

import copy
import json
import tempfile
from bisect import bisect_left
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np

from .dataset import Project, read_json


def pointcloud_xyz(message):
    """Respect PointCloud2 field offsets, byte order and organized row padding."""
    fields = {field.name: field for field in message.fields}
    names, formats, offsets = [], [], []
    order = ">" if message.is_bigendian else "<"
    for name in ("x", "y", "z"):
        field = fields[name]
        if field.count != 1 or field.datatype not in (7, 8):
            raise ValueError("PointCloud2 XYZ fields must be scalar FLOAT32 or FLOAT64.")
        names.append(name)
        formats.append(order + ("f4" if field.datatype == 7 else "f8"))
        offsets.append(field.offset)
    dtype = np.dtype(
        {"names": names, "formats": formats, "offsets": offsets, "itemsize": message.point_step}
    )
    raw = np.asarray(message.data, dtype=np.uint8).tobytes()
    if message.row_step < message.width * message.point_step:
        raise ValueError("PointCloud2 row_step is smaller than its point data.")
    if len(raw) < message.height * message.row_step:
        raise ValueError("Truncated PointCloud2 payload.")
    points = np.ndarray(
        (message.height, message.width),
        dtype=dtype,
        buffer=raw,
        strides=(message.row_step, message.point_step),
    )
    xyz = np.column_stack([points[name].ravel() for name in names]).astype(np.float32)
    return xyz[np.isfinite(xyz).all(axis=1)]


def decode_image(message, kind):
    raw = np.asarray(message.data, dtype=np.uint8)
    if kind.endswith("/CompressedImage"):
        image = cv2.imdecode(raw, cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError("Invalid compressed image.")
        return image
    channels = {"mono8": 1, "bgr8": 3, "rgb8": 3, "bgra8": 4, "rgba8": 4}
    if message.encoding not in channels:
        raise ValueError(f"Unsupported Image encoding: {message.encoding}; use 8-bit mono/RGB/BGR.")
    count = channels[message.encoding]
    if message.step < message.width * count or len(raw) < message.height * message.step:
        raise ValueError("Invalid Image row stride or payload size.")
    image = np.ndarray(
        (message.height, message.width, count),
        dtype=np.uint8,
        buffer=raw,
        strides=(message.step, count, 1),
    ).copy()
    conversion = {
        "mono8": cv2.COLOR_GRAY2BGR,
        "rgb8": cv2.COLOR_RGB2BGR,
        "rgba8": cv2.COLOR_RGBA2BGR,
        "bgra8": cv2.COLOR_BGRA2BGR,
    }
    if message.encoding == "mono8":
        image = image[:, :, 0]
    return (
        cv2.cvtColor(image, conversion[message.encoding]) if message.encoding != "bgr8" else image
    )


def import_bag(source, config_file, destination, max_frames=120, tolerance_ms=50):
    try:
        from rosbags.highlevel import AnyReader
        from rosbags.typesys import Stores, get_typestore
    except ImportError as error:
        raise ImportError('Install the optional reader with: pip install ".[bags]"') from error
    if max_frames < 1 or not np.isfinite(tolerance_ms) or tolerance_ms < 0:
        raise ValueError(
            "Use positive max-frames and a finite, nonnegative synchronization tolerance."
        )
    output = Path(destination).resolve()
    if output.exists():
        raise FileExistsError("Use a new output directory; existing datasets are never replaced.")
    config = read_json(config_file)
    cameras, lidars = config["cameras"], config["lidars"]
    topics = {}
    for kind, sensors in (("image", cameras), ("cloud", lidars)):
        for sensor in sensors:
            topic = sensor["topic"]
            if topic in topics:
                raise ValueError("Configure each source topic once.")
            topics[topic] = (kind, sensor)
    index = defaultdict(list)
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    # First pass retains only indices/timestamps. Second pass reads only chosen
    # payloads into the dataset, avoiding a whole-bag image/cloud memory cache.
    with AnyReader([Path(source)], default_typestore=typestore) as reader:
        connections = [c for c in reader.connections if c.topic in topics]
        if set(topics) != {c.topic for c in connections}:
            raise ValueError("Some configured camera or LiDAR topics are missing from the bag.")
        for ordinal, (connection, _, raw) in enumerate(reader.messages(connections=connections)):
            kind, sensor = topics[connection.topic]
            expected = ("/Image", "/CompressedImage") if kind == "image" else ("/PointCloud2",)
            if not connection.msgtype.endswith(expected):
                raise ValueError(f"Unsupported type for {connection.topic}: {connection.msgtype}")
            message = reader.deserialize(raw, connection.msgtype)
            expected_frame = sensor.get(
                "message_frame", sensor.get("frame") if kind == "cloud" else None
            )
            if expected_frame and message.header.frame_id != expected_frame:
                raise ValueError(f"Frame mismatch for {connection.topic}.")
            stamp = int(message.header.stamp.sec) * 10**9 + int(message.header.stamp.nanosec)
            offset = float(sensor.get("time_offset_ms", 0))
            if not np.isfinite(offset):
                raise ValueError("Sensor time offsets must be finite.")
            index[connection.topic].append((stamp + round(offset * 1e6), ordinal, stamp))
        for topic in topics:
            index[topic].sort()
            if not index[topic]:
                raise ValueError(f"No messages on {topic}.")
        selected = set()
        frame_lists = {}
        tolerance = tolerance_ms * 1e6
        for camera in cameras:
            candidates = []
            seen = set()
            for image in index[camera["topic"]]:
                if image[0] in seen:
                    continue
                seen.add(image[0])
                matches = {}
                for lidar in lidars:
                    options = index[lidar["topic"]]
                    right = bisect_left(options, (image[0], -1, -1))
                    nearby = options[max(0, right - 1) : min(len(options), right + 1)]
                    match = min(nearby, key=lambda row: abs(row[0] - image[0]))
                    if abs(match[0] - image[0]) > tolerance:
                        break
                    matches[lidar["id"]] = match
                if len(matches) == len(lidars):
                    candidates.append((image, matches))
            if not candidates:
                raise ValueError(f"No synchronized frames for camera {camera['id']}.")
            picks = np.unique(
                np.rint(
                    np.linspace(0, len(candidates) - 1, min(max_frames, len(candidates)))
                ).astype(int)
            )
            frames = []
            for i in picks:
                image, matches = candidates[i]
                selected.add(image[1])
                selected.update(row[1] for row in matches.values())
                frames.append(
                    {
                        "image": f"images/{image[1]}.png",
                        "image_stamp_ns": image[0],
                        "clouds": {
                            key: {
                                "path": f"clouds/{row[1]}.npy",
                                "stamp_ns": row[0],
                                "source_header_ns": row[2],
                            }
                            for key, row in matches.items()
                        },
                    }
                )
            frame_lists[camera["id"]] = frames
        output.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(prefix=".manual-import-", dir=output.parent) as temporary:
            root = Path(temporary) / "dataset"
            (root / "images").mkdir(parents=True)
            (root / "clouds").mkdir()
            for ordinal, (connection, _, raw) in enumerate(
                reader.messages(connections=connections)
            ):
                if ordinal not in selected:
                    continue
                message = reader.deserialize(raw, connection.msgtype)
                if topics[connection.topic][0] == "image":
                    if not cv2.imwrite(
                        str(root / f"images/{ordinal}.png"),
                        decode_image(message, connection.msgtype),
                    ):
                        raise ValueError("Could not save imported image.")
                else:
                    np.save(root / f"clouds/{ordinal}.npy", pointcloud_xyz(message))
            project = copy.deepcopy(config)
            project.update(
                version=1,
                recordings=[
                    {
                        "id": "recording",
                        "label": "Imported recording",
                        "fps": 10,
                        "frames": frame_lists,
                    }
                ],
            )
            project["import"] = {
                "timing": "Nearest header stamp plus configured time_offset_ms; no automatic clock fitting or deskew.",
                "max_sync_difference_ms": tolerance_ms,
                "source": str(Path(source).resolve()),
            }
            (root / "project.json").write_text(json.dumps(project, indent=2) + "\n")
            Project(root / "project.json")
            if output.exists():
                raise FileExistsError(
                    "Output directory was created during import; refusing to replace it."
                )
            root.rename(output)
    return output / "project.json"
