import hashlib
import json
from types import SimpleNamespace

import numpy as np
import pytest

from manual_calibration.dataset import Project
from manual_calibration.import_bag import decode_image, import_bag, pointcloud_xyz


@pytest.mark.parametrize("big", [False, True])
def test_organized_cloud_row_padding_and_endianness(big):
    order = ">" if big else "<"
    dtype = np.dtype(
        {
            "names": ["x", "y", "z"],
            "formats": [order + "f4"] * 3,
            "offsets": [0, 4, 8],
            "itemsize": 16,
        }
    )
    raw = bytearray(80)
    values = np.ndarray((2, 2), dtype=dtype, buffer=raw, strides=(40, 16))
    values["x"] = [[1, 2], [3, 4]]
    values["y"] = 5
    values["z"] = 6
    message = SimpleNamespace(
        fields=[
            SimpleNamespace(name=n, offset=i * 4, datatype=7, count=1) for i, n in enumerate("xyz")
        ],
        is_bigendian=big,
        height=2,
        width=2,
        point_step=16,
        row_step=40,
        data=np.frombuffer(raw, dtype=np.uint8),
    )
    assert np.array_equal(pointcloud_xyz(message), [[1, 5, 6], [2, 5, 6], [3, 5, 6], [4, 5, 6]])
    message.data = message.data[:-1]
    with pytest.raises(ValueError, match="Truncated"):
        pointcloud_xyz(message)


def test_image_row_padding():
    msg = SimpleNamespace(
        height=2,
        width=1,
        encoding="rgb8",
        step=4,
        data=np.array([255, 0, 0, 0, 0, 255, 0, 0], np.uint8),
    )
    assert np.array_equal(
        decode_image(msg, "sensor_msgs/msg/Image"), [[[0, 0, 255]], [[0, 255, 0]]]
    )


@pytest.mark.parametrize("storage", ["SQLITE3", "MCAP", "STANDALONE_MCAP"])
def test_import_real_synthetic_rosbag_without_ros(tmp_path, storage):
    pytest.importorskip("rosbags.highlevel")
    from rosbags.rosbag2 import StoragePlugin, Writer
    from rosbags.typesys import Stores, get_typestore

    store = get_typestore(Stores.ROS2_HUMBLE)
    types = store.types
    Header = types["std_msgs/msg/Header"]
    Time = types["builtin_interfaces/msg/Time"]
    Image = types["sensor_msgs/msg/Image"]
    Cloud = types["sensor_msgs/msg/PointCloud2"]
    Field = types["sensor_msgs/msg/PointField"]
    bag = tmp_path / "source"
    plugin = "MCAP" if storage == "STANDALONE_MCAP" else storage
    with Writer(bag, version=9, storage_plugin=getattr(StoragePlugin, plugin)) as writer:
        image_conn = writer.add_connection(
            "/camera/image/compressed", Image.__msgtype__, typestore=store
        )
        cloud_conn = writer.add_connection("/lidar/points", Cloud.__msgtype__, typestore=store)
        for i in range(4):
            header = Header(Time(1, i * 100000000), "camera_optical")
            image = Image(header, 8, 10, "bgr8", 0, 30, np.zeros(240, np.uint8))
            cloud = Cloud(
                Header(header.stamp, "lidar"),
                1,
                3,
                [Field(n, j * 4, 7, 1) for j, n in enumerate("xyz")],
                False,
                12,
                36,
                np.array([[5, 0, 0], [6, 1, 1], [7, -1, 1]], dtype="<f4").view(np.uint8).ravel(),
                True,
            )
            stamp = 10**9 + i * 100000000
            writer.write(image_conn, stamp, store.serialize_cdr(image, image.__msgtype__))
            writer.write(cloud_conn, stamp + 1, store.serialize_cdr(cloud, cloud.__msgtype__))
    before = {p.name: hashlib.sha256(p.read_bytes()).digest() for p in bag.iterdir() if p.is_file()}
    from pathlib import Path

    config = Path(__file__).parents[1] / "examples/bag-config.json"
    destination = tmp_path / "dataset"
    source = next(bag.glob("*.mcap")) if storage == "STANDALONE_MCAP" else bag
    filename = import_bag(source, config, destination, max_frames=3, tolerance_ms=10)
    project = Project(filename)
    assert len(project.frames["recording"]["camera"]) == 3
    assert project.load("recording", "camera", 0)[1].shape == (3, 3)
    assert project.settings("recording", "camera")["camera_frame"] == "camera_body"
    assert before == {
        p.name: hashlib.sha256(p.read_bytes()).digest() for p in bag.iterdir() if p.is_file()
    }
    with pytest.raises(FileExistsError):
        import_bag(bag, config, destination)
    parsed = json.loads(filename.read_text())
    assert parsed["import"]["max_sync_difference_ms"] == 10
