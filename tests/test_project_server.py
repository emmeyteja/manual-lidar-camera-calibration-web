import copy
import hashlib
import json
import re
import shutil
import subprocess
import threading
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import cv2
import numpy as np
import pytest

from manual_calibration.dataset import Project
from manual_calibration.demo import create_demo
from manual_calibration.render import render, write_video
from manual_calibration.server import create_server


@pytest.fixture(scope="module")
def demo(tmp_path_factory):
    return create_demo(tmp_path_factory.mktemp("synthetic") / "dataset")


def test_preview_and_readonly_sources(demo):
    project = Project(demo)
    source = project.root / "clouds/0000.npy"
    before = hashlib.sha256(source.read_bytes()).digest()
    settings = project.settings("demo", "camera")
    image, info = render(project, settings, 0)
    assert image.shape[1] == 1920 and info["points"] > 100
    settings["camera_mount"]["yaw"] += 3
    changed, _ = render(project, settings, 0)
    assert not np.array_equal(image, changed)
    assert hashlib.sha256(source.read_bytes()).digest() == before
    with pytest.raises(ValueError):
        render(project, settings, -1)
    with pytest.raises(ValueError):
        render(project, settings, 0.0)


def test_dataset_rejects_traversal_duplicates_and_missing_streams(demo, tmp_path):
    outside = tmp_path / "outside.png"
    outside.write_bytes(b"not an image")
    config = json.loads(demo.read_text())
    config["recordings"][0]["frames"]["camera"][0]["image"] = str(outside)
    alternate = demo.parent / "invalid.json"
    alternate.write_text(json.dumps(config))
    with pytest.raises(ValueError, match="within the project"):
        Project(alternate)
    config = json.loads(demo.read_text())
    config["cameras"].append(copy.deepcopy(config["cameras"][0]))
    alternate.write_text(json.dumps(config))
    with pytest.raises(ValueError, match="Duplicate"):
        Project(alternate)
    config = json.loads(demo.read_text())
    config["recordings"][0]["frames"]["camera"][0]["clouds"] = {}
    alternate.write_text(json.dumps(config))
    with pytest.raises(ValueError, match="all configured"):
        Project(alternate)


def test_multiple_cameras_and_recordings(demo):
    config = json.loads(demo.read_text())
    camera = copy.deepcopy(config["cameras"][0])
    camera.update(id="second", label="Second camera", frame="second_body")
    config["cameras"].append(camera)
    config["recordings"][0]["frames"]["second"] = copy.deepcopy(
        config["recordings"][0]["frames"]["camera"]
    )
    other = copy.deepcopy(config["recordings"][0])
    other.update(id="other", label="Other recording")
    config["recordings"].append(other)
    path = demo.parent / "multiple.json"
    path.write_text(json.dumps(config))
    project = Project(path)
    assert project.settings("other", "second")["camera_frame"] == "second_body"
    assert project.configuration("other", "second")["frame_count"] == 12


def test_http_preview_export_and_request_boundaries(demo):
    project = Project(demo)
    server = create_server(project, port=0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    url = f"http://127.0.0.1:{server.server_address[1]}"
    try:
        with urlopen(url) as response:
            html = response.read().decode()
            assert "frame-ancestors 'none'" in response.headers["Content-Security-Policy"]
        token = re.search('name="calibration-token" content="([^"]+)"', html)[1]
        with urlopen(url + "/config") as response:
            config = json.load(response)
        settings = config["settings"]
        assert "/home/" not in json.dumps(config)
        body = json.dumps({"settings": settings, "frame": 0}).encode()
        with pytest.raises(HTTPError) as error:
            urlopen(Request(url + "/preview", body, {"Content-Type": "application/json"}))
        assert error.value.code == 403
        headers = {"Content-Type": "application/json", "X-Calibration-Token": token}
        with urlopen(Request(url + "/preview", body, headers)) as response:
            image = cv2.imdecode(np.frombuffer(response.read(), np.uint8), cv2.IMREAD_COLOR)
            assert image.shape[1] == 1920
            assert json.loads(response.headers["X-Overlay-Info"])["points"] > 100
        with urlopen(Request(url + "/export", body, headers)) as response:
            assert json.load(response)["status"] == "MANUAL_PREVIEW_NOT_APPLIED"
        with pytest.raises(HTTPError) as error:
            urlopen(Request(url + "/config", headers={"Host": "evil.example"}))
        assert error.value.code == 403
        with pytest.raises(HTTPError) as error:
            urlopen(url + "/../../pyproject.toml")
        assert error.value.code == 404
    finally:
        server.shutdown()
        server.server_close()
        thread.join()


@pytest.mark.skipif(not shutil.which("ffmpeg"), reason="FFmpeg is optional")
def test_video_is_readable_and_preserves_elapsed_time(demo, tmp_path):
    project = Project(demo)
    target = tmp_path / "overlay.mp4"
    write_video(project, project.settings("demo", "camera"), target)
    capture = cv2.VideoCapture(str(target))
    try:
        assert capture.isOpened()
        assert capture.get(cv2.CAP_PROP_FRAME_COUNT) == 12
        assert capture.get(cv2.CAP_PROP_FPS) == pytest.approx(10)
        ok, image = capture.read()
        assert ok and image.shape[1] == 960
    finally:
        capture.release()
    sidecar = json.loads(target.with_suffix(".json").read_text())
    assert sidecar["source_frame_indices"] == list(range(12))
    with pytest.raises(FileExistsError):
        write_video(project, project.settings("demo", "camera"), target)


def test_installed_cli(demo, tmp_path):
    import sys

    completed = subprocess.run(
        [
            sys.executable,
            "-c",
            "from manual_calibration.cli import main; main()",
            "snapshot",
            "--project",
            str(demo),
            "--output",
            str(tmp_path / "view.jpg"),
        ],
        capture_output=True,
        text=True,
    )
    assert completed.returncode == 0, completed.stderr
    assert (tmp_path / "view.jpg").is_file()
