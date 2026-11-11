import copy

import numpy as np
import pytest

from manual_calibration.calibration import (
    BODY_TO_OPTICAL,
    IDENTITY_MOUNT,
    editable_settings,
    export_settings,
    lens_parameters,
    mount_matrix,
    optical_transform,
    project_points,
)


@pytest.fixture
def settings():
    return {
        "camera_id": "camera",
        "camera_frame": "camera_body",
        "reference_frame": "base",
        "recording_id": "run",
        "camera_mount": {**IDENTITY_MOUNT, "z": 1.5},
        "intrinsics": {
            "camera_model": "plumb_bob",
            "fx_fy_cx_cy": [500, 500, 400, 250],
            "distortion": [0, 0, 0, 0, 0],
        },
        "point_spacing": 4,
        "opacity": 0.78,
    }


def test_known_coordinate_conventions_and_export(settings):
    mount = mount_matrix({**IDENTITY_MOUNT, "x": 1, "y": 2, "z": 3, "yaw": 90})
    assert np.allclose(mount @ [1, 0, 0, 1], [1, 3, 3, 1])
    assert np.allclose(BODY_TO_OPTICAL @ [10, 0, 0], [0, 0, 10])
    optical = optical_transform(settings) @ [10, 0, 1.5, 1]
    assert np.allclose(project_points([optical[:3]], settings["intrinsics"]), [[400, 250]])
    result = export_settings(settings)
    assert np.allclose(
        np.array(result["T_reference_from_camera_optical"])
        @ np.array(result["T_camera_optical_from_reference"]),
        np.eye(4),
    )
    assert "status" not in settings


def test_fisheye_and_validation(settings):
    lens = {
        "camera_model": "equidistant",
        "fx_fy_cx_cy": [500, 500, 400, 250],
        "distortion": [0, 0, 0, 0],
    }
    assert np.allclose(project_points([[0, 0, 10]], lens), [[400, 250]])
    lens["distortion"] = [0] * 5
    with pytest.raises(ValueError, match="4 finite"):
        lens_parameters(lens)
    invalid = copy.deepcopy(settings)
    invalid["camera_mount"]["yaw"] = float("nan")
    with pytest.raises(ValueError):
        optical_transform(invalid)
    invalid = copy.deepcopy(settings)
    invalid["body_to_optical"] = np.zeros((3, 3)).tolist()
    with pytest.raises(ValueError):
        optical_transform(invalid)


def test_client_cannot_change_sources_or_frames(settings):
    supplied = {**settings, "source": "/private", "body_to_optical": np.eye(3).tolist()}
    assert "source" not in editable_settings(settings, supplied)
    assert "body_to_optical" not in editable_settings(settings, supplied)
    supplied["reference_frame"] = "other"
    with pytest.raises(ValueError, match="reference_frame"):
        editable_settings(settings, supplied)
