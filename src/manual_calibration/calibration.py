"""Coordinate conventions, validation, projection and portable calibration exports."""

import copy

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

AXES = ("x", "y", "z", "roll", "pitch", "yaw")
IDENTITY_MOUNT = dict.fromkeys(AXES, 0.0)
BODY_TO_OPTICAL = np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0]], dtype=float)


def mount_matrix(values):
    """Map a sensor's body coordinates into the reference frame (metres/degrees)."""
    vector = np.array([values[key] for key in AXES], dtype=float)
    if not np.isfinite(vector).all():
        raise ValueError("Pose values must be finite numbers.")
    if np.max(np.abs(vector[:3])) > 1000 or np.max(np.abs(vector[3:])) > 3600:
        raise ValueError("Pose exceeds the supported editing range.")
    result = np.eye(4)
    result[:3, 3] = vector[:3]
    result[:3, :3] = Rotation.from_euler("xyz", vector[3:], degrees=True).as_matrix()
    return result


def lens_parameters(intrinsics):
    model = intrinsics.get("camera_model", "plumb_bob")
    model = "fisheye" if model == "equidistant" else model
    if model not in ("plumb_bob", "fisheye"):
        raise ValueError("Use plumb_bob or fisheye/equidistant projection.")
    values = np.asarray(intrinsics["fx_fy_cx_cy"], dtype=float)
    distortion = np.asarray(intrinsics["distortion"], dtype=float)
    if values.shape != (4,) or not np.isfinite(values).all() or (values[:2] <= 0).any():
        raise ValueError("Enter finite fx, fy, cx, cy with positive focal lengths.")
    expected = 5 if model == "plumb_bob" else 4
    if distortion.shape != (expected,) or not np.isfinite(distortion).all():
        raise ValueError(f"{model} requires {expected} finite distortion coefficients.")
    fx, fy, cx, cy = values
    return model, np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1.0]]), distortion


def normalized_intrinsics(intrinsics):
    model, matrix, distortion = lens_parameters(intrinsics)
    return {
        "camera_model": model,
        "fx_fy_cx_cy": matrix[[0, 1, 0, 1], [0, 1, 2, 2]].tolist(),
        "distortion": distortion.tolist(),
    }


def optical_transform(settings):
    """Reference-frame points -> optical camera (x right, y down, z forward)."""
    rotation = np.asarray(settings.get("body_to_optical", BODY_TO_OPTICAL), dtype=float)
    if (
        rotation.shape != (3, 3)
        or not np.isfinite(rotation).all()
        or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-8)
        or not np.isclose(np.linalg.det(rotation), 1.0)
    ):
        raise ValueError("body_to_optical must be a proper 3-by-3 rotation matrix.")
    optical = np.eye(4)
    optical[:3, :3] = rotation
    return optical @ np.linalg.inv(mount_matrix(settings["camera_mount"]))


def editable_settings(base, submitted):
    """Whitelist edits; source paths, frames and sensor identities stay server-controlled."""
    for key in ("camera_id", "camera_frame", "reference_frame", "recording_id"):
        if submitted.get(key) != base[key]:
            raise ValueError(f"{key} does not match the selected camera/recording.")
    result = copy.deepcopy(base)
    for key in ("camera_mount", "intrinsics", "opacity", "point_spacing"):
        if key in submitted:
            result[key] = copy.deepcopy(submitted[key])
    mount_matrix(result["camera_mount"])
    result["intrinsics"] = normalized_intrinsics(result["intrinsics"])
    opacity = float(result.get("opacity", 0.78))
    spacing = int(result.get("point_spacing", 4))
    if not np.isfinite(opacity) or not 0 <= opacity <= 1 or not 1 <= spacing <= 12:
        raise ValueError("Opacity must be 0–1; point spacing must be 1–12.")
    result.update(opacity=opacity, point_spacing=spacing)
    return result


def export_settings(settings):
    result = copy.deepcopy(settings)
    transform = optical_transform(result)
    inverse = np.linalg.inv(transform)
    result["intrinsics"] = normalized_intrinsics(result["intrinsics"])
    result["status"] = "MANUAL_PREVIEW_NOT_APPLIED"
    result["conventions"] = {
        "mount": "Camera body -> reference frame; metres, fixed-axis XYZ RPY degrees; Rz Ry Rx.",
        "optical_axes": "x right, y down, z forward",
        "scope": "Camera extrinsics/intrinsics edited manually; LiDAR reference mounts fixed.",
        "inverse_vector": "Optical camera -> reference frame; x y z qx qy qz qw.",
    }
    result["T_camera_optical_from_reference"] = transform.tolist()
    result["T_reference_from_camera_optical"] = inverse.tolist()
    result["camera_optical_to_reference_xyzw"] = (
        inverse[:3, 3].tolist() + Rotation.from_matrix(inverse[:3, :3]).as_quat().tolist()
    )
    return result


def project_points(points, intrinsics):
    model, matrix, distortion = lens_parameters(intrinsics)
    points = np.asarray(points, dtype=np.float64).reshape(-1, 1, 3)
    project = cv2.fisheye.projectPoints if model == "fisheye" else cv2.projectPoints
    pixels, _ = project(points, np.zeros(3), np.zeros(3), matrix, distortion)
    return pixels.reshape(-1, 2)
