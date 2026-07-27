#!/usr/bin/env python3

from collections import namedtuple
import math
from numbers import Integral

import numpy as np


DepthSample = namedtuple(
    "DepthSample",
    ("depth_meters", "stamp", "stamp_seconds", "frame_id", "encoding"),
)


def _finite_float(value, label):
    try:
        converted = float(value)
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError(f"{label} must be a finite number") from error
    if not math.isfinite(converted):
        raise ValueError(f"{label} must be a finite number")
    return converted


def deproject_pixel(u, v, depth_meters, camera_matrix):
    try:
        matrix_length = len(camera_matrix)
    except TypeError as error:
        raise ValueError("CameraInfo.K must contain 9 values") from error
    if matrix_length != 9:
        raise ValueError("CameraInfo.K must contain 9 values")

    matrix = [
        _finite_float(value, f"CameraInfo.K[{index}]")
        for index, value in enumerate(camera_matrix)
    ]
    fx = matrix[0]
    fy = matrix[4]
    if fx <= 0.0 or fy <= 0.0:
        raise ValueError("CameraInfo focal lengths must be finite and positive")

    pixel_u = _finite_float(u, "u")
    pixel_v = _finite_float(v, "v")
    depth = _finite_float(depth_meters, "depth_meters")
    if depth <= 0.0:
        raise ValueError("depth_meters must be positive")
    cx = matrix[2]
    cy = matrix[5]
    return (
        (pixel_u - cx) * depth / fx,
        (pixel_v - cy) * depth / fy,
        depth,
    )


def timestamps_within(first_seconds, second_seconds, maximum_delta_seconds):
    first = _finite_float(first_seconds, "first_seconds")
    second = _finite_float(second_seconds, "second_seconds")
    maximum = _finite_float(maximum_delta_seconds, "maximum_delta_seconds")
    if maximum < 0.0:
        raise ValueError("maximum timestamp delta must be non-negative")
    return abs(first - second) <= maximum


def select_closest_depth_sample(samples, target_seconds, maximum_delta_seconds):
    target = _finite_float(target_seconds, "target_seconds")
    maximum = _finite_float(maximum_delta_seconds, "maximum_delta_seconds")
    if target <= 0.0:
        raise ValueError("target_seconds must be positive")
    if maximum < 0.0:
        raise ValueError("maximum timestamp delta must be non-negative")

    candidates = []
    for sample in samples:
        try:
            sample_seconds = _finite_float(sample.stamp_seconds, "sample.stamp_seconds")
        except AttributeError as error:
            raise ValueError("depth samples must provide stamp_seconds") from error
        if sample_seconds <= 0.0:
            raise ValueError("sample.stamp_seconds must be positive")
        delta = abs(sample_seconds - target)
        if delta <= maximum:
            candidates.append((delta, sample_seconds, sample))

    if not candidates:
        return None
    return min(candidates, key=lambda candidate: (candidate[0], candidate[1]))[2]


def depth_image_to_meters(image, encoding):
    if encoding not in ("32FC1", "16UC1"):
        raise ValueError(f"unsupported depth encoding: {encoding}")
    try:
        source = np.asarray(image)
    except Exception as error:
        raise ValueError("depth image must be a numeric 2-D array") from error
    if source.ndim != 2 or source.shape[0] == 0 or source.shape[1] == 0:
        raise ValueError("depth image must be a non-empty 2-D array")
    if not np.issubdtype(source.dtype, np.number):
        raise ValueError("depth image must contain numeric values")

    try:
        converted = np.array(source, dtype=np.float32, copy=True)
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError("depth image must contain numeric values") from error
    if encoding == "16UC1":
        if not np.all(np.isfinite(converted)):
            raise ValueError("16UC1 depth image must contain finite values")
        converted *= np.float32(0.001)
    return converted


def roi_mean_depth(depth_meters, u, v, half_size):
    if isinstance(u, bool) or not isinstance(u, Integral):
        raise ValueError("u must be an integer")
    if isinstance(v, bool) or not isinstance(v, Integral):
        raise ValueError("v must be an integer")
    if isinstance(half_size, bool) or not isinstance(half_size, Integral):
        raise ValueError("half_size must be a non-negative integer")
    if half_size < 0:
        raise ValueError("half_size must be a non-negative integer")

    try:
        depth = np.asarray(depth_meters, dtype=float)
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError("depth_meters must be a numeric 2-D array") from error
    if depth.ndim != 2 or depth.shape[0] == 0 or depth.shape[1] == 0:
        raise ValueError("depth_meters must be a non-empty 2-D array")

    height, width = depth.shape
    if u < 0 or u >= width or v < 0 or v >= height:
        raise ValueError("ROI center must be inside the depth image")

    u_min = max(0, u - half_size)
    u_max = min(width - 1, u + half_size)
    v_min = max(0, v - half_size)
    v_max = min(height - 1, v + half_size)
    roi = depth[v_min : v_max + 1, u_min : u_max + 1]
    valid_depths = roi[(roi > 0.0) & np.isfinite(roi)]
    if valid_depths.size == 0:
        return None
    return float(np.mean(valid_depths))
