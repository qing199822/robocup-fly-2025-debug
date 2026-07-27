#!/usr/bin/env python3

import math


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
