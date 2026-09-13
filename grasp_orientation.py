#!/usr/bin/env python3
"""Planar object-axis estimation helpers independent of ROS."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math

import numpy as np


def normalize_axial(angle: float) -> float:
    """Normalize an undirected axis angle to [-pi/2, pi/2)."""

    return (float(angle) + math.pi / 2.0) % math.pi - math.pi / 2.0


def axial_difference(first: float, second: float) -> float:
    """Smallest absolute difference between two pi-periodic axes."""

    return abs(normalize_axial(float(first) - float(second)))


@dataclass(frozen=True)
class PlanarAxisEstimate:
    yaw: float
    length: float
    width: float
    aspect_ratio: float
    point_count: int


def estimate_planar_axis(
    xy_points: np.ndarray,
    min_points: int,
    min_aspect_ratio: float,
) -> PlanarAxisEstimate | None:
    """Fit the long axis of a rectangular top surface using robust 2-D PCA."""

    points = np.asarray(xy_points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 2:
        raise ValueError("xy_points must have shape (N, 2)")
    points = points[np.all(np.isfinite(points), axis=1)]
    if points.shape[0] < int(min_points):
        return None

    center = np.median(points, axis=0)
    distances = np.linalg.norm(points - center, axis=1)
    cutoff = float(np.percentile(distances, 99.0))
    if not math.isfinite(cutoff) or cutoff <= 0.0:
        return None
    points = points[distances <= cutoff]
    if points.shape[0] < int(min_points):
        return None

    covariance = np.cov(points, rowvar=False)
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    if not np.all(np.isfinite(eigenvalues)) or eigenvalues[1] <= 1e-10:
        return None

    long_axis = eigenvectors[:, 1]
    short_axis = eigenvectors[:, 0]
    centered = points - np.mean(points, axis=0)
    long_projection = centered @ long_axis
    short_projection = centered @ short_axis
    length = float(np.percentile(long_projection, 98.0) - np.percentile(long_projection, 2.0))
    width = float(np.percentile(short_projection, 98.0) - np.percentile(short_projection, 2.0))
    if not math.isfinite(length) or not math.isfinite(width) or width <= 1e-4:
        return None
    aspect_ratio = length / width
    if aspect_ratio < float(min_aspect_ratio):
        return None

    return PlanarAxisEstimate(
        yaw=normalize_axial(math.atan2(long_axis[1], long_axis[0])),
        length=length,
        width=width,
        aspect_ratio=aspect_ratio,
        point_count=int(points.shape[0]),
    )


class AxialAngleFilter:
    """Windowed circular filter for an undirected (pi-periodic) axis."""

    def __init__(self, window_size: int, reset_angle: float, reset_distance: float):
        if window_size < 1 or reset_angle <= 0.0 or reset_distance <= 0.0:
            raise ValueError("invalid axial filter configuration")
        self._angles: deque[float] = deque(maxlen=int(window_size))
        self._reset_angle = float(reset_angle)
        self._reset_distance = float(reset_distance)
        self._center: np.ndarray | None = None

    def clear(self) -> None:
        self._angles.clear()
        self._center = None

    def update(self, angle: float, center_xy: np.ndarray) -> tuple[float, float, int]:
        angle = normalize_axial(angle)
        center = np.asarray(center_xy, dtype=np.float64).reshape(2)
        if not np.all(np.isfinite(center)):
            raise ValueError("center must be finite")
        if self._angles and (
            axial_difference(angle, self._angles[-1]) > self._reset_angle
            or np.linalg.norm(center - self._center) > self._reset_distance
        ):
            self.clear()
        self._angles.append(angle)
        self._center = center.copy()

        doubled = np.asarray(self._angles, dtype=np.float64) * 2.0
        cosine = float(np.mean(np.cos(doubled)))
        sine = float(np.mean(np.sin(doubled)))
        resultant = min(1.0, max(1e-12, math.hypot(cosine, sine)))
        mean = normalize_axial(0.5 * math.atan2(sine, cosine))
        standard_deviation = 0.5 * math.sqrt(max(0.0, -2.0 * math.log(resultant)))
        return mean, standard_deviation, len(self._angles)
