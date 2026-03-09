#!/usr/bin/env python3
"""PUTN-style local traversability estimation for LiDAR maps.

This module does not depend on the full PUTN stack. It reuses the core idea:
- fit a local plane around candidate points
- evaluate traversability using slope, roughness, and density
- expand from trajectory centerline to estimate adaptive corridor widths
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Tuple

import numpy as np
from scipy.spatial import cKDTree


@dataclass
class TraversabilityConfig:
    local_radius: float = 0.8
    ground_quantile: float = 0.2
    ground_band: float = 0.25
    min_points: int = 30
    min_ground_points: int = 20
    max_slope_deg: float = 18.0
    max_roughness: float = 0.08
    min_density: float = 20.0
    search_step: float = 0.1
    min_width: float = 0.6
    max_width: float = 2.0
    max_consecutive_fail: int = 2
    tangent_lookahead: int = 3
    smooth_window: int = 9
    fallback_width: float = 0.8
    progress_every: int = 50


@dataclass
class TraversabilityMetrics:
    traversable: bool
    slope_deg: float
    roughness: float
    density: float
    num_points: int
    num_ground_points: int


class LocalPlaneTraversability:
    """Evaluate local traversability around 2D query points."""

    def __init__(self, points_xyz: np.ndarray, cfg: TraversabilityConfig):
        if points_xyz.ndim != 2 or points_xyz.shape[1] != 3:
            raise ValueError("points_xyz must have shape (N, 3)")
        if len(points_xyz) == 0:
            raise ValueError("points_xyz is empty")
        self.points_xyz = points_xyz
        self.cfg = cfg
        self.xy_tree = cKDTree(points_xyz[:, :2])

    def evaluate_xy(self, query_xy: np.ndarray) -> TraversabilityMetrics:
        idx = self.xy_tree.query_ball_point(query_xy, self.cfg.local_radius)
        num_points = len(idx)
        if num_points < self.cfg.min_points:
            return TraversabilityMetrics(
                traversable=False,
                slope_deg=math.inf,
                roughness=math.inf,
                density=0.0,
                num_points=num_points,
                num_ground_points=0,
            )

        local_pts = self.points_xyz[idx]
        z_ref = np.quantile(local_pts[:, 2], self.cfg.ground_quantile)
        lower = z_ref - 0.5 * self.cfg.ground_band
        upper = z_ref + self.cfg.ground_band
        ground_mask = (local_pts[:, 2] >= lower) & (local_pts[:, 2] <= upper)
        ground_pts = local_pts[ground_mask]

        if len(ground_pts) < self.cfg.min_ground_points:
            relaxed_mask = local_pts[:, 2] <= upper
            ground_pts = local_pts[relaxed_mask]
        num_ground = len(ground_pts)
        if num_ground < self.cfg.min_ground_points:
            return TraversabilityMetrics(
                traversable=False,
                slope_deg=math.inf,
                roughness=math.inf,
                density=0.0,
                num_points=num_points,
                num_ground_points=num_ground,
            )

        centroid = ground_pts.mean(axis=0)
        demean = ground_pts - centroid
        _, _, vh = np.linalg.svd(demean, full_matrices=False)
        normal = vh[-1]
        if normal[2] < 0:
            normal = -normal

        slope_deg = math.degrees(math.acos(np.clip(normal[2], -1.0, 1.0)))
        residuals = np.abs(demean @ normal)
        roughness = float(np.sqrt(np.mean(residuals ** 2)))
        density = float(num_ground) / (math.pi * self.cfg.local_radius ** 2)

        traversable = (
            slope_deg <= self.cfg.max_slope_deg
            and roughness <= self.cfg.max_roughness
            and density >= self.cfg.min_density
        )
        return TraversabilityMetrics(
            traversable=traversable,
            slope_deg=float(slope_deg),
            roughness=roughness,
            density=density,
            num_points=num_points,
            num_ground_points=num_ground,
        )

    def estimate_side_width(
        self, center_xy: np.ndarray, normal_xy: np.ndarray, side_sign: float
    ) -> float:
        best_width = 0.0
        fail_count = 0
        num_steps = int(np.floor(self.cfg.max_width / self.cfg.search_step))
        for step_idx in range(1, num_steps + 1):
            width = step_idx * self.cfg.search_step
            query_xy = center_xy + side_sign * width * normal_xy
            metrics = self.evaluate_xy(query_xy)
            if metrics.traversable:
                best_width = width
                fail_count = 0
            else:
                fail_count += 1
                if best_width > 0.0 and fail_count >= self.cfg.max_consecutive_fail:
                    break

        if best_width < self.cfg.min_width:
            return float(min(self.cfg.max_width, max(self.cfg.min_width, self.cfg.fallback_width)))
        return float(best_width)


def _compute_normals_2d(trajectory_xy: np.ndarray, lookahead: int) -> np.ndarray:
    n = len(trajectory_xy)
    normals = np.zeros((n, 2), dtype=np.float64)
    prev_normal = np.array([0.0, 1.0], dtype=np.float64)
    for i in range(n):
        i0 = max(0, i - lookahead)
        i1 = min(n - 1, i + lookahead)
        tangent = trajectory_xy[i1] - trajectory_xy[i0]
        tnorm = np.linalg.norm(tangent)
        if tnorm < 1e-9:
            normal = prev_normal
        else:
            tangent /= tnorm
            normal = np.array([-tangent[1], tangent[0]], dtype=np.float64)
            n_norm = np.linalg.norm(normal)
            if n_norm < 1e-9:
                normal = prev_normal
            else:
                normal /= n_norm
        normals[i] = normal
        prev_normal = normal
    return normals


def _smooth_1d(values: np.ndarray, window: int) -> np.ndarray:
    if window <= 1 or len(values) == 0:
        return values.copy()
    window = min(window, len(values))
    if window % 2 == 0:
        window = max(1, window - 1)
    if window <= 1:
        return values.copy()
    kernel = np.ones(window, dtype=np.float64) / float(window)
    pad = window // 2
    padded = np.pad(values, (pad, pad), mode="edge")
    return np.convolve(padded, kernel, mode="valid")


def estimate_width_profile(
    trajectory_xy: np.ndarray,
    evaluator: LocalPlaneTraversability,
    cfg: TraversabilityConfig,
    verbose: bool = True,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    if trajectory_xy.ndim != 2 or trajectory_xy.shape[1] != 2:
        raise ValueError("trajectory_xy must have shape (M, 2)")
    if len(trajectory_xy) < 2:
        raise ValueError("trajectory_xy needs at least 2 poses")

    normals = _compute_normals_2d(trajectory_xy, cfg.tangent_lookahead)
    left_widths = np.zeros(len(trajectory_xy), dtype=np.float64)
    right_widths = np.zeros(len(trajectory_xy), dtype=np.float64)

    for i, center_xy in enumerate(trajectory_xy):
        normal_xy = normals[i]
        left_widths[i] = evaluator.estimate_side_width(center_xy, normal_xy, side_sign=1.0)
        right_widths[i] = evaluator.estimate_side_width(center_xy, normal_xy, side_sign=-1.0)
        if verbose and ((i + 1) % cfg.progress_every == 0 or i == len(trajectory_xy) - 1):
            print(f"  width profile {i + 1}/{len(trajectory_xy)}")

    left_smooth = _smooth_1d(left_widths, cfg.smooth_window)
    right_smooth = _smooth_1d(right_widths, cfg.smooth_window)
    return left_smooth, right_smooth, normals
