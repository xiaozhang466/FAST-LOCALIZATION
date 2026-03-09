#!/usr/bin/env python3
"""Generate an adaptive traversable-corridor nav map from trajectory + global PCD.

Pipeline:
1) load trajectory from pose.json
2) load global point cloud
3) estimate adaptive left/right corridor width along trajectory
4) remove points inside the traversable corridor
5) project remaining points to 2D occupancy grid
"""

from __future__ import annotations

import argparse
import csv
import os
from dataclasses import asdict

import numpy as np
import open3d as o3d
from PIL import Image
from scipy.spatial import cKDTree

from putn_traversability import (
    LocalPlaneTraversability,
    TraversabilityConfig,
    estimate_width_profile,
)


DEFAULT_MAP_DIR = "/home/ros/ZMG/fast_localization_ws/src/FAST-LOCALIZATION/map"
DEFAULT_GLOBAL_PCD = "/home/ros/ZMG/fast_localization_ws/src/global_map/global_map.pcd"
DEFAULT_OUTPUT_DIR = "/home/ros/ZMG/fast_localization_ws/src/global_map/adaptive_debug"

NAV_MAP_PGM = "nav_map_adaptive.pgm"
NAV_MAP_YAML = "nav_map_adaptive.yaml"
NAV_MAP_PNG = "nav_map_adaptive.png"
WIDTH_PROFILE_CSV = "adaptive_width_profile.csv"
FIXED_WIDTH_TXT = "fixed_widths.txt"
CORRIDOR_PCD = "adaptive_corridor_points.pcd"
OBSTACLE_PCD = "adaptive_obstacle_points.pcd"


def load_fast_localization_poses(pose_file: str):
    poses = []
    with open(pose_file, "r", encoding="utf-8") as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) != 7:
                continue
            tx, ty, tz = map(float, parts[:3])
            qw, qx, qy, qz = map(float, parts[3:])
            poses.append(((tx, ty, tz), (qw, qx, qy, qz)))
    return poses


def create_occupancy_grid(points_xy: np.ndarray, resolution: float, margin: float = 1.0):
    if len(points_xy) == 0:
        raise ValueError("No points to create occupancy grid")

    x_min, y_min = points_xy.min(axis=0) - margin
    x_max, y_max = points_xy.max(axis=0) + margin
    width = int(np.ceil((x_max - x_min) / resolution))
    height = int(np.ceil((y_max - y_min) / resolution))

    grid = np.ones((height, width), dtype=np.uint8) * 254
    cols = ((points_xy[:, 0] - x_min) / resolution).astype(int)
    rows = ((points_xy[:, 1] - y_min) / resolution).astype(int)
    valid = (cols >= 0) & (cols < width) & (rows >= 0) & (rows < height)
    grid[rows[valid], cols[valid]] = 0

    origin = (float(x_min), float(y_min), 0.0)
    return grid, origin


def save_maps(grid: np.ndarray, origin, resolution: float, out_dir: str):
    os.makedirs(out_dir, exist_ok=True)
    grid_flip = np.flipud(grid)

    pgm_path = os.path.join(out_dir, NAV_MAP_PGM)
    yaml_path = os.path.join(out_dir, NAV_MAP_YAML)
    png_path = os.path.join(out_dir, NAV_MAP_PNG)

    Image.fromarray(grid_flip).save(pgm_path)
    Image.fromarray(grid_flip).save(png_path)

    with open(yaml_path, "w", encoding="utf-8") as f:
        f.write(f"image: {NAV_MAP_PGM}\n")
        f.write(f"resolution: {resolution}\n")
        f.write(f"origin: [{origin[0]}, {origin[1]}, {origin[2]}]\n")
        f.write("negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")

    print(f"Saved map: {pgm_path}")
    print(f"Saved map: {yaml_path}")
    print(f"Saved map: {png_path}")


def save_width_profile_csv(
    csv_path: str,
    traj_xy: np.ndarray,
    left: np.ndarray,
    right: np.ndarray,
    fixed_left: float | None = None,
    fixed_right: float | None = None,
    fixed_radius: float | None = None,
):
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "idx",
                "x",
                "y",
                "left_width",
                "right_width",
                "fixed_left_width",
                "fixed_right_width",
                "fixed_radius",
            ]
        )
        for i, (xy, lw, rw) in enumerate(zip(traj_xy, left, right)):
            writer.writerow(
                [
                    i,
                    float(xy[0]),
                    float(xy[1]),
                    float(lw),
                    float(rw),
                    "" if fixed_left is None else float(fixed_left),
                    "" if fixed_right is None else float(fixed_right),
                    "" if fixed_radius is None else float(fixed_radius),
                ]
            )
    print(f"Saved profile: {csv_path}")


def save_pcd(points_xyz: np.ndarray, pcd_path: str):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points_xyz)
    o3d.io.write_point_cloud(pcd_path, pcd)
    print(f"Saved pcd: {pcd_path}")


def cut_points_inside_fixed_radius(
    points_xyz: np.ndarray,
    trajectory_xy: np.ndarray,
    radius: float,
):
    if len(trajectory_xy) == 0:
        raise ValueError("trajectory_xy is empty")
    if radius <= 0:
        raise ValueError("radius must be > 0")
    point_xy = points_xyz[:, :2]
    traj_tree = cKDTree(trajectory_xy)
    dists, _ = traj_tree.query(point_xy, k=1)
    inside = dists <= radius
    outside_points = points_xyz[~inside]
    inside_points = points_xyz[inside]
    return outside_points, inside_points


def estimate_fixed_width(
    widths: np.ndarray,
    quantile: float,
    lower: float,
    upper: float,
    saturation_eps: float,
):
    finite_mask = np.isfinite(widths)
    finite = widths[finite_mask]
    if len(finite) == 0:
        raise ValueError("No finite widths for fixed-width estimation")

    # Ignore max-width saturated samples; in open fields they tend to over-expand.
    valid = finite[finite < (upper - saturation_eps)]
    if len(valid) < max(20, int(0.1 * len(finite))):
        valid = finite

    q = float(np.quantile(valid, quantile))
    fixed = float(np.clip(q, lower, upper))
    return fixed, len(valid), len(finite)


def parse_args():
    parser = argparse.ArgumentParser(description="Build adaptive-corridor nav map.")
    parser.add_argument("--pose-file", default=os.path.join(DEFAULT_MAP_DIR, "pose.json"))
    parser.add_argument("--global-pcd", default=DEFAULT_GLOBAL_PCD)
    parser.add_argument("--output-dir", default=DEFAULT_OUTPUT_DIR)

    parser.add_argument("--grid-resolution", type=float, default=0.05)
    parser.add_argument("--grid-margin", type=float, default=1.0)
    parser.add_argument("--eval-voxel", type=float, default=0.15)

    parser.add_argument("--local-radius", type=float, default=0.8)
    parser.add_argument("--ground-quantile", type=float, default=0.2)
    parser.add_argument("--ground-band", type=float, default=0.25)
    parser.add_argument("--min-points", type=int, default=30)
    parser.add_argument("--min-ground-points", type=int, default=20)
    parser.add_argument("--max-slope-deg", type=float, default=18.0)
    parser.add_argument("--max-roughness", type=float, default=0.08)
    parser.add_argument("--min-density", type=float, default=20.0)

    parser.add_argument("--search-step", type=float, default=0.1)
    parser.add_argument("--min-width", type=float, default=0.5)
    parser.add_argument("--max-width", type=float, default=2.0)
    parser.add_argument("--fallback-width", type=float, default=0.8)
    parser.add_argument("--max-consecutive-fail", type=int, default=2)
    parser.add_argument("--tangent-lookahead", type=int, default=3)
    parser.add_argument("--smooth-window", type=int, default=9)
    parser.add_argument("--progress-every", type=int, default=50)
    parser.add_argument("--fixed-width-quantile", type=float, default=0.65)
    parser.add_argument("--fixed-width-lower", type=float, default=0.5)
    parser.add_argument("--fixed-width-upper", type=float, default=2.0)
    parser.add_argument("--saturation-eps", type=float, default=0.05)
    parser.add_argument(
        "--fixed-radius-mode",
        choices=["min", "mean", "max"],
        default="mean",
        help="How to convert fixed left/right widths into one radius for trajectory-distance cut.",
    )
    parser.add_argument("--save-debug-pcd", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    os.makedirs(args.output_dir, exist_ok=True)

    poses = load_fast_localization_poses(args.pose_file)
    if len(poses) < 2:
        raise ValueError(f"Not enough poses in {args.pose_file}")
    trajectory_xy = np.array([p[0][:2] for p in poses], dtype=np.float64)
    print(f"Loaded poses: {len(trajectory_xy)}")

    pcd = o3d.io.read_point_cloud(args.global_pcd)
    points_xyz = np.asarray(pcd.points)
    if len(points_xyz) == 0:
        raise ValueError(f"Empty point cloud: {args.global_pcd}")
    print(f"Loaded global pcd: {args.global_pcd} ({len(points_xyz)} points)")

    eval_points = points_xyz
    if args.eval_voxel > 0.0:
        eval_pcd = pcd.voxel_down_sample(args.eval_voxel)
        eval_points = np.asarray(eval_pcd.points)
        print(f"Evaluation downsample: {len(eval_points)} points (voxel={args.eval_voxel})")

    cfg = TraversabilityConfig(
        local_radius=args.local_radius,
        ground_quantile=args.ground_quantile,
        ground_band=args.ground_band,
        min_points=args.min_points,
        min_ground_points=args.min_ground_points,
        max_slope_deg=args.max_slope_deg,
        max_roughness=args.max_roughness,
        min_density=args.min_density,
        search_step=args.search_step,
        min_width=args.min_width,
        max_width=args.max_width,
        max_consecutive_fail=args.max_consecutive_fail,
        tangent_lookahead=args.tangent_lookahead,
        smooth_window=args.smooth_window,
        fallback_width=args.fallback_width,
        progress_every=args.progress_every,
    )
    print(f"Traversability config: {asdict(cfg)}")

    evaluator = LocalPlaneTraversability(eval_points, cfg)
    left_widths, right_widths, _ = estimate_width_profile(
        trajectory_xy, evaluator, cfg, verbose=True
    )
    print(
        "Adaptive widths summary: "
        f"left mean={left_widths.mean():.3f}m, right mean={right_widths.mean():.3f}m"
    )

    fixed_left, valid_left, total_left = estimate_fixed_width(
        left_widths,
        quantile=args.fixed_width_quantile,
        lower=args.fixed_width_lower,
        upper=args.fixed_width_upper,
        saturation_eps=args.saturation_eps,
    )
    fixed_right, valid_right, total_right = estimate_fixed_width(
        right_widths,
        quantile=args.fixed_width_quantile,
        lower=args.fixed_width_lower,
        upper=args.fixed_width_upper,
        saturation_eps=args.saturation_eps,
    )
    print(
        "Fixed widths (from plane-fit profile): "
        f"left={fixed_left:.3f}m ({valid_left}/{total_left} valid), "
        f"right={fixed_right:.3f}m ({valid_right}/{total_right} valid), "
        f"q={args.fixed_width_quantile}"
    )

    if args.fixed_radius_mode == "min":
        fixed_radius = min(fixed_left, fixed_right)
    elif args.fixed_radius_mode == "max":
        fixed_radius = max(fixed_left, fixed_right)
    else:
        fixed_radius = 0.5 * (fixed_left + fixed_right)
    print(f"Fixed radius for trajectory cut: {fixed_radius:.3f}m (mode={args.fixed_radius_mode})")

    obstacle_points, corridor_points = cut_points_inside_fixed_radius(
        points_xyz, trajectory_xy, fixed_radius
    )
    print(
        f"Corridor points removed: {len(corridor_points)}, "
        f"obstacle points kept: {len(obstacle_points)}"
    )
    if len(obstacle_points) == 0:
        raise ValueError("No obstacle points left after adaptive corridor cut")

    grid, origin = create_occupancy_grid(
        obstacle_points[:, :2], resolution=args.grid_resolution, margin=args.grid_margin
    )
    save_maps(grid, origin, args.grid_resolution, args.output_dir)
    save_width_profile_csv(
        os.path.join(args.output_dir, WIDTH_PROFILE_CSV),
        trajectory_xy,
        left_widths,
        right_widths,
        fixed_left,
        fixed_right,
        fixed_radius,
    )
    with open(os.path.join(args.output_dir, FIXED_WIDTH_TXT), "w", encoding="utf-8") as f:
        f.write(f"fixed_left_width: {fixed_left}\n")
        f.write(f"fixed_right_width: {fixed_right}\n")
        f.write(f"fixed_radius: {fixed_radius}\n")
        f.write(f"fixed_radius_mode: {args.fixed_radius_mode}\n")
        f.write(f"quantile: {args.fixed_width_quantile}\n")
        f.write(f"bounds: [{args.fixed_width_lower}, {args.fixed_width_upper}]\n")
    print(f"Saved fixed widths: {os.path.join(args.output_dir, FIXED_WIDTH_TXT)}")

    if args.save_debug_pcd:
        save_pcd(corridor_points, os.path.join(args.output_dir, CORRIDOR_PCD))
        save_pcd(obstacle_points, os.path.join(args.output_dir, OBSTACLE_PCD))

    print("Done.")


if __name__ == "__main__":
    main()
