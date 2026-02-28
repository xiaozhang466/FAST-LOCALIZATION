#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
1) 拼接 FAST-LOCALIZATION 的逐帧 PCD + 位姿，生成全局点云 global_map.pcd。
2) 读取全局点云，依据雷达位姿切除两侧 0.8 m 的走廊，压扁生成 2D 栅格地图（PGM+YAML+PNG）。
"""

import os
import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree
from PIL import Image

# 配置：输入地图目录（含 pcd/ 与 pose.json）和输出目录
FAST_LOCALIZATION_MAP_DIR = "/home/ros/fast_localization_ws/src/FAST-LOCALIZATION/map"
GLOBAL_MAP_OUTPUT_DIR = "/home/ros/fast_localization_ws/src/global_map"

# 走廊半径（米）：删除轨迹两侧该距离内的点
CORRIDOR_RADIUS = 0.8
# 栅格分辨率（米/像素）
GRID_RESOLUTION = 0.05
# 栅格文件名
NAV_MAP_PGM = "nav_map.pgm"
NAV_MAP_YAML = "nav_map.yaml"
NAV_MAP_PNG = "nav_map.png"


def load_fast_localization_poses(pose_file: str):
    """读取 pose.json，返回 [(t, q)]，q 顺序为 (qw, qx, qy, qz)。"""
    poses = []
    with open(pose_file, "r") as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) != 7:
                continue
            tx, ty, tz = map(float, parts[:3])
            qw, qx, qy, qz = map(float, parts[3:])
            poses.append(((tx, ty, tz), (qw, qx, qy, qz)))
    return poses


def stitch_global_map(map_dir: str, output_dir: str) -> str:
    """按 pose.json 变换 pcd/*.pcd 并拼接，保存为 global_map.pcd。"""
    pose_file = os.path.join(map_dir, "pose.json")
    pcd_dir = os.path.join(map_dir, "pcd")
    if not os.path.exists(pose_file):
        raise FileNotFoundError(f"pose.json not found: {pose_file}")
    if not os.path.isdir(pcd_dir):
        raise FileNotFoundError(f"pcd folder not found: {pcd_dir}")

    poses = load_fast_localization_poses(pose_file)
    if len(poses) == 0:
        raise ValueError(f"No valid poses found in {pose_file}")

    os.makedirs(output_dir, exist_ok=True)
    global_pcd = o3d.geometry.PointCloud()

    for idx, (t, q) in enumerate(poses):
        pcd_path = os.path.join(pcd_dir, f"{idx}.pcd")
        if not os.path.exists(pcd_path):
            print(f"Warning: missing {pcd_path}, skip.")
            continue
        pcd = o3d.io.read_point_cloud(pcd_path)

        # 世界 <- 雷达
        qw, qx, qy, qz = q
        R = o3d.geometry.get_rotation_matrix_from_quaternion([qw, qx, qy, qz])
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = np.array(t, dtype=np.float64)
        pcd.transform(T)
        global_pcd += pcd

        if (idx + 1) % 200 == 0:
            print(f"  stitched {idx + 1}/{len(poses)} frames...")

    output_pcd = os.path.join(output_dir, "global_map.pcd")
    o3d.io.write_point_cloud(output_pcd, global_pcd)
    print(f"Global map saved: {output_pcd}")
    return output_pcd


def filter_by_height(points: np.ndarray, min_z: float, max_z: float) -> np.ndarray:
    mask = (points[:, 2] >= min_z) & (points[:, 2] <= max_z)
    return points[mask]


def cut_corridor(points: np.ndarray, trajectory_xy: np.ndarray, radius: float) -> np.ndarray:
    tree = cKDTree(trajectory_xy)
    dists, _ = tree.query(points[:, :2], k=1)
    mask = dists >= radius
    return points[mask]


def create_occupancy_grid(points_xy: np.ndarray, resolution: float, margin: float = 1.0):
    if len(points_xy) == 0:
        raise ValueError("No points to create occupancy grid")

    x_min, y_min = points_xy.min(axis=0) - margin
    x_max, y_max = points_xy.max(axis=0) + margin

    width = int(np.ceil((x_max - x_min) / resolution))
    height = int(np.ceil((y_max - y_min) / resolution))

    grid = np.ones((height, width), dtype=np.uint8) * 254  # 254=free
    cols = ((points_xy[:, 0] - x_min) / resolution).astype(int)
    rows = ((points_xy[:, 1] - y_min) / resolution).astype(int)

    valid = (cols >= 0) & (cols < width) & (rows >= 0) & (rows < height)
    grid[rows[valid], cols[valid]] = 0  # occupied

    origin = (x_min, y_min, 0.0)
    return grid, origin


def save_maps(grid: np.ndarray, origin, resolution: float, out_dir: str):
    os.makedirs(out_dir, exist_ok=True)
    grid_flip = np.flipud(grid)
    pgm_path = os.path.join(out_dir, NAV_MAP_PGM)
    yaml_path = os.path.join(out_dir, NAV_MAP_YAML)
    png_path = os.path.join(out_dir, NAV_MAP_PNG)

    Image.fromarray(grid_flip).save(pgm_path)
    with open(yaml_path, "w") as f:
        f.write(f"image: {NAV_MAP_PGM}\n")
        f.write(f"resolution: {resolution}\n")
        f.write(f"origin: [{origin[0]}, {origin[1]}, {origin[2]}]\n")
        f.write("negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")
    Image.fromarray(grid_flip).save(png_path)

    print(f"Saved: {pgm_path}, {yaml_path}, {png_path}")


def main():
    print("=== Stitch global map and build 2D grid ===")
    print(f"Input map dir : {FAST_LOCALIZATION_MAP_DIR}")
    print(f"Output dir    : {GLOBAL_MAP_OUTPUT_DIR}")

    # 1) 拼接全局点云
    stitched_pcd = stitch_global_map(FAST_LOCALIZATION_MAP_DIR, GLOBAL_MAP_OUTPUT_DIR)

    # 2) 加载点云与轨迹
    pose_file = os.path.join(FAST_LOCALIZATION_MAP_DIR, "pose.json")
    poses = load_fast_localization_poses(pose_file)
    traj_xy = np.array([p[0][:2] for p in poses], dtype=np.float64)

    pcd = o3d.io.read_point_cloud(stitched_pcd)
    points = np.asarray(pcd.points)
    print(f"Loaded stitched PCD: {points.shape[0]} points")

    # 3) 删除走廊点
    points = cut_corridor(points, traj_xy, CORRIDOR_RADIUS)
    print(f"After corridor cut (radius={CORRIDOR_RADIUS} m): {points.shape[0]} points")

    # 4) 生成并保存 2D 栅格
    grid, origin = create_occupancy_grid(points[:, :2], GRID_RESOLUTION)
    save_maps(grid, origin, GRID_RESOLUTION, GLOBAL_MAP_OUTPUT_DIR)

    print("Done.")


if __name__ == "__main__":
    main()
