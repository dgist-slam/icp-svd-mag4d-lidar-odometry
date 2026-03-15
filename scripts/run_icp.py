#!/usr/bin/env python3
"""
ICP (Iterative Closest Point) LiDAR Odometry — Pre-computation Script
======================================================================
RT604 SLAM Course, DGIST

Reads viz_data.json, runs point-to-point ICP between consecutive frames,
composes global poses, and saves results to public/icp_result.json.

Usage:
    python scripts/run_icp.py
"""

import json
import numpy as np
import os
import time
from pathlib import Path

# ── Quaternion / Rotation utilities ──────────────────────────────────

def quat_to_rotation(qx, qy, qz, qw):
    """Quaternion (x,y,z,w) → 3x3 rotation matrix."""
    q = np.array([qx, qy, qz, qw], dtype=np.float64)
    q /= np.linalg.norm(q)
    x, y, z, w = q
    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
        [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
        [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])

def rotation_to_quat(R):
    """3x3 rotation matrix → quaternion (x,y,z,w)."""
    tr = np.trace(R)
    if tr > 0:
        s = 0.5 / np.sqrt(tr + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y, z, w])
    q /= np.linalg.norm(q)
    return q

def make_se3(R, t):
    """Build 4x4 SE(3) matrix from 3x3 R and 3-vector t."""
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T

def pose_dict_to_se3(p):
    """Convert pose dict {x,y,z,qx,qy,qz,qw} → 4x4 SE(3)."""
    R = quat_to_rotation(p['qx'], p['qy'], p['qz'], p['qw'])
    t = np.array([p['x'], p['y'], p['z']])
    return make_se3(R, t)

def se3_to_pose_dict(T):
    """4x4 SE(3) → pose dict {x,y,z,qx,qy,qz,qw}."""
    R = T[:3, :3]
    t = T[:3, 3]
    qx, qy, qz, qw = rotation_to_quat(R)
    return {
        'x': float(t[0]), 'y': float(t[1]), 'z': float(t[2]),
        'qx': float(qx), 'qy': float(qy), 'qz': float(qz), 'qw': float(qw),
    }


# ── ICP Algorithm ────────────────────────────────────────────────────

def filter_nonzero(points):
    """Remove [0,0,0] invalid points."""
    pts = np.array(points, dtype=np.float64)
    mask = np.any(pts != 0.0, axis=1)
    return pts[mask]

def nearest_neighbors_brute(src, tgt):
    """Brute-force nearest neighbor: for each src point, find closest tgt point.
    Returns indices and distances."""
    # src: (N, 3), tgt: (M, 3)
    # Use broadcasting for small point clouds (~1000 pts)
    diffs = src[:, None, :] - tgt[None, :, :]  # (N, M, 3)
    dists = np.sum(diffs ** 2, axis=2)           # (N, M)
    indices = np.argmin(dists, axis=1)            # (N,)
    min_dists = np.sqrt(dists[np.arange(len(src)), indices])
    return indices, min_dists

def icp_point_to_point(source, target, max_iter=50, tolerance=1e-6,
                        max_dist=5.0, min_inlier_ratio=0.3):
    """
    Point-to-point ICP.

    Args:
        source: (N, 3) points in frame k+1 (to be aligned to target)
        target: (M, 3) points in frame k (reference)
        max_iter: maximum iterations
        tolerance: convergence threshold (change in mean error)
        max_dist: max correspondence distance (outlier rejection)
        min_inlier_ratio: minimum ratio of inliers to consider valid

    Returns:
        R: 3x3 rotation matrix
        t: 3-vector translation
        iterations: number of iterations used
        final_error: final mean squared distance
        converged: bool
    """
    src = source.copy()
    R_total = np.eye(3)
    t_total = np.zeros(3)
    prev_error = float('inf')

    for i in range(max_iter):
        # 1. Find nearest neighbors
        indices, dists = nearest_neighbors_brute(src, target)

        # 2. Outlier rejection by distance threshold
        inlier_mask = dists < max_dist
        if np.sum(inlier_mask) < min_inlier_ratio * len(src):
            # Too few inliers, relax threshold
            inlier_mask = dists < np.percentile(dists, 80)

        src_matched = src[inlier_mask]
        tgt_matched = target[indices[inlier_mask]]

        if len(src_matched) < 10:
            break

        # 3. Compute centroids
        src_centroid = np.mean(src_matched, axis=0)
        tgt_centroid = np.mean(tgt_matched, axis=0)

        # 4. Center the points
        src_centered = src_matched - src_centroid
        tgt_centered = tgt_matched - tgt_centroid

        # 5. SVD to find optimal rotation (Arun et al. 1987)
        H = src_centered.T @ tgt_centered  # 3x3
        U, S, Vt = np.linalg.svd(H)
        R_step = Vt.T @ U.T

        # Ensure proper rotation (det = +1)
        if np.linalg.det(R_step) < 0:
            Vt[-1, :] *= -1
            R_step = Vt.T @ U.T

        # 6. Compute translation
        t_step = tgt_centroid - R_step @ src_centroid

        # 7. Apply transformation
        src = (R_step @ src.T).T + t_step

        # 8. Accumulate total transformation
        R_total = R_step @ R_total
        t_total = R_step @ t_total + t_step

        # 9. Check convergence
        mean_error = np.mean(dists[inlier_mask] ** 2)
        if abs(prev_error - mean_error) < tolerance:
            return R_total, t_total, i + 1, float(mean_error), True
        prev_error = mean_error

    return R_total, t_total, max_iter, float(prev_error), False


# ── Main Pipeline ────────────────────────────────────────────────────

def main():
    base_dir = Path(__file__).resolve().parent.parent
    data_path = base_dir / 'viz_data.json'
    output_path = base_dir / 'public' / 'icp_result.json'

    print(f"Loading data from {data_path}...")
    with open(data_path, 'r') as f:
        data = json.load(f)

    n_frames = len(data)
    print(f"Loaded {n_frames} frames")

    # ICP parameters
    params = {
        'max_iter': 50,
        'tolerance': 1e-6,
        'max_dist': 5.0,
    }

    # First frame: use GT pose as starting point (both GT and ICP start from same origin)
    first_gt = pose_dict_to_se3(data[0]['pose'])
    icp_global = first_gt.copy()  # ICP starts from GT frame 0

    frames = []
    total_start = time.time()

    for i in range(n_frames):
        frame = data[i]
        gt_pose = pose_dict_to_se3(frame['pose'])

        if i == 0:
            # First frame: no ICP needed
            frames.append({
                'scanIndex': frame['scanIndex'],
                'elapsed': frame['elapsed'],
                'gt_pose': se3_to_pose_dict(gt_pose),
                'icp_pose': se3_to_pose_dict(icp_global),
                'icp_iterations': 0,
                'icp_error': 0.0,
                'icp_converged': True,
                'n_points': int(np.sum(np.any(np.array(frame['points']) != 0, axis=1))),
            })
            continue

        # Get point clouds (filter zeros)
        pts_prev = filter_nonzero(data[i - 1]['points'])
        pts_curr = filter_nonzero(frame['points'])

        print(f"  Frame {frame['scanIndex']:3d}: {len(pts_curr):4d} pts", end='')

        # Run ICP: align current frame to previous frame (both in their local sensor frames)
        R_icp, t_icp, n_iter, err, converged = icp_point_to_point(
            pts_curr, pts_prev,
            max_iter=params['max_iter'],
            tolerance=params['tolerance'],
            max_dist=params['max_dist'],
        )

        # The ICP gives us: T_rel that maps frame_{k+1} → frame_{k}
        # Global pose: T_{k+1} = T_k * T_rel^{-1}
        # Because ICP aligns source to target, T_rel transforms source to target space
        # So: point_in_k = R_icp * point_in_{k+1} + t_icp
        # We want T_{k+1} in global frame:
        #   T_global_{k+1} = T_global_k * inv(T_rel_icp)
        # where T_rel_icp maps k+1 → k

        T_rel = make_se3(R_icp, t_icp)  # maps current → previous
        T_rel_inv = np.linalg.inv(T_rel)  # maps previous → current (motion)

        # Actually, the convention: ICP aligned current to previous
        # So the relative motion from frame k to frame k+1 is T_rel_inv
        # Global: T_{k+1} = T_k * T_rel_inv
        # Wait, let me think again...
        #
        # ICP: source=current, target=previous
        # Result: R_icp, t_icp such that R_icp @ current + t_icp ≈ previous
        # This means: point_prev ≈ R_icp @ point_curr + t_icp
        # So T_rel (current→previous) = [R_icp | t_icp]
        #
        # The motion from prev to curr in prev's frame:
        #   T_motion = inv(T_rel) = [R_icp^T | -R_icp^T @ t_icp]
        #
        # Global pose of current = Global_prev * T_motion
        T_motion = np.linalg.inv(T_rel)
        icp_global = icp_global @ T_motion

        status = "OK" if converged else "NC"
        print(f" | ICP: {n_iter:2d} iter, err={err:.4f} [{status}]")

        frames.append({
            'scanIndex': frame['scanIndex'],
            'elapsed': frame['elapsed'],
            'gt_pose': se3_to_pose_dict(gt_pose),
            'icp_pose': se3_to_pose_dict(icp_global),
            'icp_iterations': n_iter,
            'icp_error': round(err, 6),
            'icp_converged': converged,
            'n_points': int(len(pts_curr)),
        })

    elapsed = time.time() - total_start
    print(f"\nTotal ICP time: {elapsed:.2f}s")

    # Compute drift statistics
    gt_final = pose_dict_to_se3(frames[-1]['gt_pose'])
    icp_final = pose_dict_to_se3(frames[-1]['icp_pose'])
    pos_drift = np.linalg.norm(gt_final[:3, 3] - icp_final[:3, 3])
    print(f"Final position drift: {pos_drift:.3f} m")

    result = {
        'params': params,
        'n_frames': n_frames,
        'computation_time': round(elapsed, 3),
        'final_drift_m': round(pos_drift, 4),
        'frames': frames,
    }

    os.makedirs(output_path.parent, exist_ok=True)
    with open(output_path, 'w') as f:
        json.dump(result, f, indent=2)

    print(f"Results saved to {output_path}")


if __name__ == '__main__':
    main()
