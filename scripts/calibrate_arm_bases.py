#!/usr/bin/env python3
"""Calibrate the right arm base relative to the left using paired TCP points.

Run from the repository root:
    python3 scripts/calibrate_arm_bases.py

Edit left_points and right_points to provide corresponding measurements in metres.
The left base is the reference: p_left = R @ p_right + t.
Prints the transform, Xacro placement values and point-fit errors; does not
modify the robot configuration or publish motion commands.
"""

import numpy as np
from scipy.spatial.transform import Rotation


def rigid_transform(source_points, target_points):
    """
    Solve:

        p_target = R @ p_source + t
    """

    A = np.asarray(source_points, dtype=float)
    B = np.asarray(target_points, dtype=float)

    if A.shape != B.shape:
        raise ValueError("source_points and target_points must have the same shape")

    if A.ndim != 2 or A.shape[1] != 3:
        raise ValueError("Points must be Nx3")

    if len(A) < 3:
        raise ValueError("Need at least 3 corresponding points")

    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)

    AA = A - centroid_A
    BB = B - centroid_B

    if np.linalg.matrix_rank(AA) < 2:
        raise ValueError("Calibration points are degenerate or collinear")

    H = AA.T @ BB

    U, S, Vt = np.linalg.svd(H)

    R = Vt.T @ U.T

    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    t = centroid_B - R @ centroid_A

    return R, t, S


def main():

    # TCP positions expressed in LEFT base frame
    #2026/9 no extra extenssion for base
    # left_points = np.array([
    #     [0.483, -0.336, -0.056],  # P1
    #     [0.631, -0.337, -0.055],  # P2
    #     [0.634, -0.481, -0.053],  # P3
    #     [0.486, -0.485, -0.053],  # P4
    #     [0.560, -0.414, -0.054],  # P5
    # ])

    # # TCP positions expressed in RIGHT base frame
    # right_points = np.array([
    #     [-0.337, 0.465, -0.062],  # P1
    #     [-0.333, 0.613, -0.063],  # P2
    #     [-0.190, 0.615, -0.063],  # P3
    #     [-0.189, 0.466, -0.063],  # P4
    #     [-0.258, 0.541, -0.063],  # P5
    # ])

    left_points = np.array([
            [0.504, -0.368, -0.039],  # P1
            [0.652, -0.369, -0.038],  # P2
            [0.580, -0.446, -0.038],  # P3
            [0.506, -0.517, -0.038],  # P4
            [0.654, -0.512, -0.037],  # P5
        ])

    # TCP positions expressed in RIGHT base frame
    right_points = np.array([
            [-0.532, 0.485, -0.047], #P1
            [-0.527, 0.633, -0.048], #P2
            [-0.454, 0.560, -0.047], #P3
            [-0.385, 0.484, -0.048],
            [-0.384, 0.634, -0.048], #P5
    ])

    # ============================================================
    # Solve:
    #
    # p_left = R_left_right @ p_right + t_left_right
    #
    # Result:
    # ^left T_right
    #
    # LEFT robot is the reference frame.
    # ============================================================

    R, t, singular_values = rigid_transform(
        right_points,
        left_points
    )

    rpy = Rotation.from_matrix(R).as_euler("xyz")

    print("\n========================================")
    print("RIGHT BASE RELATIVE TO LEFT BASE")
    print("^left T_right")
    print("========================================")

    print("\nRotation matrix:")
    print(R)

    print("\nTranslation xyz [m]:")
    print(t)

    print("\nRPY [rad]:")
    print(rpy)

    print("\nRPY [deg]:")
    print(np.degrees(rpy))

    print("\nSVD singular values:")
    print(singular_values)

    # Homogeneous transform
    T_left_right = np.eye(4)
    T_left_right[:3, :3] = R
    T_left_right[:3, 3] = t

    print("\nHomogeneous transform ^left T_right:")
    print(T_left_right)

    # ============================================================
    # URDF / XACRO output
    # ============================================================

    print("\n========================================")
    print("URDF / XACRO VALUES")
    print("========================================")

    print("\nLeft robot stays as reference:")

    print(
        '<xacro:arg name="left_xyz" default="0 0 0"/>'
    )

    print(
        '<xacro:arg name="left_rpy" default="0 0 0"/>'
    )

    print("\nRight robot calibrated relative to left:")

    print(
        f'<xacro:arg name="right_xyz" '
        f'default="{t[0]:.8f} {t[1]:.8f} {t[2]:.8f}"/>'
    )

    print(
        f'<xacro:arg name="right_rpy" '
        f'default="{rpy[0]:.8f} {rpy[1]:.8f} {rpy[2]:.8f}"/>'
    )

    # ============================================================
    # Error evaluation
    # ============================================================

    predicted_left_points = (
        R @ right_points.T
    ).T + t

    errors = np.linalg.norm(
        predicted_left_points - left_points,
        axis=1
    )

    print("\n========================================")
    print("CALIBRATION ERROR")
    print("========================================")

    for i, e in enumerate(errors):
        print(f"P{i + 1}: {e * 1000.0:.3f} mm")

    rmse = np.sqrt(np.mean(errors ** 2))
    max_error = np.max(errors)

    print(f"\nRMSE: {rmse * 1000.0:.3f} mm")
    print(f"Max error: {max_error * 1000.0:.3f} mm")


if __name__ == "__main__":
    main()
