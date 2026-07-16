"""
Pure coordinate-frame math: the rigid transform between Motive's mocap world
frame and the UR12e's base frame.

    p_base = R @ p_mocap + t

No I/O here on purpose (see CLAUDE.md "Catch Integration") - calibrate_frames.py
does the robot/NatNet talking and calls umeyama_rigid_transform() on the
collected point pairs; catch.py will later import mocap_point_to_base() to
convert a predicted catch point. Kept separate and dependency-free (just
numpy) so it's trivially unit-testable.

Math: Umeyama (1991) / Kabsch point-set registration, no scaling (both frames
are already real-world meters). Given N corresponding point pairs
(source_i, destination_i), it finds the rotation R and translation t
minimizing sum_i || destination_i - (R @ source_i + t) ||^2 in closed form via
SVD - see the derivation in the docstring of umeyama_rigid_transform.
"""

from typing import Tuple

import numpy as np


def umeyama_rigid_transform(source: np.ndarray, destination: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Best-fit rotation + translation mapping `source` points onto `destination` points.

    Solves, in the least-squares sense:

        destination_i ~= R @ source_i + t

    `source` and `destination` must be (N, 3) arrays of N >= 3 corresponding
    points, same N, same order (point i in one array corresponds to point i
    in the other). Returns (R, t): R is a (3, 3) proper rotation matrix
    (orthonormal, det=+1), t is a (3,) translation vector.

    Derivation sketch (standard orthogonal-Procrustes result): for fixed R,
    the optimal t is destination_mean - R @ source_mean. Substituting that
    back turns the problem into maximizing trace(R @ M) over rotations R,
    where M = sum_i source_i' @ destination_i'^T (centered coordinates). With
    SVD M = U @ diag(S) @ Vt, the maximizer is R = V @ diag(1,1,d) @ U^T,
    d = sign(det(V @ U^T)), the last diagonal entry being flipped only when
    needed to keep det(R) = +1 (a proper rotation, not a reflection).

    This function computes it via H = M^T = destination_centered^T @
    source_centered instead of M itself, purely so a direct
    `np.linalg.svd(H)` call already returns (U, S, Vt) in the roles the
    formula above needs (see self-test below for numeric verification of the
    whole thing against known ground-truth transforms, including a Y-up to
    Z-up axis swap matching Motive vs. the robot's base frame convention).
    """
    source = np.asarray(source, dtype=float)
    destination = np.asarray(destination, dtype=float)
    if source.shape != destination.shape or source.ndim != 2 or source.shape[1] != 3:
        raise ValueError("source and destination must both be (N, 3) arrays of the same shape")
    if source.shape[0] < 3:
        raise ValueError("need at least 3 point pairs to solve a 3D rigid transform")

    src_mean = source.mean(axis=0)
    dst_mean = destination.mean(axis=0)
    src_centered = source - src_mean
    dst_centered = destination - dst_mean

    H = dst_centered.T @ src_centered  # (3, 3)

    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(U @ Vt))
    if d == 0:
        d = 1.0  # degenerate (near-singular H) - keep a proper rotation rather than dividing by zero elsewhere
    correction = np.diag([1.0, 1.0, d])
    R = U @ correction @ Vt

    t = dst_mean - R @ src_mean
    return R, t


def mocap_point_to_base(p_mocap: np.ndarray, R: np.ndarray, t: np.ndarray) -> np.ndarray:
    """Transform mocap-frame point(s) into base-frame.

    `p_mocap` may be a single (3,) point or an (N, 3) array of points -
    both work via the same `p @ R.T + t` broadcasting.
    """
    p_mocap = np.asarray(p_mocap, dtype=float)
    return p_mocap @ R.T + t


def _random_rotation(rng: np.random.Generator) -> np.ndarray:
    """A uniformly random proper rotation matrix, via QR of a random Gaussian matrix."""
    a = rng.normal(size=(3, 3))
    q, r = np.linalg.qr(a)
    q = q @ np.diag(np.sign(np.diag(r)))  # fix QR's sign ambiguity
    if np.linalg.det(q) < 0:
        q[:, 0] *= -1  # flip one column to force a proper rotation, not a reflection
    return q


if __name__ == "__main__":
    # Self-test: verify the fit recovers known ground-truth transforms, since
    # a transpose/sign slip in a Kabsch/Umeyama implementation is easy to make
    # and easy to miss just by eyeballing the formula - same spirit as
    # trajectory.py's synthetic self-test.
    rng = np.random.default_rng(42)

    def check(name, R_true, t_true, n_points=30, noise_m=0.0003):
        source = rng.uniform(-1.0, 1.0, size=(n_points, 3))
        destination = (source @ R_true.T + t_true) + rng.normal(0, noise_m, size=(n_points, 3))

        R_fit, t_fit = umeyama_rigid_transform(source, destination)

        predicted = mocap_point_to_base(source, R_fit, t_fit)
        rmse = float(np.sqrt(np.mean(np.sum((predicted - destination) ** 2, axis=1))))

        r_err = np.max(np.abs(R_fit - R_true))
        t_err = np.max(np.abs(t_fit - t_true))
        orthogonality_err = np.max(np.abs(R_fit @ R_fit.T - np.eye(3)))
        det_err = abs(np.linalg.det(R_fit) - 1.0)

        print(f"[{name}] rmse={rmse * 1000:.3f}mm  max|R_fit-R_true|={r_err:.5f}  "
              f"max|t_fit-t_true|={t_err:.5f}  orthogonality_err={orthogonality_err:.2e}  "
              f"det_err={det_err:.2e}")

        assert rmse < 0.005, f"{name}: fit RMSE too large ({rmse:.5f} m)"
        assert r_err < 0.01, f"{name}: recovered rotation doesn't match ground truth"
        assert t_err < 0.01, f"{name}: recovered translation doesn't match ground truth"
        assert orthogonality_err < 1e-8, f"{name}: R_fit is not orthonormal"
        assert det_err < 1e-8, f"{name}: R_fit is a reflection, not a proper rotation (det != +1)"

    # 1) A generic random rigid transform.
    check("random rotation", _random_rotation(rng), rng.uniform(-2.0, 2.0, size=3))

    # 2) Identity - the trivial case (both frames already aligned).
    check("identity", np.eye(3), np.zeros(3))

    # 3) The actual case this project cares about: Motive is Y-up, the UR
    # base frame is Z-up. A pure axis-remap (mocap X -> base Y, mocap Y ->
    # base Z, mocap Z -> base X - a cyclic permutation, so it's handedness-
    # preserving/det=+1, unlike a plain two-axis swap which is a reflection)
    # plus a translation - this is the kind of transform calibrate_frames.py
    # should recover if the rig is mounted with no additional tilt.
    y_up_to_z_up = np.array([
        [0.0, 0.0, 1.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
    ])
    check("Y-up mocap -> Z-up base axis swap", y_up_to_z_up, np.array([0.8, -0.3, 0.05]))

    # 4) Only 3 points (the hard floor) - should still solve exactly (no noise).
    check("minimal 3-point fit", _random_rotation(rng), rng.uniform(-1, 1, size=3),
          n_points=3, noise_m=0.0)

    print("\nself-test passed")
