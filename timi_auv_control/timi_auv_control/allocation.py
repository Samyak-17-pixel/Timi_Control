"""Thrust allocation: wrench (6) -> per-thruster forces (N), with saturation."""

from __future__ import annotations

import numpy as np


def allocate_wrench(
    wrench: np.ndarray,
    B: np.ndarray,
    f_min: np.ndarray,
    f_max: np.ndarray,
) -> tuple[np.ndarray, bool]:
    """
    wrench = B @ f. Use Moore-Penrose pseudoinverse then clip to [f_min, f_max].

    Returns (f, saturated) where saturated is True if any bound active.
    """
    if wrench.shape != (6,):
        raise ValueError("wrench must be (6,)")

    f = np.linalg.pinv(B) @ wrench
    f_clipped = np.clip(f, f_min, f_max)
    saturated = not np.allclose(f, f_clipped, rtol=0, atol=1e-6)
    return f_clipped, saturated
