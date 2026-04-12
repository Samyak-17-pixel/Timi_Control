"""
Map thrust force (N) to PWM (µs) for Blue Robotics T200 @ 16 V.

Reference (typical published values @ 16 V):
  Forward max ~ 5.25 kgf (~51.5 N)
  Reverse max ~ 4.1 kgf (~40.2 N)

PWM: neutral 1500 µs; user clip 1200–1800 µs.
"""

from __future__ import annotations

import numpy as np


def force_to_pwm_linear(
    force_n: float,
    f_forward_max_n: float,
    f_reverse_max_n: float,
    pwm_min: float,
    pwm_neutral: float,
    pwm_max: float,
) -> float:
    """
    Piecewise linear: positive force -> forward thrust, negative -> reverse.

    force_n : desired thrust along thruster +direction (N)
    """
    if force_n >= 0.0:
        if f_forward_max_n < 1e-9:
            return pwm_neutral
        a = min(1.0, max(0.0, force_n / f_forward_max_n))
        return pwm_neutral + a * (pwm_max - pwm_neutral)
    else:
        if f_reverse_max_n < 1e-9:
            return pwm_neutral
        a = min(1.0, max(0.0, (-force_n) / f_reverse_max_n))
        return pwm_neutral - a * (pwm_neutral - pwm_min)


def forces_to_pwm_array(
    forces: np.ndarray,
    f_forward_max_n: float,
    f_reverse_max_n: float,
    pwm_min: float,
    pwm_neutral: float,
    pwm_max: float,
) -> np.ndarray:
    out = np.zeros_like(forces, dtype=float)
    for i in range(len(forces)):
        out[i] = force_to_pwm_linear(
            float(forces[i]),
            f_forward_max_n,
            f_reverse_max_n,
            pwm_min,
            pwm_neutral,
            pwm_max,
        )
    return out
