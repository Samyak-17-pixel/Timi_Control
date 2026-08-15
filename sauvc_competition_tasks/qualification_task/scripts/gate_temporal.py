"""EMA smoothing for sequential frames (video / live)."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass
class SmoothedPoles:
    left_x: float
    right_x: float


class GateTemporalFilter:
    def __init__(
        self,
        alpha: float = 0.45,
        max_jump_frac: float = 0.32,
    ):
        self.alpha = float(alpha)
        self.max_jump_frac = float(max(0.0, max_jump_frac))
        self._sl: Optional[float] = None
        self._sr: Optional[float] = None

    def reset(self) -> None:
        self._sl = self._sr = None

    def update_two(
        self,
        left_x: float,
        right_x: float,
        *,
        frame_width: Optional[int] = None,
    ) -> Tuple[float, float]:
        """
        EMA on pole x. If ``max_jump_frac`` and ``frame_width`` are set, a measurement
        that jumps farther than ``max_jump_frac * width`` from the previous smoothed
        value resets the filter (avoids locking onto a border artefact after a real gate appears).
        """
        a = self.alpha
        if self._sl is None:
            self._sl, self._sr = left_x, right_x
        else:
            if (
                self.max_jump_frac > 0
                and frame_width is not None
                and frame_width > 0
            ):
                lim = self.max_jump_frac * float(frame_width)
                if (
                    abs(left_x - self._sl) > lim
                    or abs(right_x - self._sr) > lim
                ):
                    self._sl, self._sr = left_x, right_x
                    return self._sl, self._sr
            self._sl = a * self._sl + (1.0 - a) * left_x
            self._sr = a * self._sr + (1.0 - a) * right_x
        return self._sl, self._sr
