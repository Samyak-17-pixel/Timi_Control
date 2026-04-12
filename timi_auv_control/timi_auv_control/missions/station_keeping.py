"""Station keeping / hover: hold NED pose for duration or indefinitely."""

from __future__ import annotations

import math
from typing import Any, Dict

import numpy as np

from .base import MissionBase, MissionCommand, VehicleState


class StationKeepingMission(MissionBase):
    """
    Hold position (NED), depth, and attitude (roll, pitch, yaw).

    YAML keys:
      position_ned: [north, east, down] (m)
      attitude_deg: { roll, pitch, yaw } optional, default 0
      duration_s: float | null — if null or negative, hold until node stops
    """

    def __init__(self, cfg: Dict[str, Any]):
        super().__init__(cfg)
        self._p_des = np.array(cfg["position_ned"], dtype=float)
        att = cfg.get("attitude_deg", {})
        self._roll_des = math.radians(float(att.get("roll", 0.0)))
        self._pitch_des = math.radians(float(att.get("pitch", 0.0)))
        self._yaw_des = math.radians(float(att.get("yaw", 0.0)))
        ds = cfg.get("duration_s", None)
        self._duration = None if ds is None else float(ds)
        if self._duration is not None and self._duration < 0:
            self._duration = None
        self._t0: float | None = None

    def reset(self) -> None:
        self._t0 = None

    def step(self, state: VehicleState, dt: float) -> MissionCommand:
        if self._t0 is None:
            self._t0 = state.t_wall
        finished = False
        if self._duration is not None and self._t0 is not None:
            if (state.t_wall - self._t0) >= self._duration:
                finished = True

        return MissionCommand(
            p_des_ned=self._p_des.copy(),
            v_des_ned=np.zeros(3),
            roll_des=self._roll_des,
            pitch_des=self._pitch_des,
            yaw_des=self._yaw_des,
            omega_des_body=np.zeros(3),
            finished=finished,
        )
