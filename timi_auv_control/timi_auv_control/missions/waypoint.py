"""Waypoint mission: pass through intermediates, stop at last (3D)."""

from __future__ import annotations

import math
from enum import Enum
from typing import Any, Dict, List

import numpy as np

from .base import MissionBase, MissionCommand, VehicleState


class _Phase(Enum):
    TRACK = 1
    DONE = 2


class WaypointMission(MissionBase):
    """
    YAML:
      waypoints_ned: list of [n,e,d] (m)
      acceptance_radius_m: enter sphere to switch to next (last: stop inside)
      cruise_speed_m_s: desired horizontal speed scale (NED velocity cap from controller)
      yaw_mode: "path" | "fixed"
      fixed_yaw_deg: used if yaw_mode is fixed
    """

    def __init__(self, cfg: Dict[str, Any]):
        super().__init__(cfg)
        wps = cfg["waypoints_ned"]
        self._wps = [np.array(p, dtype=float) for p in wps]
        if len(self._wps) < 1:
            raise ValueError("waypoints_ned needs at least one point")
        self._radius = float(cfg.get("acceptance_radius_m", 0.5))
        self._cruise = float(cfg.get("cruise_speed_m_s", 0.3))
        self._yaw_mode = cfg.get("yaw_mode", "path")
        self._fixed_yaw = math.radians(float(cfg.get("fixed_yaw_deg", 0.0)))
        self._idx = 0
        self._phase = _Phase.TRACK

    def reset(self) -> None:
        self._idx = 0
        self._phase = _Phase.TRACK

    def step(self, state: VehicleState, dt: float) -> MissionCommand:
        if self._phase == _Phase.DONE:
            last = self._wps[-1]
            return MissionCommand(
                p_des_ned=last.copy(),
                v_des_ned=np.zeros(3),
                roll_des=0.0,
                pitch_des=0.0,
                yaw_des=self._yaw_for(last - state.p_ned, state),
                omega_des_body=np.zeros(3),
                finished=True,
            )

        while True:
            target = self._wps[self._idx]
            err = target - state.p_ned
            dist = float(np.linalg.norm(err))
            last_wp = self._idx == len(self._wps) - 1
            if dist < self._radius:
                if last_wp:
                    self._phase = _Phase.DONE
                    return self.step(state, dt)
                self._idx += 1
                continue
            break

        d = float(np.linalg.norm(err))
        if d < 1e-6:
            v_ff = np.zeros(3)
        else:
            v_ff = (err / d) * min(self._cruise, d * 2.0)

        yaw_des = self._yaw_for(err, state)

        return MissionCommand(
            p_des_ned=target.copy(),
            v_des_ned=v_ff,
            roll_des=0.0,
            pitch_des=0.0,
            yaw_des=yaw_des,
            omega_des_body=np.zeros(3),
            finished=False,
        )

    def _yaw_for(self, err_ned: np.ndarray, state: VehicleState) -> float:
        if self._yaw_mode == "fixed":
            return self._fixed_yaw
        # Path tangent ~ direction to goal (horizontal)
        ex, ey = float(err_ned[0]), float(err_ned[1])
        return math.atan2(ey, ex)
