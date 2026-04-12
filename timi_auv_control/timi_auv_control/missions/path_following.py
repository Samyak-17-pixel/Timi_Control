"""3D path following with splines and trapezoidal speed along arc length."""

from __future__ import annotations

import math
from typing import Any, Dict, Optional

import numpy as np
from scipy.interpolate import splprep, splev

from .base import MissionBase, MissionCommand, VehicleState


def _cumlen(points: np.ndarray) -> np.ndarray:
    if len(points) < 2:
        return np.zeros(len(points))
    seg = np.linalg.norm(np.diff(points, axis=0), axis=1)
    return np.concatenate([[0.0], np.cumsum(seg)])


class PathFollowingMission(MissionBase):
    """
    YAML:
      waypoints_ned: list of [n,e,d]
      trapezoid: { accel_m_s2, max_speed_m_s, decel_m_s2 }
      total_time_s: optional — uniform motion in s-parameter [0, S_max]
      yaw_mode: tangent_h | fixed | hold_initial
      yaw_fixed_deg: optional override
    """

    def __init__(self, cfg: Dict[str, Any]):
        super().__init__(cfg)
        pts = np.array(cfg["waypoints_ned"], dtype=float)
        if pts.shape[0] < 2:
            raise ValueError("path_following needs at least two waypoints")
        self._pts = pts
        self._s_tab = _cumlen(pts)
        self._s_max = float(self._s_tab[-1])
        if self._s_max < 1e-6:
            raise ValueError("path waypoints must not be degenerate (zero length)")
        n = pts.shape[0]

        if n == 2:
            self._mode = "linear"
            self._seg_vec = pts[1] - pts[0]
            sl = float(np.linalg.norm(self._seg_vec))
            self._u_hat = self._seg_vec / sl if sl > 1e-9 else np.array([1.0, 0.0, 0.0])
        elif n >= 3:
            self._mode = "spline"
            k = min(3, n - 1)
            self._tck, _ = splprep(
                [pts[:, 0], pts[:, 1], pts[:, 2]], s=0, k=k, per=False
            )
        else:
            raise ValueError("invalid waypoint count")

        trap = cfg.get("trapezoid", {})
        self._a_acc = float(trap.get("accel_m_s2", 0.05))
        self._v_max = float(trap.get("max_speed_m_s", 0.35))
        self._a_dec = float(trap.get("decel_m_s2", self._a_acc))

        self._total_time = cfg.get("total_time_s", None)
        if self._total_time is not None:
            self._total_time = float(self._total_time)

        self._yaw_mode = cfg.get("yaw_mode", "tangent_h")
        self._yaw_fixed = math.radians(float(cfg.get("yaw_fixed_deg", 0.0)))
        self._yaw_hold: Optional[float] = None

        self._t0: Optional[float] = None
        self._s_cmd = 0.0
        self._v_along = 0.0
        self._trap_phase = "acc"

    def reset(self) -> None:
        self._t0 = None
        self._s_cmd = 0.0
        self._v_along = 0.0
        self._trap_phase = "acc"
        self._yaw_hold = None

    def _eval_pos(self, s: float) -> np.ndarray:
        s = float(np.clip(s, 0.0, self._s_max))
        if self._mode == "linear":
            if self._s_max < 1e-9:
                return self._pts[0].copy()
            t = s / self._s_max
            return (1 - t) * self._pts[0] + t * self._pts[1]
        u = s / self._s_max if self._s_max > 1e-9 else 0.0
        x, y, z = splev(u, self._tck)
        return np.array([float(x), float(y), float(z)], dtype=float)

    def _eval_tangent(self, s: float) -> np.ndarray:
        s = float(np.clip(s, 0.0, self._s_max))
        if self._mode == "linear":
            t = np.linalg.norm(self._u_hat)
            return self._u_hat / t if t > 1e-9 else np.array([1.0, 0.0, 0.0])
        u = s / self._s_max if self._s_max > 1e-9 else 0.0
        dx, dy, dz = splev(u, self._tck, der=1)
        d = np.array([float(dx), float(dy), float(dz)], dtype=float)
        # chain rule ds/du
        du_ds = 1.0 / self._s_max if self._s_max > 1e-9 else 0.0
        d = d * du_ds
        n = np.linalg.norm(d)
        if n < 1e-9:
            return np.array([1.0, 0.0, 0.0])
        return d / n

    def step(self, state: VehicleState, dt: float) -> MissionCommand:
        dt = max(1e-4, float(dt))
        if self._t0 is None:
            self._t0 = state.t_wall
            if self._yaw_mode == "hold_initial":
                from ..geometry import roll_pitch_yaw_from_quat

                qx, qy, qz, qw = state.q
                _, _, y0 = roll_pitch_yaw_from_quat(qx, qy, qz, qw)
                self._yaw_hold = y0

        finished = False
        v_along = 0.0

        if self._total_time is not None and self._total_time > 1e-6:
            t = state.t_wall - self._t0
            tau = min(1.0, max(0.0, t / self._total_time))
            self._s_cmd = tau * self._s_max
            v_along = self._s_max / self._total_time
            finished = tau >= 1.0 - 1e-6
        else:
            # Trapezoidal speed along path
            s_rem = self._s_max - self._s_cmd
            v = self._v_along
            if self._trap_phase == "acc":
                v = min(self._v_max, v + self._a_acc * dt)
                if v >= self._v_max - 1e-6:
                    self._trap_phase = "cruise"
            elif self._trap_phase == "cruise":
                v = self._v_max
                stop_d = (v * v) / (2 * max(self._a_dec, 1e-6))
                if s_rem <= stop_d:
                    self._trap_phase = "dec"
            else:
                v = max(0.0, v - self._a_dec * dt)
            self._v_along = v
            v_along = v
            self._s_cmd = min(self._s_max, self._s_cmd + v * dt)
            finished = self._s_cmd >= self._s_max - 1e-3

        p_des = self._eval_pos(self._s_cmd)
        tang = self._eval_tangent(self._s_cmd)
        v_ned_ff = tang * v_along

        yaw_des = self._yaw_from_mode(tang, state)

        return MissionCommand(
            p_des_ned=p_des,
            v_des_ned=v_ned_ff,
            roll_des=0.0,
            pitch_des=0.0,
            yaw_des=yaw_des,
            omega_des_body=np.zeros(3),
            finished=finished,
        )

    def _yaw_from_mode(self, tang: np.ndarray, state: VehicleState) -> float:
        if self._yaw_mode == "fixed":
            return self._yaw_fixed
        if self._yaw_mode == "hold_initial" and self._yaw_hold is not None:
            return self._yaw_hold
        tx, ty = float(tang[0]), float(tang[1])
        return math.atan2(ty, tx)
