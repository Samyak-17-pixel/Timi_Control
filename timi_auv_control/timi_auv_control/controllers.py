"""PID / PD helpers and 6-DOF wrench computation."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Tuple

import numpy as np

from .geometry import roll_pitch_yaw_from_quat


def rot_body_to_ned_from_quat(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """Rotation matrix R_nb: v_ned = R_nb @ v_body."""
    # Normalize
    n = (qx * qx + qy * qy + qz * qz + qw * qw) ** 0.5
    if n < 1e-12:
        return np.eye(3)
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    # From quaternion to R (body to world/NED)
    r00 = 1 - 2 * (qy * qy + qz * qz)
    r01 = 2 * (qx * qy - qz * qw)
    r02 = 2 * (qx * qz + qy * qw)
    r10 = 2 * (qx * qy + qz * qw)
    r11 = 1 - 2 * (qx * qx + qz * qz)
    r12 = 2 * (qy * qz - qx * qw)
    r20 = 2 * (qx * qz - qy * qw)
    r21 = 2 * (qy * qz + qx * qw)
    r22 = 1 - 2 * (qx * qx + qy * qy)
    return np.array([[r00, r01, r02], [r10, r11, r12], [r20, r21, r22]], dtype=float)


def wrap_pi(a: float) -> float:
    return (a + np.pi) % (2 * np.pi) - np.pi


@dataclass
class PID1D:
    kp: float = 0.0
    ki: float = 0.0
    kd: float = 0.0
    int_min: float = -1e6
    int_max: float = 1e6
    out_min: float = -1e6
    out_max: float = 1e6
    _i: float = 0.0
    _prev_e: float = 0.0

    def reset(self) -> None:
        self._i = 0.0
        self._prev_e = 0.0

    def step(self, e: float, dt: float, de_dt: float | None = None) -> float:
        if dt <= 0:
            return 0.0
        self._i = np.clip(self._i + e * dt, self.int_min, self.int_max)
        d = de_dt if de_dt is not None else (e - self._prev_e) / dt
        self._prev_e = e
        u = self.kp * e + self.ki * self._i + self.kd * d
        return float(np.clip(u, self.out_min, self.out_max))


@dataclass
class WrenchController:
    """Translational + rotational PD with configurable gains from dict."""

    mass_kg: float = 40.0
    # Diagonal inertia guess (kg m^2) when unknown — tune via YAML
    Ixx: float = 2.0
    Iyy: float = 2.0
    Izz: float = 2.0

    # Position -> desired NED velocity
    kp_pos: np.ndarray = field(default_factory=lambda: np.ones(3) * 0.2)
    # Velocity body error -> force
    kp_vel: np.ndarray = field(default_factory=lambda: np.ones(3) * 80.0)
    ki_vel: np.ndarray = field(default_factory=lambda: np.ones(3) * 5.0)
    kd_vel: np.ndarray = field(default_factory=lambda: np.ones(3) * 20.0)

    kp_att: np.ndarray = field(default_factory=lambda: np.ones(3) * 40.0)
    kd_rate: np.ndarray = field(default_factory=lambda: np.ones(3) * 15.0)

    vel_ned_max: float = 0.5
    force_max: float = 200.0
    moment_max: float = 40.0

    _pid_vx: PID1D = field(default_factory=PID1D)
    _pid_vy: PID1D = field(default_factory=PID1D)
    _pid_vz: PID1D = field(default_factory=PID1D)

    def configure_from_yaml(self, cfg: Dict) -> None:
        c = cfg.get("vehicle", {})
        self.mass_kg = float(c.get("mass_kg", self.mass_kg))
        self.Ixx = float(c.get("Ixx", self.Ixx))
        self.Iyy = float(c.get("Iyy", self.Iyy))
        self.Izz = float(c.get("Izz", self.Izz))

        pos = cfg.get("position_loop", {})
        self.kp_pos = np.array(pos.get("kp", self.kp_pos.tolist()), dtype=float)
        self.vel_ned_max = float(pos.get("vel_ned_max", self.vel_ned_max))

        vloop = cfg.get("velocity_loop", {})
        self.kp_vel = np.array(vloop.get("kp", self.kp_vel.tolist()), dtype=float)
        self.ki_vel = np.array(vloop.get("ki", self.ki_vel.tolist()), dtype=float)
        self.kd_vel = np.array(vloop.get("kd", self.kd_vel.tolist()), dtype=float)
        lim = vloop.get("integral_limit", [50.0, 50.0, 50.0])
        ilim = np.array(lim, dtype=float)
        for pid, i in ((self._pid_vx, 0), (self._pid_vy, 1), (self._pid_vz, 2)):
            pid.ki = float(self.ki_vel[i])
            pid.int_min = float(-ilim[i])
            pid.int_max = float(ilim[i])
            pid.kp = float(self.kp_vel[i])
            pid.kd = float(self.kd_vel[i])
            pid.out_min = -abs(self.force_max)
            pid.out_max = abs(self.force_max)

        al = cfg.get("limits", {})
        self.force_max = float(al.get("force_max_n", self.force_max))
        self.moment_max = float(al.get("moment_max_nm", self.moment_max))

        att = cfg.get("attitude_loop", {})
        self.kp_att = np.array(att.get("kp", self.kp_att.tolist()), dtype=float)
        self.kd_rate = np.array(att.get("kd", self.kd_rate.tolist()), dtype=float)

    def reset_integrators(self) -> None:
        self._pid_vx.reset()
        self._pid_vy.reset()
        self._pid_vz.reset()

    def compute_wrench(
        self,
        dt: float,
        p_meas_ned: np.ndarray,
        v_meas_body: np.ndarray,
        omega_meas_body: np.ndarray,
        q: Tuple[float, float, float, float],
        p_des_ned: np.ndarray,
        v_des_ned: np.ndarray | None,
        roll_des: float,
        pitch_des: float,
        yaw_des: float,
        omega_des_body: np.ndarray | None = None,
    ) -> np.ndarray:
        """
        Returns wrench [Fx,Fy,Fz,Mx,My,Mz] in body frame.
        """
        qx, qy, qz, qw = q
        R_nb = rot_body_to_ned_from_quat(qx, qy, qz, qw)
        R_bn = R_nb.T

        e_p = p_des_ned - p_meas_ned
        v_ned_cmd = self.kp_pos * e_p
        if v_des_ned is not None:
            v_ned_cmd = v_ned_cmd + v_des_ned
        vn = np.linalg.norm(v_ned_cmd)
        if vn > self.vel_ned_max > 0:
            v_ned_cmd = v_ned_cmd * (self.vel_ned_max / vn)

        v_des_body = R_bn @ v_ned_cmd

        e_v = v_des_body - v_meas_body
        Fx = self._pid_vx.step(float(e_v[0]), dt, None)
        Fy = self._pid_vy.step(float(e_v[1]), dt, None)
        Fz = self._pid_vz.step(float(e_v[2]), dt, None)
        F = np.array([Fx, Fy, Fz], dtype=float)
        F = np.clip(F, -self.force_max, self.force_max)

        # Attitude: small-angle error in body (roll pitch yaw)
        roll, pitch, yaw = roll_pitch_yaw_from_quat(qx, qy, qz, qw)
        e_roll = wrap_pi(roll_des - roll)
        e_pitch = wrap_pi(pitch_des - pitch)
        e_yaw = wrap_pi(yaw_des - yaw)

        if omega_des_body is None:
            omega_des_body = np.zeros(3)

        Mx = self.kp_att[0] * e_roll + self.kd_rate[0] * (omega_des_body[0] - omega_meas_body[0])
        My = self.kp_att[1] * e_pitch + self.kd_rate[1] * (omega_des_body[1] - omega_meas_body[1])
        Mz = self.kp_att[2] * e_yaw + self.kd_rate[2] * (omega_des_body[2] - omega_meas_body[2])
        M = np.array([Mx, My, Mz], dtype=float)
        M = np.clip(M, -self.moment_max, self.moment_max)

        return np.concatenate([F, M])
