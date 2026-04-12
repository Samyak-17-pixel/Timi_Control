"""Load thruster positions and body-frame thrust directions from YAML."""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List

import numpy as np
import yaml


@dataclass
class ThrusterSpec:
    """Single thruster: position (m) and unit direction of force on vehicle (body frame)."""

    name: str
    position_body: np.ndarray  # (3,)
    direction_body: np.ndarray  # (3,) unit vector


def _unit(v: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(v)
    if n < 1e-9:
        raise ValueError("Zero direction vector")
    return v / n


def load_geometry(path: str | Path) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def build_thruster_list(geo: Dict[str, Any]) -> List[ThrusterSpec]:
    """Build ordered thruster list matching `order` in YAML."""
    order: List[str] = geo["thruster_order"]
    thrusters: Dict[str, Any] = geo["thrusters"]
    out: List[ThrusterSpec] = []
    for name in order:
        t = thrusters[name]
        pos = np.array(t["position_body_m"], dtype=float)
        d = np.array(t["direction_body"], dtype=float)
        out.append(ThrusterSpec(name=name, position_body=pos, direction_body=_unit(d)))
    return out


def build_allocation_matrix(thrusters: List[ThrusterSpec]) -> np.ndarray:
    """
    Build 6 x n_thrusters matrix B such that wrench_body = B @ f,
    where f_i is thrust magnitude (N) along direction_body for thruster i.

    wrench = [Fx, Fy, Fz, Mx, My, Mz]^T in body frame (Z down NED).
    Moment about COG: tau = r x F, with F = f_i * direction_i.
    """
    cols = []
    for t in thrusters:
        F = t.direction_body
        r = t.position_body
        tau = np.cross(r, F)
        cols.append(np.concatenate([F, tau]))
    return np.column_stack(cols)


def yaw_from_quaternion_ned(qx: float, qy: float, qz: float, qw: float) -> float:
    """Yaw (psi) about +Z (down), radians, from quaternion (map/body convention)."""
    # Assuming standard conversion: body relative to NED
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def roll_pitch_yaw_from_quat(qx: float, qy: float, qz: float, qw: float) -> tuple:
    """Roll, pitch, yaw (radians), ZYX intrinsic / typical aerospace."""
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw
