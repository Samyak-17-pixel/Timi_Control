"""Mission base types."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple

import numpy as np


@dataclass
class VehicleState:
    """Snapshot from odometry + time."""

    t_wall: float
    p_ned: np.ndarray  # (3,)
    v_body: np.ndarray  # (3,)
    omega_body: np.ndarray  # (3,)
    q: Tuple[float, float, float, float]  # x,y,z,w


@dataclass
class MissionCommand:
    """Desired references for the wrench controller."""

    p_des_ned: np.ndarray
    v_des_ned: Optional[np.ndarray]  # optional feedforward in NED
    roll_des: float
    pitch_des: float
    yaw_des: float
    omega_des_body: Optional[np.ndarray]
    finished: bool


class MissionBase:
    """Override reset() and step()."""

    def __init__(self, cfg: Dict[str, Any]):
        self.cfg = cfg

    def reset(self) -> None:
        pass

    def step(self, state: VehicleState, dt: float) -> MissionCommand:
        raise NotImplementedError
