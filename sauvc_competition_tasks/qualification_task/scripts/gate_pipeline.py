"""Single-frame pipeline: state -> optional temporal -> PnP -> overlay metrics."""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Tuple

import cv2
import numpy as np

from gate_detection_core import GateStateResult, detect_gate_with_state
from gate_draw import draw_one_pole_searching, draw_status_corner, draw_two_poles_and_center
from gate_pose_pnp import GatePoseResult, camera_matrix_from_fov, estimate_gate_pose
from gate_temporal import GateTemporalFilter


@dataclass
class PipelineConfig:
    use_color_boost: bool = False
    horizontal_fov_deg: float = 60.0
    gate_width_m: float = 1.5
    use_pnp: bool = True
    temporal_alpha: float = 0.0
    """If >0, EMA smooth pole x before PnP/draw."""
    edge_ignore_frac: float = 0.04
    """Ignore this fraction of width at each side in the column histogram (webcam borders)."""
    temporal_max_jump_frac: float = 0.32
    """Reset EMA when a pole jumps by more than this fraction of frame width."""


def detect_horizontal_bars(frame: np.ndarray, left: int, right: int) -> List[int]:
    height = frame.shape[0]
    roi = frame[:, left:right]
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    sobely = cv2.Sobel(blur, cv2.CV_64F, 0, 1, ksize=3)
    sobely = np.uint8(np.absolute(sobely))
    edges = cv2.Canny(sobely, 30, 80)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (25, 3))
    edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
    row_strength = np.sum(edges, axis=1)
    if np.max(row_strength) == 0:
        return []
    row_strength = cv2.GaussianBlur(row_strength.reshape(-1, 1), (51, 1), 0).flatten()
    peaks = np.argsort(row_strength)[::-1]
    bars: List[int] = []
    min_sep = 40
    for idx in peaks:
        if len(bars) == 0:
            bars.append(idx)
        elif abs(idx - bars[0]) > min_sep:
            bars.append(idx)
            break
    return bars


def process_frame(
    frame_bgr: np.ndarray,
    cfg: PipelineConfig,
    temporal: Optional[GateTemporalFilter],
) -> Tuple[GateStateResult, Optional[GatePoseResult], np.ndarray]:
    """
    Returns (state_result, pose_or_none, display_frame BGR copy).
    """
    out = frame_bgr.copy()
    h, w = out.shape[:2]
    st = detect_gate_with_state(
        out,
        use_color_boost=cfg.use_color_boost,
        edge_ignore_frac=cfg.edge_ignore_frac,
    )

    if st.state == "none":
        draw_status_corner(out, "no gate", (120, 120, 255))
        return st, None, out

    if st.state == "one":
        draw_one_pole_searching(out, st.single_x)
        return st, None, out

    lx, rx = st.left_x, st.right_x
    if temporal is not None and cfg.temporal_alpha > 0:
        lx, rx = temporal.update_two(st.left_x, st.right_x, frame_width=w)

    center_err = 0.5 * (lx + rx) - (w / 2.0)
    li, ri = int(round(lx)), int(round(rx))
    bars = detect_horizontal_bars(out, max(0, li), min(w, ri))

    pose: Optional[GatePoseResult] = None
    if cfg.use_pnp and st.filtered_edges is not None:
        K = camera_matrix_from_fov(w, h, cfg.horizontal_fov_deg)
        pose = estimate_gate_pose(
            out,
            lx,
            rx,
            st.filtered_edges,
            st.roi_y0,
            K,
            width_m=cfg.gate_width_m,
            height_m=None,
        )

    pose_draw = (
        pose
        if pose and pose.ok and pose.reproj_err_px <= 25.0
        else None
    )

    draw_two_poles_and_center(
        out,
        lx,
        rx,
        0.5 * (lx + rx),
        st.center_y,
        rx - lx,
        center_err,
        st.skew_score,
        pose=pose_draw,
        bars_y=bars if bars else None,
    )
    return st, pose, out
