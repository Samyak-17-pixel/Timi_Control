"""
Qualification gate: HSV red boost + poles + center + PnP.

  python3 qualification_gate_detector.py --folder images/image_qualification_01
"""

from __future__ import annotations

import argparse
import os
import sys

import cv2

from gate_pipeline import PipelineConfig, process_frame
from gate_temporal import GateTemporalFilter


def main() -> None:
    p = argparse.ArgumentParser(description="Qualification gate detector")
    p.add_argument("--folder", default="images/image_qualification_01")
    p.add_argument("--delay", type=int, default=500)
    p.add_argument("--no-gui", action="store_true")
    p.add_argument("--fov", type=float, default=60.0)
    p.add_argument("--gate-width", type=float, default=1.5)
    p.add_argument("--no-pnp", action="store_true")
    p.add_argument("--temporal", type=float, default=0.0)
    args = p.parse_args()

    root = os.path.dirname(os.path.abspath(__file__))
    folder = args.folder
    if not os.path.isabs(folder):
        folder = os.path.join(root, folder)
    if not os.path.isdir(folder):
        print("Folder not found:", folder, file=sys.stderr)
        sys.exit(1)

    cfg = PipelineConfig(
        use_color_boost=True,
        horizontal_fov_deg=args.fov,
        gate_width_m=args.gate_width,
        use_pnp=not args.no_pnp,
        temporal_alpha=args.temporal,
    )
    temporal = (
        GateTemporalFilter(cfg.temporal_alpha, cfg.temporal_max_jump_frac)
        if cfg.temporal_alpha > 0
        else None
    )

    images = sorted(f for f in os.listdir(folder) if f.endswith(".png"))
    n_two = n_one = n_none = 0
    n_pnp = 0

    if not args.no_gui:
        cv2.namedWindow("Gate Detection", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Gate Detection", min(1600, 1200), min(900, 800))
        cv2.namedWindow("Edges", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Edges", 640, 360)

    for img_name in images:
        path = os.path.join(folder, img_name)
        frame = cv2.imread(path)
        if frame is None:
            continue

        st, pose, disp = process_frame(frame, cfg, temporal)
        if st.state == "two":
            n_two += 1
            if pose and pose.ok:
                n_pnp += 1
        elif st.state == "one":
            n_one += 1
        else:
            n_none += 1

        if not args.no_gui:
            if st.filtered_edges is not None:
                cv2.imshow("Edges", cv2.resize(st.filtered_edges, (640, 360)))
            cv2.imshow(
                "Gate Detection",
                cv2.resize(disp, (min(1600, disp.shape[1]), min(900, disp.shape[0]))),
            )
            if cv2.waitKey(args.delay) == 27:
                break

    if not args.no_gui:
        cv2.destroyAllWindows()

    n = n_two + n_one + n_none
    print("\nDetection Summary")
    print("---------------------------")
    print("Total Images:", n)
    print("Two poles:", n_two, "| One pole:", n_one, "| None:", n_none)
    if n_two > 0:
        print("PnP solves (raw ok):", n_pnp, "/", n_two)
    if n > 0:
        print("Two-pole rate:", round(100 * n_two / n, 2), "%")


if __name__ == "__main__":
    main()
