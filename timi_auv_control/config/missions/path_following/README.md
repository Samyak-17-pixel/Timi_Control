# Path following (3D splines + trapezoidal speed)

Everything for this mission lives **in this folder**:

| File | Purpose |
|------|---------|
| **`mission.yaml`** | Mission parameters (waypoints, trapezoid, yaw) — edit this for runs |
| **`README.md`** | This documentation |

The Python plugin is at `timi_auv_control/missions/path_following.py`.

---

The mother node loads this mission when `mission_type` is:

- `path_following`
- `path`
- `spline`

---

## Purpose

Use path following when you want the AUV to track a **smooth 3D curve** through **multiple** NED points (e.g. lawn-mower patterns, smooth approaches, cinematic trajectories) rather than **discrete** jumps between waypoints.

The implementation uses **B-spline** interpolation via SciPy’s `splprep` / `splev` when **three or more** waypoints are given. With **exactly two** waypoints, the path **degenerates** to a **straight line segment** (linear interpolation).

---

## Behavior (what the code does)

### Path parameter

- The curve is parameterized by **arc length** `s ∈ [0, S_max]`, where `S_max` is the total length of the **control polyline** (chord-length cumulation of the input waypoints — used only to define the spline domain in the linear two-point case; for splines, `splprep` uses its internal parameter `u ∈ [0,1]` mapped to `s`).

### Motion in time (two modes)

#### Mode A — `total_time_s` **set** (not `null`)

- The mission maps **wall-clock** time `t` to normalized progress `τ = t / total_time_s` (clamped to `[0,1]`).
- **Arc position:** `s_cmd = τ * S_max`.
- **Along-path speed magnitude:** approximately **constant** `S_max / total_time_s` (used as feedforward velocity along the **tangent**).

Use this when you need a **predictable mission duration** (e.g. timed run in a competition).

#### Mode B — `total_time_s` **`null`** (default)

- A **trapezoidal** speed profile along the path:
  - **Acceleration** phase: increase along-path speed up to `max_speed_m_s` with `accel_m_s2`.
  - **Cruise:** hold max speed until **deceleration** must begin so the vehicle can stop at `S_max` using `decel_m_s2`.
  - **Deceleration:** reduce speed to zero at the end.

This is the **recommended** default for smooth, intuitive speed control (see package `README.md`).

### Feedforward velocity

At each step, the mission sets:

- `p_des_ned` = position on the curve at `s_cmd`.
- `v_des_ned` = **tangent direction** × **current scalar along-path speed**.

The wrench controller adds **position feedback** on top of this feedforward.

### Completion

When `s_cmd` reaches **end of path** (within a small epsilon), `finished` becomes **true** and the mother node publishes **neutral PWM**.

---

## Yaw modes (`yaw_mode`)

| Value | Behavior |
|-------|----------|
| `tangent_h` (default) | Yaw = `atan2(tangent_east, tangent_north)` using the **horizontal** projection of the path tangent (depth changes do not twist heading by default). |
| `fixed` | Yaw = `yaw_fixed_deg` (constant). |
| `hold_initial` | On first valid odometry, **snapshot** current yaw and hold it for the whole path. |

Roll and pitch references are **0**.

---

## YAML schema (reference)

Edit **`mission.yaml`** in this folder:

```yaml
waypoints_ned:
  - [0.0, 0.0, 5.0]
  - [1.0, 0.5, 5.2]
  - [2.0, 1.0, 5.5]
  - [3.0, 0.0, 5.0]

trapezoid:
  accel_m_s2: 0.06
  max_speed_m_s: 0.35
  decel_m_s2: 0.06

total_time_s: null

yaw_mode: tangent_h
yaw_fixed_deg: 0.0
```

### Field details

#### `waypoints_ned` (required)

- **Minimum:** **2** points.
- Points are **control points** for the spline (not necessarily visited exactly — the curve **approximates** a smooth path through them).

#### `trapezoid` (optional)

- **`accel_m_s2`:** Along-path acceleration during ramp-up.
- **`max_speed_m_s`:** Cruise speed cap along path.
- **`decel_m_s2`:** Deceleration magnitude.

If values are too aggressive, the **global** velocity and force limits will **clip** output; reduce these or tune global limits.

#### `total_time_s` (optional)

- **`null` or omitted:** use **trapezoidal** profile (Mode B).
- **Positive float:** uniform-time mapping (Mode A).

#### `yaw_mode` / `yaw_fixed_deg`

- See table above. For **`fixed`**, set **`yaw_fixed_deg`** to the desired **heading** in degrees.

---

## Spline degree and few waypoints

- **Two points:** straight line — no B-spline ambiguity.
- **Three or more:** `splprep` uses `k = min(3, n-1)` so **short** paths still get a valid spline (may be quadratic if only three points).

---

## Interaction with global tuning

- **Tracking error:** reduce `vel_ned_max` and/or increase position gains if the vehicle **cuts** the curve.
- **Oscillation:** reduce feedforward speed (`max_speed_m_s` or longer `total_time_s`) or damp velocity PID.

---

## Launch example

```bash
ros2 launch timi_auv_control auv_control.launch.py \
  mission_type:=path_following \
  mission_file:=$(ros2 pkg prefix timi_auv_control)/share/timi_auv_control/config/missions/path_following/mission.yaml
```

---

## Troubleshooting

| Symptom | Likely cause | Suggestion |
|---------|--------------|------------|
| Path bulges outside waypoints | Spline nature | Add more control points or use waypoint mission |
| Finishes too fast / slow | `total_time_s` or trapezoid limits | Adjust `total_time_s` or `max_speed_m_s` |
| Heading hunts on vertical legs | `tangent_h` with mostly vertical motion | Use `hold_initial` or `fixed` |
| Large cross-track error | Speed too high for gains | Slow down or tune loops |
| Mission never finishes | Numerical end detection | Check `S_max` and logs; verify odometry time advances |
