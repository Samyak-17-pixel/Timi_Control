# Waypoint tracking (pass-through, stop at last)

Everything for this mission lives **in this folder**:

| File | Purpose |
|------|---------|
| **`mission.yaml`** | Mission parameters (waypoints, radius, speeds) — edit this for runs |
| **`README.md`** | This documentation |

The Python plugin is at `timi_auv_control/missions/waypoint.py`.

---

The mother node loads this mission when `mission_type` is:

- `waypoint`
- `waypoints`

---

## Purpose

Use this mission when you have an **ordered list of 3D points** in NED and you want the vehicle to:

1. **Visit** each waypoint in order.
2. **Pass through** every **intermediate** waypoint (no full stop at each unless you place them far apart and tune acceptance).
3. **Stop** when it enters an **acceptance sphere** around the **last** waypoint.

This is **not** smooth path interpolation between points (that is **path following** with splines). Here, the **active setpoint** jumps to the next waypoint when the vehicle is **inside** the acceptance radius of the current one.

---

## Behavior (what the code does)

### State machine (conceptual)

1. **TRACK:** Current goal is `waypoints_ned[index]`.
2. When **distance** from `p_meas` to current goal **<** `acceptance_radius_m`:
   - If this is **not** the last waypoint → **increment index** and continue tracking the **next** goal.
   - If this **is** the last waypoint → transition to **DONE**.
3. **DONE:** Mission reports `finished = true`. The mother node then publishes **neutral PWM** (same as other missions on completion).

### Velocity feedforward

While tracking, the mission sets:

- `p_des_ned` = current waypoint position.
- `v_des_ned` = **unit vector** from current pose toward the waypoint × **min**( `cruise_speed_m_s`, `k * distance` ) with a simple cap (implementation uses distance-based scaling so speed reduces near the point).

### Yaw (`yaw_mode`)

| `yaw_mode` | Meaning |
|------------|---------|
| `path` (default) | Yaw aligns with the **horizontal** direction to the **current** waypoint: `atan2(east_error, north_error)`. |
| `fixed` | Yaw is **`fixed_yaw_deg`** (converted to radians) for the whole mission. |

Roll and pitch references are **0** (level flight).

---

## YAML schema (reference)

Edit **`mission.yaml`** in this folder:

```yaml
waypoints_ned:
  - [0.0, 0.0, 5.0]
  - [2.0, 0.0, 5.0]
  - [2.0, 2.0, 6.0]

acceptance_radius_m: 0.6
cruise_speed_m_s: 0.3
yaw_mode: path
fixed_yaw_deg: 0.0
```

### Field details

#### `waypoints_ned` (required)

- **Minimum length:** 1.
- If **one** waypoint only, behavior reduces to “go to that point” until inside the radius, then **DONE** (same as a single-point visit).

#### `acceptance_radius_m` (optional)

- **Default:** `0.5` if omitted (see `waypoint.py`).
- **Too small:** vehicle may **orbit** or **never** satisfy distance (sensor noise).
- **Too large:** may **skip** corners of a zig-zag path.

#### `cruise_speed_m_s` (optional)

- Scales how large **feedforward** NED velocity is. The **global** `vel_ned_max` in `control_params.yaml` still **caps** the outer loop.

#### `yaw_mode` / `fixed_yaw_deg`

- Use **`path`** for survey lines where the nose should point along the leg.
- Use **`fixed`** for **sideways** or **constant-heading** runs (subject to vehicle capability).

---

## Interaction with global tuning

- **Horizontal tracking:** depends on `position_loop.kp`, `velocity_loop`, and `vel_ned_max`.
- **Turning between legs:** large heading changes may require **lower** `cruise_speed_m_s` or **gentler** position gains to avoid saturation.

---

## Differences from path following

| Aspect | Waypoint mission | Path following (splines) |
|--------|------------------|---------------------------|
| Geometry | Straight **segments** implied by jumping setpoints | **Smooth** 3D curve through all points |
| Setpoint | Current waypoint only | Continuous position on spline |
| Typical use | Quick mission scripts, coarse coverage | Smooth paths |

If you need **curved** 3D trajectories, use **`path_following`** (`config/missions/path_following/`).

---

## Launch example

```bash
ros2 launch timi_auv_control auv_control.launch.py \
  mission_type:=waypoint \
  mission_file:=$(ros2 pkg prefix timi_auv_control)/share/timi_auv_control/config/missions/waypoint/mission.yaml
```

---

## Troubleshooting

| Symptom | Likely cause | Suggestion |
|---------|--------------|------------|
| Never reaches waypoint | Radius too small, or gains too weak | Increase `acceptance_radius_m`; tune gains |
| Overshoots each corner | Too fast / aggressive gains | Reduce `cruise_speed_m_s` or `vel_ned_max` |
| Zig-zag path cuts corners | Large radius | Reduce radius or add intermediate waypoints |
| Wrong heading | `yaw_mode` | Use `fixed` + `fixed_yaw_deg` or verify NED |
| Mission ends with drift | **DONE** publishes neutral | Expected — plan next mission or restart |
