# Station keeping / hover

Everything for this mission lives **in this folder**:

| File | Purpose |
|------|---------|
| **`mission.yaml`** | Mission parameters (pose, duration) — edit this for runs |
| **`README.md`** | This documentation |

The Python plugin is in the package at `timi_auv_control/missions/station_keeping.py` (shared runtime code, not duplicated here).

---

This mission is the **same** behavior as **hover** in your specification: the vehicle **holds** a commanded **NED position**, **depth** (via Down coordinate), and **full attitude** (roll, pitch, yaw).

The mother node (`mother_node.py`) loads this mission when the parameter `mission_type` is one of:

- `station_keeping`
- `station`
- `hover`

---

## Purpose

Use station keeping when you need the AUV to **remain** at a fixed pose in the world for:

- **Calibration** of sensors or thruster alignment in a tank.
- **Short-duration** holds at a science waypoint.
- **Stabilization** before switching to another mission (manual orchestration outside this package).

---

## Behavior (what the code does)

1. **Desired position** is fixed to `position_ned` from **`mission.yaml`** for the entire run (until the mission ends).
2. **Desired attitude** is fixed to `attitude_deg` (converted to radians internally).
3. **Feedforward velocity** in NED is **zero** (the mission passes a zero vector so the outer loop is **pure feedback** from position error).
4. **Feedforward angular rate** is zero.

### Time limit (`duration_s`)

| `duration_s` in YAML | Mission `finished` flag |
|----------------------|-------------------------|
| **Omitted** or **`null`** | **Never** becomes true based on time — the vehicle keeps holding until you stop the node or switch configuration. |
| **Positive number** (e.g. `120.0`) | Becomes **true** after that many **seconds** from the first valid odometry sample. |

When `finished` is **true**, the mother node **stops** closed-loop control and publishes **neutral PWM** (1500 µs) on all eight thrusters. **Important:** if you need the vehicle to **keep holding** after the timer, you must **restart** the node or use a new mission file; this package does not auto-restart missions.

---

## YAML schema (reference)

All keys are read from **`mission.yaml`** only (not from `control_params.yaml`).

```yaml
# Required: NED position [North_m, East_m, Down_m]
position_ned: [0.0, 0.0, 5.0]

# Optional: degrees. Default 0 if omitted.
attitude_deg:
  roll: 0.0
  pitch: 0.0
  yaw: 0.0

# Optional: seconds. Use null or omit for indefinite hold.
duration_s: null
```

### Field details

#### `position_ned` (required)

- **Type:** list of three floats `[north, east, down]` in **meters**.
- **North / East:** horizontal plane relative to your fixed NED origin (same as your Kalman map origin).
- **Down:** positive **down** — larger Z means **deeper** in standard NED. This matches using **pressure depth** as primary vertical reference in your stack; ensure your estimator’s Z axis is consistent.

#### `attitude_deg` (optional)

- **roll:** rotation about +X (NED body), degrees.
- **pitch:** rotation about +Y, degrees.
- **yaw:** rotation about +Z (heading), degrees.

The controller uses **small-angle** error between **desired** and **measured** Euler angles from the odometry quaternion. For large misalignment, tune attitude gains conservatively.

#### `duration_s` (optional)

- **`null` or omitted:** no time-based completion.
- **Positive float:** mission completes after **wall-clock** time (ROS clock) from first successful control cycle, not from mission start in absolute GPS time.

---

## Interaction with global tuning (`config/control_params.yaml`)

Station keeping does **not** add its own gains. All **Kp/Ki/Kd** and limits come from the **global** file at the package root:

- **Position loop** `kp` scales how aggressively the vehicle tries to return when displaced (NED).
- **Velocity loop** PID generates body forces from velocity error.
- **Attitude loop** generates moments from attitude and rate error.

If the vehicle **oscillates** at the hold point:

- Reduce `position_loop.kp` or increase damping via `velocity_loop.kd`.
- Reduce `attitude_loop.kp` first before raising limits.

If it **drifts** slowly:

- Increase `velocity_loop.ki` slightly (watch integral windup in saturation).
- Check odometry bias and thruster neutral calibration.

---

## Operational checklist

1. **Set** `position_ned` in **`mission.yaml`** to a pose that is **reachable** and **safe** (depth, obstacles).
2. **Verify** yaw/roll/pitch commands match your mission (often roll/pitch **0** for stable survey).
3. **Choose** `duration_s`:
   - **`null`** for open-ended tests with a **kill switch**.
   - **Finite** for timed station tests (remember: on completion, PWM goes **neutral**).
4. **Monitor** `/auv/thrusters/pwm` — values should stay **within 1200–1800 µs** after clipping.

---

## Launch example

Point `mission_file` at **`mission.yaml` in this folder** (after install, under `share/timi_auv_control/...`):

```bash
ros2 launch timi_auv_control auv_control.launch.py \
  mission_type:=station_keeping \
  mission_file:=$(ros2 pkg prefix timi_auv_control)/share/timi_auv_control/config/missions/station_keeping/mission.yaml
```

---

## Troubleshooting

| Symptom | Likely cause | Suggestion |
|---------|--------------|------------|
| Vehicle drifts in horizontal plane | Weak position/velocity gains or saturation | Increase gains within safe limits; check allocation saturation logs |
| Oscillation in depth | Aggressive Z gains or wrong depth sign | Verify NED Z sign vs pressure; reduce `kp[2]` |
| Wrong heading at hold | Yaw reference vs compass | Check quaternion convention; tune yaw `kp_att` |
| Mission ends immediately | `duration_s` too small or wrong type | Use `null` or large value |
| Thrusters idle after a few minutes | `duration_s` elapsed | Expected — restart node or mission file |

---

## Relation to other missions

- **Waypoint** and **path following** missions **move** the setpoint; station keeping keeps it **fixed**.
- The **bucket / magnet** hover mission you described earlier is **not** implemented here; when you add it, give it its own folder under `config/missions/` with `mission.yaml` + `README.md`.
