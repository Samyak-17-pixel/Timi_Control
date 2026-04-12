# Timi AUV Control (`timi_auv_control`)

ROS 2 package for **full 6-DOF** control of a **fully actuated** AUV with **eight Blue Robotics T200 thrusters** (four horizontal “vectored” thrusters at 45°, four vertical heave thrusters). The stack is designed to match your **Kalman-style filter** outputs:

- **Position** in **NED** (North, East, Down) in the odometry message frame.
- **Linear velocity** and **angular velocity** in the **body** frame (child frame of `nav_msgs/Odometry`).

This document is the **system-level** manual. Each mission type has its **own folder** under `config/missions/<mission_name>/` containing **`mission.yaml`** (parameters) and **`README.md`** (mission-only docs).

---

## Table of contents

1. [What this package does](#what-this-package-does)
2. [Architecture](#architecture)
3. [Frames and odometry contract](#frames-and-odometry-contract)
4. [Thruster model and geometry](#thruster-model-and-geometry)
5. [Control law (summary)](#control-law-summary)
6. [Configuration files](#configuration-files)
7. [ROS 2 interfaces](#ros-2-interfaces)
8. [Parameters](#parameters)
9. [Build and run](#build-and-run)
10. [Mission selection](#mission-selection)
11. [Tuning workflow](#tuning-workflow)
12. [Safety and limitations](#safety-and-limitations)
13. [Related documentation](#related-documentation)

---

## What this package does

- Subscribes to a **single** `nav_msgs/Odometry` topic (your fused filter output).
- Runs a **fixed-rate** control loop (default **50 Hz**, recommended range **50–100 Hz**).
- Computes a **desired** body-frame **wrench** (forces and moments) using:
  - **Outer loop:** NED position error → commanded NED velocity (limited).
  - **Inner loop:** body-frame velocity error → PID forces (with integrators).
  - **Attitude:** roll/pitch/yaw error + body angular rate damping → moments.
- Maps the wrench to **eight thrust commands** via a **6×8 allocation matrix** (`B`) and **Moore–Penrose pseudoinverse**, then clamps each thruster to a **force limit**.
- Converts each thrust (N) to **PWM microseconds** using **piecewise linear** maps for **forward** and **reverse** max thrust at **16 V**, with **PWM clipped to 1200–1800 µs** (as you requested).
- **Mother node** loads:
  - **Global tuning** (`config/control_params.yaml`).
  - **Thruster geometry** (`config/geometry.yaml`).
  - **One mission file** (`config/missions/<mission_name>/mission.yaml`) plus a **mission type** parameter.

**Not implemented** (by design, per your spec): sensor fault handling, added mass / quadratic damping in the model, explicit buoyancy torque (COB–COG unknown), bucket/magnet mission.

---

## Architecture

```
┌─────────────────┐     nav_msgs/Odometry      ┌──────────────────┐
│ Kalman / filter │ ────────────────────────► │   mother_node    │
└─────────────────┘                           │   (timi_auv_mother)│
                                              └────────┬─────────┘
                                                       │
                       ┌───────────────────────────────┼───────────────────────────────┐
                       │                               │                               │
                       ▼                               ▼                               ▼
              ┌────────────────┐              ┌─────────────────┐              ┌──────────────┐
              │ Mission plugin │              │ WrenchController │              │ B matrix +   │
              │ (station /     │  references  │ (PD/PID loops)   │  wrench      │ pinv + clamp │
              │  waypoint /    │─────────────►│                  │─────────────►│              │
              │  path)         │              │                  │              │              │
              └────────────────┘              └─────────────────┘              └──────┬───────┘
                                                                                        │
                                                                                        ▼
                                                                               ┌────────────────┐
                                                                               │ PWM map (T200) │
                                                                               └────────┬───────┘
                                                                                        │
                                                                                        ▼
                                                                               std_msgs/Int32MultiArray
                                                                               (8 channels, µs)
```

**Mission plugins** (Python modules):

| Mission type        | Module                    | Purpose |
|---------------------|---------------------------|---------|
| Station / hover     | `missions/station_keeping.py` | Hold NED pose + attitude for a duration or indefinitely |
| Waypoint            | `missions/waypoint.py`    | Pass through 3D points; stop at last |
| Path following      | `missions/path_following.py` | 3D spline path + trapezoidal speed profile |

---

## Frames and odometry contract

### NED

- **North** = X, **East** = Y, **Down** = Z (depth increases with positive Z).
- **Roll** about +X, **pitch** about +Y, **yaw** about +Z (right-hand rule in NED).

### Required `nav_msgs/Odometry` usage

- **`pose.pose.position`:** interpreted as **NED** position in the message’s **frame_id** (usually `map` or `odom`).
- **`pose.pose.orientation`:** quaternion giving **body relative to NED** (standard ROS convention: `geometry_msgs/Pose` in `map` frame).
- **`twist.twist.linear`:** **body-frame** linear velocity (m/s) — must match **`child_frame_id`** (your body frame).
- **`twist.twist.angular`:** body-frame angular velocity (rad/s).

The controller **does not** perform TF lookups; it trusts the message contents. Ensure your published quaternion matches the same convention used by your estimator.

---

## Thruster model and geometry

### Naming and PWM order

The published `Int32MultiArray` is ordered **exactly** as `thruster_order` in `config/geometry.yaml`:

`stfr`, `stbk`, `psfr`, `psbk`, `hstfr`, `hstbk`, `hpsfr`, `hpsbk`

### Thruster positions

Taken from your **engineering drawing** (converted to meters, origin at vehicle center):

- Horizontal corners: **±0.450 m** in X, **±0.22525 m** in Y.
- Heave cluster: **±0.142 m** in X, **±0.168 m** in Y.

### Horizontal thrust directions (body frame)

Four horizontal thrusters are modeled with **45°** horizontal components so that **equal positive thrust on all four** produces **positive surge** and **zero sway** (symmetric layout). Exact **unit vectors** are in `geometry.yaml`. If you **bench-test** and find surge or yaw coupling wrong, **flip signs** per thruster by negating that row’s `direction_body` in YAML (see README in package for hydrodynamic sign conventions).

### Heave thrust directions

Thrusters point **down** (+Z). Positive PWM is mapped so that **positive allocated force** along the configured direction **(0,0,-1)** produces **upward** motion (negative Z in NED body).

### Blue Robotics T200 @ 16 V

Default max values in `control_params.yaml` (typical published values):

- **Forward:** ~51.5 N  
- **Reverse:** ~40.2 N  

Update these if you use a different voltage curve or measured thrust.

---

## Control law (summary)

1. **Position (NED):**  
   `v_ned_cmd = Kp ⊙ (p_des − p_meas) + v_ff_ned`  
   then **saturate** `‖v_ned_cmd‖` to `vel_ned_max`.

2. **Map to body:**  
   `v_des_body = R_bn · v_ned_cmd` where `R_nb` is body→NED from quaternion.

3. **Velocity PID (body):**  
   Per-axis PID on `v_des_body − v_meas_body` → **body forces** `Fx, Fy, Fz`.

4. **Attitude:**  
   Small-angle errors on roll/pitch/yaw + PD on angular rates → **moments** `Mx, My, Mz`.

5. **Wrench:**  
   `τ = [F; M]` (6×1), then `f = pinv(B) * τ`, **clamp** per-thruster force, then **PWM**.

**Inertias** in `control_params.yaml` are **placeholders**; angular gains dominate real behavior until you identify `Ixx, Iyy, Izz`.

---

## Configuration layout

**Shared (not mission-specific):**

| File | Role |
|------|------|
| `config/control_params.yaml` | **Global** gains, limits, T200 thrust/PWM limits |
| `config/geometry.yaml` | Thruster positions, directions, `thruster_order` |

**Per-mission folders** (`config/missions/<name>/`):

| Folder | Contents |
|--------|----------|
| `station_keeping/` | `mission.yaml`, `README.md` |
| `waypoint/` | `mission.yaml`, `README.md` |
| `path_following/` | `mission.yaml`, `README.md` |

See also `config/missions/README.md` for an index of these folders. Future missions (e.g. bucket pickup) can add `config/missions/<new_mission>/` the same way.

---

## ROS 2 interfaces

### Subscriptions

| Topic (default) | Type | Description |
|-----------------|------|-------------|
| `/odometry/filtered` | `nav_msgs/Odometry` | Fused state (NED pose, body twist) |

### Publications

| Topic (default) | Type | Description |
|-----------------|------|-------------|
| `/auv/thrusters/pwm` | `std_msgs/Int32MultiArray` | Eight PWM values in **microseconds** |

### Parameters

| Parameter | Type | Description |
|-----------|------|-------------|
| `control_config` | string | **Absolute** path to `control_params.yaml` |
| `geometry_config` | string | **Absolute** path to `geometry.yaml` |
| `mission_config` | string | **Absolute** path to mission YAML |
| `mission_type` | string | `station_keeping`, `waypoint`, or `path_following` |
| `odom_topic` | string | Odometry subscription |
| `pwm_topic` | string | PWM publication |
| `control_rate_hz` | double | Control loop rate (default **50 Hz**) |

---

## Build and run

### Dependencies

- ROS 2 (Humble / Iron / Jazzy or compatible)
- Python 3: `numpy`, `scipy`, `PyYAML`

### Build

Use your ROS 2 distro name (examples: `humble`, `jazzy`, `iron`):

```bash
cd ~/your_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select timi_auv_control
source install/setup.bash
```

### Launch (examples)

**Station keeping** (default mission file):

```bash
ros2 launch timi_auv_control auv_control.launch.py
```

**Waypoint mission:**

```bash
ros2 launch timi_auv_control auv_control.launch.py \
  mission_type:=waypoint \
  mission_file:=$(ros2 pkg prefix timi_auv_control)/share/timi_auv_control/config/missions/waypoint/mission.yaml
```

**Path following:**

```bash
ros2 launch timi_auv_control auv_control.launch.py \
  mission_type:=path_following \
  mission_file:=$(ros2 pkg prefix timi_auv_control)/share/timi_auv_control/config/missions/path_following/mission.yaml
```

**Custom mission file** (recommended for development — keep a copy of a mission folder or point to your own `mission.yaml`):

```bash
ros2 launch timi_auv_control auv_control.launch.py \
  mission_type:=station_keeping \
  mission_file:=/home/you/missions/pool_test/mission.yaml
```

You can also pass the same parameters when running the node directly:

```bash
ros2 run timi_auv_control mother_node --ros-args \
  -p control_config:=/path/to/control_params.yaml \
  -p geometry_config:=/path/to/geometry.yaml \
  -p mission_config:=/path/to/mission.yaml \
  -p mission_type:=station_keeping
```

---

## Mission selection

| `mission_type` value | Parameters + docs |
|------------------------|-------------------|
| `station_keeping`, `station`, `hover` | `config/missions/station_keeping/` |
| `waypoint`, `waypoints` | `config/missions/waypoint/` |
| `path_following`, `path`, `spline` | `config/missions/path_following/` |

Each folder contains **`mission.yaml`** and **`README.md`** for that mission only.

---

## Tuning workflow

1. **Verify odometry** on a plot: NED position, body velocity, attitude — no jumps or wrong signs.
2. **PWM disconnected / motors off:** run node and confirm PWM stays near **1500 µs** when references match state (or zero errors).
3. **Depth / vertical:** tune `position_loop.kp[2]` and `velocity_loop` Z gains first (pressure depth).
4. **Horizontal:** tune `kp` for NED position, then velocity PID gains.
5. **Attitude:** tune `attitude_loop` — start conservative to avoid oscillation.
6. **Allocation saturation:** if `moment_max` or `force_max` is hit often, increase limits **only** if mechanically safe.

---

## Safety and limitations

- **Mission finished** or **no odometry** → node publishes **neutral PWM** (1500 µs) on all channels.
- This is **not** a safety-certified controller. Always use **hardware estop**, **tether**, and **pool testing** before open water.
- **Pseudoinverse allocation** does not guarantee constraint satisfaction beyond simple clamping; heavy saturation causes **tracking error**.
- **No model-based hydrodynamics** — expect to tune gains in water.

---

## Related documentation

| Location | Content |
|----------|---------|
| `config/missions/station_keeping/README.md` | Station holding, duration, YAML fields |
| `config/missions/waypoint/README.md` | Pass-through waypoints, acceptance radius |
| `config/missions/path_following/README.md` | Splines, trapezoidal speed, yaw modes |

---

## License

Apache-2.0 (see `package.xml`).
