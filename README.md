# Timi_Control — complete workspace manual

This document explains **this entire folder from scratch**: what it is, how the two software trees relate, what every file is for, how data flows, how to build and run, and what is *not* in this repository.

It is written so someone who has never seen the project can understand it without opening other docs first. Package-level READMEs under `timi_auv_control/` still exist for mission-specific tuning; this file is the full map.

---

## 0. What this repository is

**Timi_Control** is the software workspace for the **Timi AUV** (autonomous underwater vehicle). It contains ROS 2 packages for:

1. **Core 6-DOF vehicle control** — fused odometry in, eight thruster PWM commands out.
2. **Competition / mission software** kept in a **separate** tree — depth hold, qualification **gate** perception and mission FSM, and **ball-dropping / drum** perception.

The two trees are **intentionally not mixed**. They can be built in the same ROS 2 workspace later, but they were developed as different stacks (different allocation, different command topics, different external packages).

| Tree | Path | Role |
|------|------|------|
| Core control | `timi_auv_control/` | Fully actuated 8× T200, NED pose + body twist → PWM |
| Competition tasks | `sauvc_competition_tasks/` | Depth PID, gate detection/mission, drum detection |

**Nothing in this workspace is a complete robot bring-up.** Cameras, DVL drivers, IMU drivers, and (for the competition stack) `auv_2d_control` / `auv_3d_control` live **outside** this folder.

---

## 1. Mental model (start here)

### 1.1 Core stack (`timi_auv_control`)

```
Kalman / filter  --nav_msgs/Odometry-->  mother_node
                                            |
                    mission plugin (station / waypoint / path)
                                            |
                    WrenchController (position + velocity PID + attitude PD)
                                            |
                    allocation B⁺  →  8 forces (N)  →  T200 PWM map
                                            |
                                            v
                              /auv/thrusters/pwm  (Int32MultiArray, µs)
```

One node does everything. There is **no** separate 2D controller, depth PID, or 3D allocator in this tree.

### 1.2 Competition stack (`sauvc_competition_tasks`)

This stack assumes an **older / parallel** architecture:

- Planar commands on `/control/cmd_vel` (`geometry_msgs/Twist`).
- Heave from a **depth PID** on `/control/heave_cmd`.
- Merge into `/control/cmd_vel_3d`.
- External `thruster_allocator_3d` (package **not** in this repo) turns 3D twist into thruster commands.

Qualification mission:

```
camera  →  gate_detector_node  →  /gate/detection
                                      ↓
DVL, IMU, depth  →  gate_mission  →  /control/cmd_vel
                                      + /qualification/mission/target_depth
                                      ↓
                         depth_controller  →  /control/heave_cmd
                                      ↓
                         cmd_vel_3d_merge  →  /control/cmd_vel_3d
                                      ↓
                         thruster_allocator_3d  (external)
```

Ball-dropping (perception only in this repo):

```
/front/image_raw   →  front_drum_detector
/bottom/image_raw  →  bottom_drum_detector
         both      →  drum_detection_fusion  →  /ball_dropping/fusion/*
```

Acquisition / reacquisition **control** packages are empty scaffolds.

---

## 2. Full directory tree (this folder)

```text
Timi_Control/
├── README.md                          ← this file
├── .gitignore                         ← colcon/Python/IDE ignore rules
├── timi_auv_control/                  ← ROS 2 package: 6-DOF mother node
└── sauvc_competition_tasks/           ← separate competition packages
    ├── README.md                      ← competition-stack overview
    ├── auv_depth_control/
    ├── qualification_task/
    └── ball_dropping_task/
```

Git: the parent repo tracks `timi_auv_control` and this README. `sauvc_competition_tasks/` may contain its **own** nested `.git` if it was cloned as a separate repository.

---

## 3. Root files

### 3.1 `README.md`

This manual.

### 3.2 `.gitignore`

Ignores ROS 2 build artifacts (`build/`, `install/`, `log/`), Python bytecode and venvs, IDE files, coverage, and log backups. Source and YAML are **not** ignored.

---

## 4. `timi_auv_control/` — core 6-DOF control (file by file)

ROS 2 **ament_python** package named `timi_auv_control`, version **1.0.0**, license **Apache-2.0**.

**Vehicle assumption:** fully actuated AUV, **eight Blue Robotics T200** thrusters:

- Four **horizontal** (vectored ~45°) at the corners: `stfr`, `stbk`, `psfr`, `psbk` (starboard/port, front/back).
- Four **heave** (vertical) clustered inward: `hstfr`, `hstbk`, `hpsfr`, `hpsbk`.

**Odometry contract (required):**

| Field | Interpretation |
|-------|----------------|
| `pose.pose.position` | **NED** position (North, East, Down) in `frame_id` |
| `pose.pose.orientation` | Quaternion of **body relative to NED** |
| `twist.twist.linear` | **Body-frame** linear velocity (m/s) |
| `twist.twist.angular` | Body-frame angular velocity (rad/s) |

The node does **not** look up TF. Wrong conventions produce wrong PWM.

**Not implemented (by design):** sensor-fault handling, added-mass / quadratic damping model, buoyancy torque from COB–COG, bucket/magnet missions.

---

### 4.1 Package metadata

| File | Purpose |
|------|---------|
| `package.xml` | ROS package name, deps (`rclpy`, `geometry_msgs`, `nav_msgs`, `std_msgs`, `sensor_msgs`, `tf2_*`), build type `ament_python`. |
| `setup.py` | Installs Python modules, launch, YAML, mission READMEs; console script `mother_node`. Python deps: numpy, scipy, PyYAML. |
| `setup.cfg` | Places scripts under `lib/timi_auv_control` (ROS 2 convention). |
| `resource/timi_auv_control` | Empty marker file for `ament_index` (package discovery). |
| `README.md` | Package-level control manual (architecture, frames, tuning). |

---

### 4.2 Launch

#### `launch/auv_control.launch.py`

Starts **one** node: executable `mother_node`, node name `timi_auv_mother`.

Launch arguments:

| Argument | Default | Meaning |
|----------|---------|---------|
| `mission_type` | `station_keeping` | `station_keeping` / `waypoint` / `path_following` (aliases exist in code) |
| `mission_file` | share path to `station_keeping/mission.yaml` | Absolute path to mission YAML |
| `odom_topic` | `/odometry/filtered` | Odometry input |
| `pwm_topic` | `/auv/thrusters/pwm` | PWM output |
| `control_rate_hz` | `50.0` | Control loop rate (recommended 50–100 Hz) |

Parameters passed into the node also include absolute paths to `control_params.yaml` and `geometry.yaml` from the installed share directory.

---

### 4.3 Configuration

#### `config/control_params.yaml` — **global** gains (all missions)

| Section | Keys | Meaning |
|---------|------|---------|
| `vehicle` | `mass_kg` (40), `Ixx/Iyy/Izz` (placeholders 2.0) | Mass used conceptually; inertias are guesses until identified |
| `position_loop` | `kp` [N,E,D], `vel_ned_max` | Outer loop: position error → commanded NED velocity, then saturated |
| `velocity_loop` | `kp`, `ki`, `kd`, `integral_limit` | Inner PID: body velocity error → body forces |
| `attitude_loop` | `kp`, `kd` | Roll/pitch/yaw error + rate damping → moments |
| `limits` | `force_max_n`, `moment_max_nm` | Clip wrench before allocation |
| `thrusters` | T200 forward/reverse max N, `force_limit_n`, PWM 1200/1500/1800 | Force clamp + PWM map |

#### `config/geometry.yaml` — thruster layout

- Body: **+X forward, +Y starboard, +Z down** (NED body).
- `d: 0.707…` is \(1/\sqrt{2}\) (45° components).
- `thruster_order` is the **exact PWM array order**.
- Each thruster: `position_body_m` and `direction_body` (unit vector of **force on the vehicle** when thrust is positive).

Horizontal corners ≈ ±0.450 m X, ±0.22525 m Y. Heave cluster ≈ ±0.142 m X, ±0.168 m Y. Heave directions are `(0,0,-1)` so positive allocated force along that vector is **upward** (negative NED Z).

If bench tests show wrong surge/yaw coupling, **negate** that thruster’s `direction_body` in YAML (do not change allocation math in code unless you intend to).

#### `config/missions/README.md`

Index of mission folders and how to add a new mission (Python plugin + folder + `setup.py` install).

#### `config/missions/station_keeping/`

| File | Role |
|------|------|
| `mission.yaml` | `position_ned`, `attitude_deg`, `duration_s` (null = hold forever) |
| `README.md` | Full station-keeping ops/tuning notes |

Default pose: `[0, 0, 5]` m NED (5 m down), attitude 0, indefinite hold.

#### `config/missions/waypoint/`

| File | Role |
|------|------|
| `mission.yaml` | Ordered `waypoints_ned`, `acceptance_radius_m`, `cruise_speed_m_s`, `yaw_mode`, `fixed_yaw_deg` |
| `README.md` | Pass-through vs last-stop behavior |

Default: three points at 5–6 m depth, radius 0.6 m, cruise 0.3 m/s, yaw along path.

#### `config/missions/path_following/`

| File | Role |
|------|------|
| `mission.yaml` | Spline control points, trapezoid accel/cruise/decel, optional `total_time_s`, yaw mode |
| `README.md` | Spline vs two-point line, time modes |

---

### 4.4 Python package `timi_auv_control/`

#### `__init__.py`

Package docstring only.

#### `mother_node.py` — **the** ROS node

Class `MotherNode` (`timi_auv_mother`):

1. Requires parameters `control_config`, `geometry_config`, `mission_config` (paths). Fatal if missing.
2. Loads YAML, builds thruster list and **6×8** allocation matrix `B`.
3. Instantiates a mission from `mission_type`.
4. Subscribes to odometry; timer at `control_rate_hz`.
5. Each tick: if no odom → **neutral PWM**. Else mission `step()` → wrench → `pinv(B)` + clip → PWM publish.
6. If mission `finished` or mission exception → **neutral PWM** (1500 µs × 8).

`_resolve()` can also find files under the package share directory if a relative path is given.

`main()`: `rclpy.init`, spin, shutdown.

**Console executable:** `ros2 run timi_auv_control mother_node`.

#### `controllers.py` — wrench from errors

- `rot_body_to_ned_from_quat` — \(v_{ned} = R_{nb} v_{body}\).
- `wrap_pi` — angle wrap.
- `PID1D` — scalar PID with integral clamp.
- `WrenchController.compute_wrench`:
  1. Position error in NED → `v_ned_cmd = Kp ⊙ e + v_ff`, saturate to `vel_ned_max`.
  2. Rotate to body: `v_des_body = R_bn @ v_ned_cmd`.
  3. Velocity PID on body axes → `Fx, Fy, Fz`.
  4. Euler error (roll/pitch/yaw) + rate PD → `Mx, My, Mz`.
  5. Return 6-vector wrench.

Gains come from `configure_from_yaml`.

#### `geometry.py`

- `ThrusterSpec` dataclass.
- `load_geometry`, `build_thruster_list` (order from YAML).
- `build_allocation_matrix`: column \(i\) is \([F_i; r_i \times F_i]\) so \(\tau = B f\).
- `yaw_from_quaternion_ned`, `roll_pitch_yaw_from_quat` (ZYX / aerospace).

#### `allocation.py`

`allocate_wrench(wrench, B, f_min, f_max)`: \(f = B^{+} \tau\), then clip. Returns `(f, saturated)`.

Pseudoinverse + clip does **not** re-solve a constrained QP; heavy saturation means tracking error.

#### `thruster_model.py`

Piecewise linear T200 map @ 16 V defaults (~51.5 N forward, ~40.2 N reverse):

- Positive force → PWM from 1500 toward 1800.
- Negative force → PWM from 1500 toward 1200.

#### `missions/__init__.py`

Exports `MissionBase`, `MissionCommand`, `VehicleState`, and the three mission classes.

#### `missions/base.py`

- `VehicleState`: time, `p_ned`, `v_body`, `omega_body`, quaternion `(x,y,z,w)`.
- `MissionCommand`: desired pose/attitude, optional NED velocity FF, optional body omega, `finished` flag.
- `MissionBase`: `reset()`, `step(state, dt)`.

#### `missions/station_keeping.py`

Holds fixed NED pose and attitude. `duration_s` None/negative → never finishes; else finishes after wall time from first step. Zero velocity/omega feedforward.

Aliases in mother: `station_keeping`, `station`, `hover`.

#### `missions/waypoint.py`

State machine TRACK → DONE. Advances waypoint when inside sphere. Last waypoint: finish. Feedforward along error vector capped by `cruise_speed_m_s`. Yaw `path` = `atan2(east, north)` or `fixed`.

#### `missions/path_following.py`

- 2 points: straight line.
- ≥3 points: SciPy `splprep`/`splev` B-spline (`k = min(3, n-1)`).
- Motion: either uniform in `total_time_s`, or trapezoidal along arc length.
- Yaw: `tangent_h`, `fixed`, or `hold_initial`.

When `finished`, mother still publishes **neutral**, so the vehicle will **not** hover at the end unless you switch to station keeping.

---

### 4.5 How to build and run (core)

From a ROS 2 workspace that contains this package (this repo root works if you use `--paths` or put the package under `src/`):

```bash
source /opt/ros/humble/setup.bash   # or jazzy/iron
cd /path/to/Timi_Control
colcon build --packages-select timi_auv_control
source install/setup.bash

ros2 launch timi_auv_control auv_control.launch.py

ros2 launch timi_auv_control auv_control.launch.py \
  mission_type:=waypoint \
  mission_file:=$(ros2 pkg prefix timi_auv_control)/share/timi_auv_control/config/missions/waypoint/mission.yaml
```

You must have a live `/odometry/filtered` (or remapped topic) or PWM stays at 1500.

---

## 5. `sauvc_competition_tasks/` — competition software (file by file)

This tree is **separate** from `timi_auv_control`. It does **not** publish `/auv/thrusters/pwm` itself. It publishes `/control/cmd_vel` and `/control/heave_cmd` / `/control/cmd_vel_3d` for an **external** 3D allocator.

Internal README: `sauvc_competition_tasks/README.md` (topics, troubleshooting, known gaps).

ROS 2 packages **inside** this tree:

| Package name | Location | Type |
|--------------|----------|------|
| `auv_depth_control` | `auv_depth_control/` | ament_python |
| `qualification_gate_interfaces` | `qualification_task/qualification_gate_interfaces/` | ament_cmake (msgs) |
| `qualification_gate_detection` | `qualification_task/detection/` | ament_python |
| `qualification_gate_control` | `qualification_task/control/` | ament_python |
| `ball_dropping_detection` | `ball_dropping_task/detection/` | ament_python |

**External packages referenced but not in this folder:** `auv_2d_control`, `auv_3d_control`, `dvl_to_odom_bridge`, `dvl_msgs`, `sbg_driver`.

---

### 5.1 `auv_depth_control/`

Pressure-primary **depth hold** plus **planar + heave merge**.

#### Metadata

| File | Purpose |
|------|---------|
| `package.xml` | Package `auv_depth_control` 0.1.0; rclpy, std_msgs, geometry_msgs, std_srvs |
| `setup.py` | Executables `depth_controller`, `cmd_vel_3d_merge` |
| `setup.cfg` | Script install path |
| `resource/auv_depth_control` | ament_index marker |
| `auv_depth_control/__init__.py` | Empty |

#### `auv_depth_control/depth_controller_node.py`

Node `depth_controller`:

- Subscribes `/auv/depth` (`Float32`, depth **downward**), `/ping1d/data` (optional floor sonar).
- Optional mission setpoint topic (default in YAML: `/qualification/mission/target_depth`).
- Service `~/arm` (`std_srvs/SetBool`) — relative name `arm` → typically `/depth_controller/arm`.
- Publishes `/control/heave_cmd` as `geometry_msgs/Vector3`:
  - **x** = heave encoded as PWM-like number (`neutral + heave * per_unit`)
  - **y** = depth error
  - **z** = measured depth

PID on `target - depth`. Derivative is on **measured depth** (filtered), with sign so damping opposes depth rate. `output_sign` in YAML is **-1.0** (convention for this vehicle). Optional ping **floor guard** (disabled in default YAML). Disarmed → heave command 0. `auto_arm_on_start: true` in default YAML.

#### `auv_depth_control/cmd_vel_merge.py`

Node `cmd_vel_3d_merge`:

- Takes last `/control/cmd_vel` (x, y, yaw rate) and last heave (decoded from Vector3.x using same PWM scale).
- Publishes `/control/cmd_vel_3d` at `publish_rate_hz` (20 Hz): `linear.z = heave`, roll/pitch rates 0.
- Optional `mission_duration_sec`: after timeout, publish zero twist and optionally call depth **disarm**. Default YAML sets duration **0** so the **gate mission** owns the time limit.

#### `config/depth_control_default.yaml`

All ROS parameters for both nodes (nested under `depth_controller:` and `cmd_vel_3d_merge:`). Notable: `target_depth_m: 1.5`, mission latch (`mission_target_stale_sec: 0`), ping guard off, auto-arm on.

#### `launch/depth_stack.launch.py`

Starts **four** nodes:

1. `auv_2d_control/controller_2d` (**external**)
2. `depth_controller`
3. `cmd_vel_3d_merge`
4. `auv_3d_control/thruster_allocator_3d` (**external**)

Do **not** also run a 2D allocator that publishes the same thruster topic. This launch is for 2D controller + depth, **not** for `gate_mission` (which owns `/control/cmd_vel`).

---

### 5.2 Qualification — interfaces

#### `qualification_task/qualification_gate_interfaces/`

| File | Purpose |
|------|---------|
| `package.xml` | CMake interface package |
| `CMakeLists.txt` | `rosidl_generate_interfaces` on `msg/GateDetection.msg` |
| `msg/GateDetection.msg` | Contract between detector and mission |

**Message fields:**

| Field | Meaning |
|-------|---------|
| `pole1_detected` / `pole2_detected` | At least one vertical candidate on left/right-ish |
| `gate_center_valid` | Two poles + stability streak |
| `gate_center_x_px` | Horizontal center of opening (NaN if invalid) |
| `alignment_status` | 0 UNKNOWN, 1 ALIGNED (\|error\|≤8 px), 2 NOT_ALIGNED |
| `center_error_px` | `gate_center - image_center` (positive = gate to the **right**) |

Build **this package first** (or with the others); Python packages depend on the generated types.

---

### 5.3 Qualification — detection (ROS)

Folder: `qualification_task/detection/` → ROS package **`qualification_gate_detection`**.

| File | Purpose |
|------|---------|
| `package.xml` / `setup.py` / `setup.cfg` | Package metadata; executable `gate_detector_node` |
| `resource/qualification_gate_detection` | ament_index marker |
| `qualification_gate_detection/__init__.py` | Empty |
| `qualification_gate_detection/gate_detector_node.py` | OpenCV detector node |
| `config/gate_detector_params.yaml` | HSV, Canny, Hough, geometry, topics |
| `launch/gate_detector.launch.py` | Starts `gate_detector` with YAML |
| `detection/assets/.gitkeep` | Keeps empty assets dir in git |
| `detection/assets/image_2170.png` | Sample image (not used by the ROS node) |

#### `gate_detector_node.py` algorithm (summary)

1. Convert camera `sensor_msgs/Image` to BGR.
2. Optional **red HSV dual-range** mask (orange/red poles), morphology, blur.
3. Canny + `HoughLinesP`; keep near-vertical long lines; ignore top `roi_top_frac`.
4. Score pole **pairs** (separation, vertical overlap, length, slight center bias).
5. If pair found for `stable_center_frames` consecutive frames → valid center.
6. Publish `GateDetection`; optional debug BGR and mask; **heartbeat** republishes last detection at `publish_rate_hz` even if images stop (with a warning).

Default image topic in YAML: **`/front/image_raw`** (node default in code is `/camera/image_raw`; YAML overrides).

---

### 5.4 Qualification — mission control (ROS)

Folder: `qualification_task/control/` → package **`qualification_gate_control`**.

| File | Purpose |
|------|---------|
| `package.xml` | Depends on interfaces, `dvl_msgs`, `sbg_driver`; exec_depend on detection, depth, `auv_3d_control`, DVL bridge |
| `setup.py` | Executable `gate_mission` |
| `qualification_gate_control/__init__.py` | Empty |
| `qualification_gate_control/gate_mission_node.py` | Finite-state mission |
| `config/qualification_mission.yaml` | All mission parameters |
| `launch/qualification_full.launch.py` | Detector + mission only |
| `launch/qualification_complete.launch.py` | Detector + depth + merge + 3D allocator + mission + optional DVL bridge |

**Does not start `controller_2d`.** `gate_mission` owns `/control/cmd_vel`.

#### Mission states (`MissionState`)

In order (typical):

1. **DEPTH_DIVE** — wait until depth within tolerance for hold time (or skip if `skip_depth_acquisition`).
2. **APPROACH** — surge toward gate until vision sees a valid center (timeout).
3. **ALIGN** — sway/yaw from `center_error_px`; one-pole scan if only one pole.
4. **PASS** — surge through gate.
5. **CLEAR_DISTANCE** — DVL horizontal distance `post_gate_forward_m`.
6. **TURN_AROUND** — ~180° in place with IMU damping.
7. **RETURN_APPROACH** / **SECOND_ALIGN** / **SECOND_PASS** / **SECOND_CLEAR_DISTANCE** — return through gate, then extra forward `post_second_pass_forward_m`.
8. **SURFACE** — publish `surface_depth_m`, hold, then **IDLE**.

Also: global `mission_timeout_sec` (YAML **180 s**) can force surface.

Publishes:

- `/control/cmd_vel` (`Twist`)
- `/qualification/mission/status` (`String`)
- `/qualification/mission/target_depth` (`Float32`) if enabled

Subscribes: gate, DVL `/dvl/position` (`dvl_msgs/DVLDR`), SBG IMU, depth.

YAML note: `mission_target_depth_m: -0.140` is the **configured** dive target in this file (sign must match your depth convention; depth controller treats `/auv/depth` as downward). Align YAML with the real sensor.

---

### 5.5 Qualification — standalone scripts (not ROS)

Folder: `qualification_task/scripts/`

These are **offline** OpenCV tools. They are **not** the ROS detector. They import modules that are **not in this repository** (`gate_detection_core`, `gate_draw`, `gate_pose_pnp`). Running them as-is will fail until those modules exist on `PYTHONPATH`.

| File | Purpose |
|------|---------|
| `qualification_gate_detector.py` | CLI: loop PNG folder, show windows, print two/one/none pole stats; optional PnP and temporal filter |
| `gate_pipeline.py` | Single-frame pipeline config + `process_frame` (depends on missing modules) |
| `gate_temporal.py` | EMA smoother for left/right pole x; reset on large jumps |

`qualification_gate_detector.py` looks for a **directory** of `.png` files (default `images/image_qualification_01` next to the script), not a single PNG.

---

### 5.6 Ball dropping

```text
ball_dropping_task/
├── detection/                 ← real ROS package
├── acquisition_control/       ← scaffold only
├── reacquisition_control/     ← scaffold only
└── interfaces/                ← .gitkeep only
```

Scaffolds contain empty dirs + `.gitkeep` (package names reserved for future control). **No nodes.**

#### `detection/` → package **`ball_dropping_detection`**

| File | Purpose |
|------|---------|
| `package.xml` / `setup.py` / `setup.cfg` | Three executables |
| `resource/ball_dropping_detection` | ament_index |
| `ball_dropping_detection/__init__.py` | Empty |
| `common.py` | `clamp`, `status_json` (compact JSON status strings) |
| `front_drum_detector.py` | Front camera: contours + blue/red color scoring, lock, distance-from-size |
| `bottom_drum_detector.py` | Bottom camera: HSV blobs, circularity, EMA center, alignment error |
| `drum_detection_fusion.py` | Prefer fresh high-confidence **bottom**, else **front** |
| `config/ball_dropping_detection.yaml` | All three nodes’ parameters in one file |
| `launch/ball_dropping_detection.launch.py` | Starts all three |

**Front node topics:**

- In: `/front/image_raw`
- Out: `/ball_dropping/front/blue_target` (`Vector3Stamped`, z often used as confidence), `blue_distance_m`, `status`, `candidates`, `debug_image`

**Bottom node topics:**

- In: `/bottom/image_raw`
- Out: `/ball_dropping/bottom/center`, `blue_target` (pixel error), `red_candidates`, `status`, `debug_image`

**Fusion:**

- Prefers bottom if age &lt; timeout and `vector.z` ≥ `prefer_bottom_min_conf`
- Else front if similar with `front_min_conf`
- Else invalid target `(-1, -1, 0)`
- Out: `/ball_dropping/fusion/target`, `state`, `status`

There is **no** ball-drop mission FSM in this repo.

---

## 6. Cross-tree comparison (do not mix blindly)

| Topic | `timi_auv_control` | `sauvc_competition_tasks` |
|-------|--------------------|---------------------------|
| Command to thrusters | PWM µs on `/auv/thrusters/pwm` | Twist 3D then **external** allocator |
| Depth | Inner loop on NED Z from odometry | Separate pressure PID |
| Missions | Station / waypoint / path plugins | Gate FSM; drums perception only |
| Odometry | Required `nav_msgs/Odometry` | DVL + IMU messages for gate mission |
| Thruster geometry | `geometry.yaml` in this repo | In external `auv_3d_control` |

Running **both** stacks at once on the same vehicle without a clear arbitration layer will fight over actuators.

---

## 7. Building the competition packages

Place this tree’s packages where `colcon` can see them (e.g. copy/symlink into a workspace `src/`, or `colcon build --paths sauvc_competition_tasks/**`).

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select \
  qualification_gate_interfaces \
  qualification_gate_detection \
  qualification_gate_control \
  auv_depth_control \
  ball_dropping_detection
source install/setup.bash
```

Interfaces **must** build successfully before the Python packages that import `GateDetection`.

### Run examples

```bash
ros2 launch auv_depth_control depth_stack.launch.py
ros2 launch qualification_gate_detection gate_detector.launch.py
ros2 launch qualification_gate_control qualification_full.launch.py
ros2 launch qualification_gate_control qualification_complete.launch.py
ros2 launch ball_dropping_detection ball_dropping_detection.launch.py
```

`qualification_complete` and `depth_stack` **fail** if `auv_3d_control` / `auv_2d_control` / `dvl_to_odom_bridge` are not installed.

---

## 8. Topic cheat sheet

### Core (`timi_auv_control`)

| Direction | Topic (default) | Type |
|-----------|-----------------|------|
| In | `/odometry/filtered` | `nav_msgs/Odometry` |
| Out | `/auv/thrusters/pwm` | `std_msgs/Int32MultiArray` (8 × µs) |

### Depth

| Direction | Topic / service | Type |
|-----------|-----------------|------|
| In | `/auv/depth` | `Float32` |
| In | `/ping1d/data` | `Float32` |
| In | `/qualification/mission/target_depth` | `Float32` (optional) |
| Out | `/control/heave_cmd` | `Vector3` |
| Srv | `/depth_controller/arm` | `SetBool` |
| In | `/control/cmd_vel` | `Twist` |
| Out | `/control/cmd_vel_3d` | `Twist` |

### Gate

| Direction | Topic | Type |
|-----------|-------|------|
| In | `/front/image_raw` (YAML) | `Image` |
| Out | `/gate/detection` | `GateDetection` |
| Out | `/gate/mask`, `/gate/debug_image` | `Image` |
| Out | `/control/cmd_vel` | `Twist` |
| Out | `/qualification/mission/status` | `String` |

### Drums

See §5.6. Cameras: `/front/image_raw`, `/bottom/image_raw`.

---

## 9. Tuning order (practical)

**Core 6-DOF:** verify odometry signs → motors off, PWM ~1500 at zero error → tune depth/Z → horizontal position → attitude → watch allocation saturation.

**Gate:** tune HSV/mask until poles are stable → `stable_center_frames` → alignment deadbands and `kp_sway`/`kp_yaw` in the pool → DVL distance signs → 180° turn IMU sign (`turn_imu_gyro_z_sign`).

**Drums:** tune HSV and area/circularity with debug images; fusion timeouts if cameras drop.

---

## 10. Safety

- Core stack: **no odom** or **mission finished** → all PWM **1500**.
- Competition merge: optional mission timer then zero `cmd_vel_3d` + depth disarm.
- None of this is safety-certified. Use hardware e-stop, tether, and pool tests.
- PWM is clipped 1200–1800 µs in the core T200 map; still respect ESC and propeller safety.

---

## 11. Known gaps

- Standalone gate scripts depend on **missing** Python modules.
- Ball-drop **control** and **interfaces** are placeholders.
- Competition launch files need **external** 2D/3D control and DVL packages.
- Core package lists `tf2` in `package.xml` but the mother node does not use TF.
- Two control philosophies coexist; integrating them is a future design choice, not done here.
- No automated tests are checked into this workspace.

---

## 12. File index (every meaningful path)

### Workspace root

- `README.md` — this manual  
- `.gitignore` — ignore rules  

### `timi_auv_control`

- `package.xml`, `setup.py`, `setup.cfg`, `README.md`, `resource/timi_auv_control`  
- `launch/auv_control.launch.py`  
- `config/control_params.yaml`, `config/geometry.yaml`  
- `config/missions/README.md`  
- `config/missions/station_keeping/{mission.yaml, README.md}`  
- `config/missions/waypoint/{mission.yaml, README.md}`  
- `config/missions/path_following/{mission.yaml, README.md}`  
- `timi_auv_control/{__init__, mother_node, controllers, geometry, allocation, thruster_model}.py`  
- `timi_auv_control/missions/{__init__, base, station_keeping, waypoint, path_following}.py`  

### `sauvc_competition_tasks`

- `README.md`  
- `auv_depth_control/` — depth PID + merge + `depth_stack.launch.py`  
- `qualification_task/qualification_gate_interfaces/` — `GateDetection.msg`  
- `qualification_task/detection/` — ROS gate detector + `assets/image_2170.png`  
- `qualification_task/control/` — `gate_mission` + two launch files  
- `qualification_task/scripts/` — offline detector helpers  
- `ball_dropping_task/detection/` — front/bottom/fusion  
- `ball_dropping_task/{acquisition_control, reacquisition_control, interfaces}/` — empty scaffolds  

---

## 13. License

- `timi_auv_control`: Apache-2.0 (`package.xml`).  
- Competition packages: Apache-2.0 (`package.xml` in each package).  

---

*End of workspace manual. For mission YAML field-by-field ops notes, also see the READMEs under `timi_auv_control/config/missions/<name>/`.*
