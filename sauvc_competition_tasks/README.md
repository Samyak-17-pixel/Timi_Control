# SAUVC Control Workspace

Comprehensive ROS 2 workspace for SAUVC mission control and perception, covering:

- depth control and 2D->3D command merging,
- qualification gate detection + mission execution,
- ball-dropping detection (front/bottom camera + fusion).

This repository is organized as multiple ROS 2 packages grouped by task domain.

## Repository Layout

```text
sauvc_competition_tasks/
├── auv_depth_control/
│   ├── auv_depth_control/                 # Python nodes
│   ├── config/                            # Depth + merge params
│   ├── launch/                            # Depth stack launch
│   ├── package.xml
│   └── setup.py
├── qualification_task/
│   ├── detection/                         # qualification_gate_detection package
│   ├── control/                           # qualification_gate_control package
│   ├── qualification_gate_interfaces/     # GateDetection.msg package
│   └── scripts/                           # Standalone (non-ROS) gate helpers
│       ├── qualification_gate_detector.py
│       ├── gate_pipeline.py
│       └── gate_temporal.py
└── ball_dropping_task/
    ├── detection/                         # ball_dropping_detection package
    ├── acquisition_control/               # scaffold (placeholder)
    ├── reacquisition_control/             # scaffold (placeholder)
    └── interfaces/                        # scaffold (placeholder)
```

## Packages Overview

### 1) `auv_depth_control`

ROS 2 Python package for pressure-based depth hold and heave/planar command fusion.

#### Nodes

- `depth_controller` (`auv_depth_control/depth_controller_node.py`)
  - Subscribes to depth and ping,
  - runs PID and optional floor-clearance guard,
  - publishes heave command,
  - exposes arming service.
- `cmd_vel_3d_merge` (`auv_depth_control/cmd_vel_merge.py`)
  - merges `/control/cmd_vel` (2D) and `/control/heave_cmd`,
  - publishes `/control/cmd_vel_3d`.

#### Launch

- `auv_depth_control/launch/depth_stack.launch.py`
  - starts:
    - `auv_2d_control/controller_2d` (external package),
    - `auv_depth_control/depth_controller`,
    - `auv_depth_control/cmd_vel_3d_merge`,
    - `auv_3d_control/thruster_allocator_3d` (external package).

#### Config

- `auv_depth_control/config/depth_control_default.yaml`
  - contains parameters for both `depth_controller` and `cmd_vel_3d_merge`.

---

### 2) `qualification_gate_interfaces`

ROS 2 interface package (CMake + `rosidl`) providing:

- `msg/GateDetection.msg`
  - `pole1_detected`, `pole2_detected`, `gate_center_valid`,
  - `gate_center_x_px`,
  - `alignment_status` (`0=UNKNOWN`, `1=ALIGNED`, `2=NOT_ALIGNED`),
  - `center_error_px`.

This message is the contract between detector and mission-control nodes.

---

### 3) `qualification_gate_detection` (under `qualification_task/detection`)

ROS 2 Python package for qualification gate perception.

#### Node

- `gate_detector_node` (`qualification_gate_detection/gate_detector_node.py`)
  - image-based pole detection using HSV filtering + morphology + Canny/Hough processing,
  - produces gate center/alignment state.

#### Launch

- `qualification_task/detection/launch/gate_detector.launch.py`

#### Config

- `qualification_task/detection/config/gate_detector_params.yaml`

---

### 4) `qualification_gate_control` (under `qualification_task/control`)

ROS 2 Python package implementing qualification mission logic as a finite-state machine.

#### Node

- `gate_mission` (`qualification_gate_control/gate_mission_node.py`)
  - subscribes to gate detection, DVL, IMU, and depth,
  - publishes mission command velocity + mission status,
  - publishes mission target depth for depth controller integration.

#### Launches

- `qualification_task/control/launch/qualification_full.launch.py`
- `qualification_task/control/launch/qualification_complete.launch.py`

#### Config

- `qualification_task/control/config/qualification_mission.yaml`

---

### 5) `ball_dropping_detection` (under `ball_dropping_task/detection`)

ROS 2 Python package for drum detection from front and bottom cameras with fusion.

#### Nodes

- `front_drum_detector` (`ball_dropping_detection/front_drum_detector.py`)
  - contour + color-scoring based front camera detection,
  - estimates blue-target distance from apparent size.
- `bottom_drum_detector` (`ball_dropping_detection/bottom_drum_detector.py`)
  - HSV segmentation + morphology + circularity filtering,
  - stabilized target center and alignment error output.
- `drum_detection_fusion` (`ball_dropping_detection/drum_detection_fusion.py`)
  - fuses front/bottom detections using confidence and timeout logic.

#### Launch

- `ball_dropping_task/detection/launch/ball_dropping_detection.launch.py`

#### Config

- `ball_dropping_task/detection/config/ball_dropping_detection.yaml`

## Build and Environment Setup

> Assumes Ubuntu with ROS 2 already installed and sourced.

### 1) Build

From workspace root that contains this folder's packages:

```bash
colcon build
```

If this folder itself is your workspace root, run the same command from here.

### 2) Source

```bash
source install/setup.bash
```

### 3) Optional package-selective build

```bash
colcon build --packages-select \
  qualification_gate_interfaces \
  qualification_gate_detection \
  qualification_gate_control \
  auv_depth_control \
  ball_dropping_detection
```

## Runtime Entry Points

### Depth + control stack

```bash
ros2 launch auv_depth_control depth_stack.launch.py
```

### Qualification detector only

```bash
ros2 launch qualification_gate_detection gate_detector.launch.py
```

### Qualification mission stack

```bash
ros2 launch qualification_gate_control qualification_full.launch.py
```

### Qualification complete launch

```bash
ros2 launch qualification_gate_control qualification_complete.launch.py
```

### Ball-dropping detection stack

```bash
ros2 launch ball_dropping_detection ball_dropping_detection.launch.py
```

## Topic and Service Reference

Below are default interfaces as declared in code/config. They can be remapped or overridden via params.

### `depth_controller`

- Subscriptions:
  - `/auv/depth` (`std_msgs/Float32`)
  - `/ping1d/data` (`std_msgs/Float32`)
  - `/qualification/mission/target_depth` (`std_msgs/Float32`, optional via param)
- Publisher:
  - `/control/heave_cmd` (`geometry_msgs/Vector3`)
- Service:
  - `/depth_controller/arm` (`std_srvs/SetBool`)

### `cmd_vel_3d_merge`

- Subscriptions:
  - `/control/cmd_vel` (`geometry_msgs/Twist`)
  - `/control/heave_cmd` (`geometry_msgs/Vector3`)
- Publisher:
  - `/control/cmd_vel_3d` (`geometry_msgs/Twist`)

### `gate_detector_node`

- Subscription:
  - `/camera/image_raw` (`sensor_msgs/Image`)
- Publishers:
  - `/gate/detection` (`qualification_gate_interfaces/msg/GateDetection`)
  - `/gate/debug_image` (`sensor_msgs/Image`)
  - `/gate/mask` (`sensor_msgs/Image`)

### `gate_mission`

- Subscriptions (configurable):
  - `/gate/detection`
  - `/dvl/position`
  - `/sbg/imu_data`
  - `/auv/depth`
- Publishers:
  - `/control/cmd_vel` (`geometry_msgs/Twist`)
  - `/qualification/mission/status` (`std_msgs/String`)
  - `/qualification/mission/target_depth` (`std_msgs/Float32`)

### `front_drum_detector`

- Subscription:
  - `/front/image_raw` (`sensor_msgs/Image`)
- Publishers:
  - `/ball_dropping/front/blue_target` (`geometry_msgs/Vector3Stamped`)
  - `/ball_dropping/front/blue_distance_m` (`std_msgs/Float32`)
  - `/ball_dropping/front/status` (`std_msgs/String`)
  - `/ball_dropping/front/candidates` (`std_msgs/String`)
  - `/ball_dropping/front/debug_image` (`sensor_msgs/Image`)

### `bottom_drum_detector`

- Subscription:
  - `/bottom/image_raw` (`sensor_msgs/Image`)
- Publishers:
  - `/ball_dropping/bottom/center` (`geometry_msgs/Vector3Stamped`)
  - `/ball_dropping/bottom/blue_target` (`geometry_msgs/Vector3Stamped`)
  - `/ball_dropping/bottom/red_candidates` (`std_msgs/String`)
  - `/ball_dropping/bottom/status` (`std_msgs/String`)
  - `/ball_dropping/bottom/debug_image` (`sensor_msgs/Image`)

### `drum_detection_fusion`

- Subscriptions:
  - `/ball_dropping/front/blue_target`
  - `/ball_dropping/bottom/center`
  - `/ball_dropping/front/status`
  - `/ball_dropping/bottom/status`
- Publishers:
  - `/ball_dropping/fusion/target` (`geometry_msgs/Vector3Stamped`)
  - `/ball_dropping/fusion/state` (`std_msgs/String`)
  - `/ball_dropping/fusion/status` (`std_msgs/String`)

## Important Parameters

### Depth stack (`depth_control_default.yaml`)

- Control gains: `kp`, `ki`, `kd`
- Output clamping: `min_heave_cmd`, `max_heave_cmd`
- Safety/quality: `use_ping_guard`, `min_clearance_ping_m`, `ping_stale_sec`
- Arming behavior: `auto_arm_on_start`, `reset_integral_on_arm`
- Mission integration: `mission_target_depth_topic`, `mission_target_stale_sec`
- Merge behavior: `mission_duration_sec`, `on_mission_expire_disarm_depth`

### Qualification mission (`qualification_mission.yaml`)

- Mission depth: `mission_target_depth_m`
- State timing constraints: `approach_timeout_sec`, `align_timeout_sec`, `mission_timeout_sec`, etc.
- Visual alignment tuning:
  - `align_deadband_px`, `align_fine_deadband_px`
  - `kp_sway_per_px`, `kp_yaw_per_px`
  - `max_sway_m_s`, `max_yaw_rate_rad_s`
- Turn behavior:
  - `turn_yaw_kp`, `turn_yaw_max_rate_rad_s`, `turn_done_err_rad`
- Sensor conventions:
  - `dvl_yaw_in_degrees`
  - `dvl_use_horizontal_distance`
  - `turn_imu_gyro_z_sign`, `align_imu_gyro_z_sign`

### Ball-dropping (`ball_dropping_detection.yaml`)

- Color segmentation thresholds for blue/red in HSV
- Front detector candidate filters (`contour_min_area`, aspect bounds, etc.)
- Bottom detector shape filters (`min_area_px`, `min_circularity`)
- Fusion arbitration:
  - `prefer_bottom_min_conf`
  - `front_min_conf`
  - source timeout parameters

## Mission Data Flow

### Qualification flow

1. `gate_detector_node` consumes camera stream and publishes `GateDetection`.
2. `gate_mission` consumes `GateDetection`, DVL, IMU, depth.
3. `gate_mission` publishes:
   - planar commands to `/control/cmd_vel`,
   - mission depth target to `/qualification/mission/target_depth`.
4. `depth_controller` tracks target depth and publishes heave command.
5. `cmd_vel_3d_merge` merges planar+heave into `/control/cmd_vel_3d`.
6. external `thruster_allocator_3d` allocates thrusters from `/control/cmd_vel_3d`.

### Ball-dropping flow

1. Front and bottom detectors process camera feeds independently.
2. Fusion node arbitrates active target based on confidence + freshness.
3. Fused target/state are published to `/ball_dropping/fusion/*`.

## External Dependencies / Assumptions

The following packages/interfaces are referenced and must exist in your full ROS environment:

- `auv_2d_control`
- `auv_3d_control`
- `dvl_to_odom_bridge`
- `dvl_msgs`
- `sbg_driver`

OpenCV + `cv_bridge` are required for perception packages.

## Known Gaps / Notes

- `ball_dropping_task/acquisition_control`, `reacquisition_control`, and `interfaces` are placeholders only.
- Standalone files in `qualification_task/scripts` (`qualification_gate_detector.py`, `gate_pipeline.py`) are not the main ROS runtime path and may reference modules not present in this repository.
- No top-level automated tests are currently included in this folder.

## Basic Validation Checklist

After launching a stack, verify:

1. Nodes are running:
   ```bash
   ros2 node list
   ```
2. Topics are present:
   ```bash
   ros2 topic list
   ```
3. Key streams are live:
   ```bash
   ros2 topic hz /gate/detection
   ros2 topic hz /control/cmd_vel_3d
   ```
4. Depth arming service is reachable:
   ```bash
   ros2 service list | grep depth_controller
   ```

## Troubleshooting

### Build fails on interface dependencies

- Build `qualification_gate_interfaces` first (or include it in selected package build).
- Ensure `rosidl_default_generators` is installed.

### Launch fails with missing package executable

- Confirm external packages (`auv_2d_control`, `auv_3d_control`, etc.) are installed and sourced.
- Check package name and executable name using:
  ```bash
  ros2 pkg executables <package_name>
  ```

### No detections from vision nodes

- Verify camera topics (`/camera/image_raw`, `/front/image_raw`, `/bottom/image_raw`) are publishing.
- Tune HSV thresholds and morphology parameters in relevant YAML.
- Enable debug image publishing and inspect with `rqt_image_view`.

### Vehicle does not hold depth correctly

- Verify depth sign convention in your sensor pipeline and mission config.
- Re-tune PID gains and output limits.
- Check whether ping guard is enabled and clamping output.

## Contribution Notes

When adding new mission logic or perception nodes:

- prefer parameters over hardcoded constants,
- keep topic names configurable,
- document default topics/services in package-level docs,
- update launch files and this README in the same change.

