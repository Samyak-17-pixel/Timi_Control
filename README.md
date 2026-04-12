# Timi_Control workspace

This workspace contains the **Timi AUV** ROS 2 control software.

## Package

| Directory | Description |
|-----------|-------------|
| [`timi_auv_control/`](timi_auv_control/) | **6-DOF** AUV control: mother node, thrust allocation, Python mission plugins |

## Documentation

- **System overview, build, tuning, interfaces:**  
  [`timi_auv_control/README.md`](timi_auv_control/README.md)

- **Per-mission docs and parameters** (each mission has its own folder):

| Mission | Folder |
|---------|--------|
| Station keeping / hover | [`timi_auv_control/config/missions/station_keeping/`](timi_auv_control/config/missions/station_keeping/) — `mission.yaml` + `README.md` |
| Waypoint | [`timi_auv_control/config/missions/waypoint/`](timi_auv_control/config/missions/waypoint/) |
| Path following | [`timi_auv_control/config/missions/path_following/`](timi_auv_control/config/missions/path_following/) |

**Shared** tuning and thruster geometry live in `timi_auv_control/config/` (`control_params.yaml`, `geometry.yaml`).

## Build (ROS 2)

Replace `humble` with your ROS 2 distro if different (`jazzy`, `iron`, etc.):

```bash
cd /path/to/Timi_Control
source /opt/ros/humble/setup.bash
colcon build --packages-select timi_auv_control
source install/setup.bash
```

## Run

```bash
ros2 launch timi_auv_control auv_control.launch.py
```

See the package README for parameters and mission selection.
