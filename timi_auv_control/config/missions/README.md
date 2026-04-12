# Mission configuration (folders)

Each **mission type** has its **own directory** here. Inside each directory:

| File | Purpose |
|------|---------|
| **`mission.yaml`** | Parameters the mother node loads for that run (`mission_config` points here) |
| **`README.md`** | Mission-specific documentation (behavior, YAML keys, launch examples) |

**Shared** settings (gains, thruster geometry) stay in the parent `config/` folder:

- `../control_params.yaml`
- `../geometry.yaml`

**Python** mission logic lives in the package under `timi_auv_control/missions/*.py` (not here).

## Missions included

| Folder | `mission_type` values |
|--------|------------------------|
| [`station_keeping/`](station_keeping/) | `station_keeping`, `station`, `hover` |
| [`waypoint/`](waypoint/) | `waypoint`, `waypoints` |
| [`path_following/`](path_following/) | `path_following`, `path`, `spline` |

## Adding a new mission

1. Add a Python module under `timi_auv_control/missions/` and register it in `mother_node.py`.
2. Create a new folder here, e.g. `config/missions/my_mission/`, with `mission.yaml` + `README.md`.
3. Install the new files in `setup.py` (`data_files`).
4. Launch with `mission_type` and `mission_file:=.../my_mission/mission.yaml`.
