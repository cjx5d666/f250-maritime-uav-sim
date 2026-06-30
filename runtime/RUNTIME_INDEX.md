# Runtime Index

## UI

- `catkin_ws/src/f250_maritime_uav_sim/scripts/f250_open_control_panel.sh`
- `catkin_ws/src/f250_maritime_uav_sim/scripts/f250_install_desktop_shortcut.sh`

## Main Tasks

- `f250_start_to_p0_hover.sh` - launch P0 hover with `--sensor lidar|depth`
- `f250_run_p0_p8_route.sh` - run the accepted P0-P8 route mission
- `f250_run_fc_3_10_steady_state.sh` - run FC Metric 3.10 from fresh P0 hover
- `f250_stop_all.sh` - stop runtime processes and write stopped state

## Runtime State

- `runtime_state/active_sensor.env` - current LiDAR/Depth selection
- `runtime_state/active_task.env` - current task/runtime state
- `runtime_state/work/` - temporary run directories, logs, worker output, and intermediate files; cleaned after accepted evidence is published

Do not retain accepted evidence in `runtime_state/work/`.

## Current Evidence

Accepted machine evidence lives under `evidence/current/`:

```text
evidence/current/
  index.json
  route_p0_p8/
    manifest.json
    status.env
    inputs/
      route_waypoints.csv
      effective_scene.yaml
      params.json
    measurements/
      actual_trajectory.csv
    metrics/
      acceptance_summary.json
      metric_summary.json
      metrics_full.json
      waypoint_errors.csv
  fc_3_10/
    manifest.json
    status.env
    inputs/
      decagon_points.csv
    measurements/
      samples.csv
      phases.csv
    metrics/
      summary.json
      geometry_audit.json
```

`maintenance/publish_current_review.py` publishes a whitelist of accepted machine evidence into this layout. `maintenance/generate_current_index.py` rebuilds `evidence/current/index.json`. `maintenance/sanitize_current_evidence.py` removes transient files and machine-local paths.


## Map Authority And Plots

- Map authority: `map_authority/p0p8_clean_scene/`
- Renderer: `map_authority/p0p8_clean_scene/render_map.py`
- Plan/result PNGs remain in the map-authority directory.
- Result plot reads `evidence/current/route_p0_p8/measurements/actual_trajectory.csv`.
- PNGs are review outputs, not geometry truth.

## External Derived Package

`maintenance/build_delivery_package.py` derives the tester-facing package from `evidence/current` and writes outside the runtime tree by default:

```text
~/delivery_package
```

The package layout is `README.md`, `SUMMARY.csv`, `plots/`, `route/`, and `flight_control/`. It keeps full sample timing for route and FC metric sections while simplifying columns for human review.

## Source Layers

1. UI / one-click flow / state machine
   - `catkin_ws/src/f250_maritime_uav_sim/scripts/f250_control_panel.py`
   - `catkin_ws/src/f250_maritime_uav_sim/scripts/f250_start_to_p0_hover.sh`
   - `catkin_ws/src/f250_maritime_uav_sim/scripts/f250_run_p0_p8_route.sh`
   - `catkin_ws/src/f250_maritime_uav_sim/scripts/f250_run_fc_3_10_steady_state.sh`
   - `catkin_ws/src/f250_maritime_uav_sim/scripts/f250_stop_all.sh`

2. P0-P8 route
   - `catkin_ws/src/f250_maritime_uav_sim/config/routes/classic_p0_p8.yaml`
   - `catkin_ws/src/f250_maritime_uav_sim/scripts/f250_prealign_yaw.py`
   - `evidence/current/route_p0_p8/inputs/params.json`
   - `evidence/current/route_p0_p8/inputs/route_waypoints.csv`

3. Scene geometry / map range / water / geo origin
   - `catkin_ws/src/f250_maritime_uav_sim/config/scenes/level_m_gps_assets_quick_complex.yaml`
   - `catkin_ws/src/f250_maritime_uav_sim/worlds/maritime_level_m_gps_assets_quick_complex.world`
   - `evidence/current/route_p0_p8/inputs/effective_scene.yaml`

4. Map authority rendering layer
   - `map_authority/p0p8_clean_scene/README.md`
   - `map_authority/p0p8_clean_scene/sources/layer_index.csv`
   - `map_authority/p0p8_clean_scene/sources/map_manifest.json`

5. EGO planner parameters
   - `evidence/current/route_p0_p8/inputs/params.json`
   - `catkin_ws/src/f250_maritime_uav_sim/launch/maritime_ego_planner.launch`
   - `catkin_ws/src/f250_maritime_uav_sim/launch/f250_ego_advanced_param_px4_native_pose.xml`
   - `catkin_ws/src/ego-planner`

6. LiDAR / Depth sensor parameters
   - `runtime_state/active_sensor.env`
   - `evidence/current/route_p0_p8/inputs/params.json`
   - `catkin_ws/src/f250_maritime_uav_sim/launch/maritime_obstacles.launch`
   - `catkin_ws/src/f250_maritime_uav_sim/models/maritime_mid360_lidar/model.sdf`
   - `catkin_ws/src/f250_maritime_uav_sim/scripts/maritime_laser_scan_adapter.py`
   - `catkin_ws/src/f250_maritime_uav_sim/scripts/maritime_sensor_cloud_adapter.py`

7. F250 / PX4 airframe and dynamics
   - `/home/adminpc/PX4-Autopilot-v1.16.0-src-main/ROMFS/px4fmu_common/init.d-posix/airframes/10020_gazebo-classic_f250`
   - `/home/adminpc/PX4-Autopilot-v1.16.0-src-main/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/f250/f250.sdf`
   - Current runtime SDF hash: `f57e4f06c849d8c7cd350399b8500ea3f47ca7d929e014acd5caaa99d7fcca98`

8. Visual model layer
   - Current scene mesh URIs: `catkin_ws/src/f250_maritime_uav_sim/config/scenes/level_m_gps_assets_quick_complex.yaml`
   - Generated world mirror: `catkin_ws/src/f250_maritime_uav_sim/worlds/maritime_level_m_gps_assets_quick_complex.world`
   - Map-authority mirror: `map_authority/p0p8_clean_scene/sources/scene.yaml`
   - Footprints used for map rendering: `map_authority/p0p8_clean_scene/sources/visual_mesh_footprints.csv`
   - Gazebo standalone model metadata: `catkin_ws/src/f250_maritime_uav_sim/models/*/model.sdf`
   - Active visual meshes use materialized/detailed variants where present. Retained visual model `model.sdf` files are aligned to those active variants for standalone Gazebo model loading. Original imported meshes may remain beside them in the same `meshes/` directory as source/reference assets; do not delete them by default and do not treat `model.sdf` alone as the current scene mesh source.

9. Collision / planning / evaluation layer
   - `map_authority/p0p8_clean_scene/sources/planner_obstacles.csv`
   - `evidence/current/route_p0_p8/metrics/acceptance_summary.json`
   - `catkin_ws/src/f250_maritime_uav_sim/scripts/maritime_metric_core.py`
   - `catkin_ws/src/f250_maritime_uav_sim/scripts/f250_route_human_summary.py`

10. Metrics calculation / steady-state / evidence publishing
    - `evidence/current/route_p0_p8/metrics/metric_summary.json`
    - `evidence/current/fc_3_10/metrics/summary.json`
    - `catkin_ws/src/f250_maritime_uav_sim/scripts/maritime_metric_core.py`
    - `catkin_ws/src/f250_maritime_uav_sim/scripts/maritime_metric_monitor.py`
    - `catkin_ws/src/f250_maritime_uav_sim/scripts/f250_fc_3_10_steady_state.py`
    - `maintenance/publish_current_review.py`
    - `maintenance/generate_current_index.py`
    - `maintenance/sanitize_current_evidence.py`

## Maintenance

- `f250_cleanup_runs.sh` removes temporary P0, route, flight-control, and stop-log outputs from `runtime_state/work/` after accepted evidence has been published.
- `maintenance/check_package.sh` rejects old `runs/` state, retired layouts, generated caches, bytecode, external tooling state, private config path references, and launcher/prealign default regressions. It also verifies the `evidence/current` whitelist layout.
