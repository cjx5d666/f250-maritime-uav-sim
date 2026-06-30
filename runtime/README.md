# F250 Maritime Runtime

This directory is the simulation runtime body. It assumes the external environment is already prepared: ROS Noetic, Gazebo Classic, MAVROS, PX4 v1.16.0, and the repository's F250 PX4 patch.

## Main Entrypoints

```bash
./catkin_ws/src/f250_maritime_uav_sim/scripts/f250_open_control_panel.sh
./catkin_ws/src/f250_maritime_uav_sim/scripts/f250_start_to_p0_hover.sh --sensor lidar
./catkin_ws/src/f250_maritime_uav_sim/scripts/f250_start_to_p0_hover.sh --sensor depth
./catkin_ws/src/f250_maritime_uav_sim/scripts/f250_run_p0_p8_route.sh
./catkin_ws/src/f250_maritime_uav_sim/scripts/f250_run_fc_3_10_steady_state.sh
./catkin_ws/src/f250_maritime_uav_sim/scripts/f250_stop_all.sh
```

Optional desktop shortcut:

```bash
./catkin_ws/src/f250_maritime_uav_sim/scripts/f250_install_desktop_shortcut.sh
```

## Runtime Layout

```text
catkin_ws/src/f250_maritime_uav_sim/   ROS package source, launch files, worlds, RViz, models, scripts
catkin_ws/src/ego-planner/             EGO-Planner source used by this runtime
maintenance/                           build/check/evidence/plot/delivery helpers
map_authority/p0p8_clean_scene/        map sources and current plan/result plots
evidence/current/                      retained machine-readable reference evidence
```

Generated local state is created when scripts run:

```text
runtime_state/
  active_sensor.env
  active_task.env
  work/
```

Generated catkin outputs are local only:

```text
catkin_ws/build/
catkin_ws/devel/
```

## Checks

```bash
./maintenance/check_package.sh
./maintenance/build_catkin_ws.sh --dry-run
./maintenance/build_catkin_ws.sh
```

Successful evidence publishes refresh `map_authority/p0p8_clean_scene/route_result.png`. They do not rebuild `~/delivery_package`; run `./maintenance/build_delivery_package.py` only when that external review package is needed.
