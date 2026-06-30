# F250 Maritime UAV Simulation

This repository packages the completed F250 maritime PX4/Gazebo/MAVROS/EGO-Planner simulation environment. It includes the quick-complex maritime scene, LiDAR/Depth perception switching, route validation, FC Metric 3.10 validation, current map/result plots, retained reference evidence, and the PX4-side F250 patch needed to reproduce the vehicle in PX4 v1.16.0.

Known compatible baseline:

- Ubuntu 20.04.x desktop environment
- ROS Noetic
- Gazebo Classic 11
- MAVROS for ROS Noetic
- PX4-Autopilot v1.16.0

This README is the GitHub reproduction tutorial. The internal `runtime/README.md` is only a concise guide for users after the external dependencies are installed.

## Repository Layout

```text
README.md                         full reproduction tutorial
apply_px4_f250_patch.sh           installs the F250 patch into a local PX4 tree
check_release_package.sh          checks this GitHub package layout
px4_f250_patch/                   F250 airframe and Gazebo model patch for PX4 v1.16.0
runtime/                          simulation system body
```

Inside `runtime/`:

```text
catkin_ws/src/f250_maritime_uav_sim/   ROS package, launch files, scripts, worlds, RViz, models
catkin_ws/src/ego-planner/             EGO-Planner source used by this simulation
maintenance/                           build, static check, evidence, plot, and delivery helpers
map_authority/p0p8_clean_scene/        map sources and current review plots
evidence/current/                      retained reference evidence for comparison
```

The repository intentionally does not include generated local outputs:

```text
runtime/catkin_ws/build/
runtime/catkin_ws/devel/
runtime/runtime_state/
delivery_package/
ROS logs, caches, desktop shortcut outputs, and agent scratch files
```

## 1. Prepare External Dependencies

Prepare Ubuntu 20.04.x yourself first. Then install ROS/Gazebo/MAVROS packages using the official ROS Noetic path for Ubuntu 20.04 and your normal package mirror.

Typical package set:

```bash
sudo apt update
sudo apt install -y git build-essential cmake python3-pip python3-rosdep python3-catkin-tools screen xterm
sudo apt install -y ros-noetic-desktop-full ros-noetic-mavros ros-noetic-mavros-extras
sudo apt install -y ros-noetic-pcl-ros ros-noetic-cv-bridge ros-noetic-image-transport
sudo apt install -y ros-noetic-dynamic-reconfigure ros-noetic-nodelet ros-noetic-laser-geometry ros-noetic-cmake-modules
```

Install MAVROS GeographicLib data using the method recommended by your MAVROS package. A common ROS Noetic command is:

```bash
sudo /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh
```

Python helpers used by maps, plots, and evidence tooling:

```bash
python3 -m pip install --user numpy pyyaml matplotlib pillow trimesh
```

## 2. Prepare PX4 v1.16.0

Clone PX4 separately. Do not put the full PX4 tree inside this repository.

```bash
git clone --recursive https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
git checkout v1.16.0
git submodule update --init --recursive
bash ./Tools/setup/ubuntu.sh --no-nuttx
make px4_sitl gazebo-classic
```

## 3. Clone This Repository And Check The Package

```bash
git clone git@github.com:cjx5d666/f250-maritime-uav-sim.git
cd f250-maritime-uav-sim

./check_release_package.sh
```

## 4. Apply The F250 PX4 Patch

```bash
./apply_px4_f250_patch.sh /path/to/PX4-Autopilot
```

The patch installs only the F250-specific PX4 additions:

```text
ROMFS/px4fmu_common/init.d-posix/airframes/10020_gazebo-classic_f250
Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/f250/
```

If PX4 was already built before applying the patch, rebuild PX4 SITL:

```bash
cd /path/to/PX4-Autopilot
make px4_sitl gazebo-classic
```

If PX4 is not under a common home-directory path, set this before running the simulation:

```bash
export F250_PX4_ROOT=/path/to/PX4-Autopilot
```

## 5. Build The Simulation Runtime

```bash
cd /path/to/f250-maritime-uav-sim/runtime
./maintenance/build_catkin_ws.sh
```

This creates local generated folders:

```text
runtime/catkin_ws/build/
runtime/catkin_ws/devel/
```

They are build products and should not be committed.

## 6. Run The Simulation

The normal operator entry is the control panel:

```bash
cd /path/to/f250-maritime-uav-sim/runtime
./catkin_ws/src/f250_maritime_uav_sim/scripts/f250_open_control_panel.sh
```

Optional desktop launcher:

```bash
./catkin_ws/src/f250_maritime_uav_sim/scripts/f250_install_desktop_shortcut.sh
```

Command-line equivalents:

```bash
# LiDAR P0 hover with Gazebo GUI + RViz + PX4 GUI
./catkin_ws/src/f250_maritime_uav_sim/scripts/f250_start_to_p0_hover.sh --sensor lidar

# Depth P0 hover
./catkin_ws/src/f250_maritime_uav_sim/scripts/f250_start_to_p0_hover.sh --sensor depth

# Run accepted P0-P8 route after P0 is ready
./catkin_ws/src/f250_maritime_uav_sim/scripts/f250_run_p0_p8_route.sh

# Run FC Metric 3.10 from a fresh P0 hover
./catkin_ws/src/f250_maritime_uav_sim/scripts/f250_run_fc_3_10_steady_state.sh

# Stop runtime processes
./catkin_ws/src/f250_maritime_uav_sim/scripts/f250_stop_all.sh
```

Default startup is LiDAR. Depth is selected explicitly through the control panel or `--sensor depth`.

## 7. Validate A Reproduction

Minimum checks:

```bash
cd /path/to/f250-maritime-uav-sim
./check_release_package.sh
cd runtime
./maintenance/check_package.sh
./maintenance/build_catkin_ws.sh --dry-run
```

Runtime smoke validation:

1. Start LiDAR P0 and wait for READY.
2. Run the P0-P8 Route.
3. Confirm `map_authority/p0p8_clean_scene/route_result.png` refreshes after successful evidence publish.
4. Stop all runtime processes.
5. Start Depth P0 and confirm the Depth perception chain reaches READY.

Successful route/FC validation publishes machine evidence into:

```text
runtime/evidence/current/
```

Successful route evidence publish refreshes:

```text
runtime/map_authority/p0p8_clean_scene/route_result.png
```

It does not rebuild the external delivery package. To build that package explicitly:

```bash
cd runtime
./maintenance/build_delivery_package.py
```

Default output:

```text
~/delivery_package
```

## 8. Generated Files And Cleanup

Runtime scripts create this local state automatically:

```text
runtime/runtime_state/
  active_sensor.env
  active_task.env
  work/
```

Safe generated outputs to remove after stopping the simulation:

```text
runtime/catkin_ws/build/
runtime/catkin_ws/devel/
runtime/runtime_state/
~/delivery_package
~/.ros/log/
```

Rebuild catkin after deleting `build/` or `devel/`.

## 9. Troubleshooting

PX4 root not found:

```bash
export F250_PX4_ROOT=/path/to/PX4-Autopilot
```

F250 vehicle/model not found:

```bash
./apply_px4_f250_patch.sh /path/to/PX4-Autopilot
cd /path/to/PX4-Autopilot
make px4_sitl gazebo-classic
```

Catkin cannot find ROS packages:

```bash
source /opt/ros/noetic/setup.bash
cd runtime
./maintenance/build_catkin_ws.sh
```

Gazebo spawn or sensor startup issue:

```bash
cd runtime
./catkin_ws/src/f250_maritime_uav_sim/scripts/f250_stop_all.sh
./catkin_ws/src/f250_maritime_uav_sim/scripts/f250_start_to_p0_hover.sh --sensor lidar
```

## Source Notes

EGO-Planner is included as source under `runtime/catkin_ws/src/ego-planner` to make this reproduction package version-stable. The public build ignores EGO's `local_sensing` package because this maritime LiDAR/Depth pipeline does not use it and it pulls unrelated SVO-only dependencies.

The full PX4 source tree is not vendored. Only the F250 PX4 patch files are included under `px4_f250_patch/`.
