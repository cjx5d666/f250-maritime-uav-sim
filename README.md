# F250 Maritime UAV Simulation

A complete PX4/Gazebo/MAVROS/EGO-Planner simulation setup for an F250 UAV navigating a cluttered maritime environment. Use this if you want to reproduce the same environment, inspect route planning results, or experiment with the LiDAR/Depth perception switching system.

This package includes the simulation runtime, F250-specific PX4 additions, map data, result plots, and validation references. PX4 itself is installed separately—you'll clone PX4 v1.16.0 and apply the F250 patch provided here.

Tested on:

- Ubuntu 20.04.x desktop
- ROS Noetic
- Gazebo Classic 11
- MAVROS for ROS Noetic
- PX4-Autopilot v1.16.0

Everything has been verified from a fresh clone: LiDAR startup, the P0-P8 route, Depth startup, PX4 patch installation, and the catkin build process.

## What's Inside

```text
README.md                         reproduction guide
apply_px4_f250_patch.sh           installs F250 additions into your PX4 tree
check_release_package.sh          verifies package structure
px4_f250_patch/                   F250 PX4 airframe and Gazebo model files
runtime/                          ROS/Gazebo/EGO simulation runtime
```

Inside `runtime/`:

```text
catkin_ws/src/f250_maritime_uav_sim/   ROS package with launch files, scripts, worlds, RViz configs, models
catkin_ws/src/ego-planner/             EGO-Planner source
maintenance/                           build, check, evidence, plot, and delivery scripts
map_authority/p0p8_clean_scene/        map sources and route/result plots
evidence/current/                      validation reference data
```

Build outputs and generated files are excluded:

```text
runtime/catkin_ws/build/
runtime/catkin_ws/devel/
runtime/runtime_state/
delivery_package/
ROS logs, caches, and temporary run folders
```

## 1. Install External Dependencies

Start with Ubuntu 20.04.x. Install ROS Noetic, Gazebo Classic, MAVROS, and common ROS packages.

Typical installation:

```bash
sudo apt update
sudo apt install -y git build-essential cmake python3-pip python3-rosdep python3-catkin-tools screen xterm
sudo apt install -y ros-noetic-desktop-full ros-noetic-mavros ros-noetic-mavros-extras
sudo apt install -y ros-noetic-pcl-ros ros-noetic-cv-bridge ros-noetic-image-transport
sudo apt install -y ros-noetic-dynamic-reconfigure ros-noetic-nodelet ros-noetic-laser-geometry ros-noetic-cmake-modules
```

Install MAVROS GeographicLib datasets:

```bash
sudo /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh
```

Install Python packages for map processing, plotting, and validation:

```bash
python3 -m pip install --user numpy pyyaml matplotlib pillow trimesh
```

## 2. Install PX4 v1.16.0

Clone PX4 separately—don't put it inside this repository.

```bash
git clone --recursive https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
git checkout v1.16.0
git submodule update --init --recursive
bash ./Tools/setup/ubuntu.sh --no-nuttx
make px4_sitl gazebo-classic
```

## 3. Clone This Repository

HTTPS clone:

```bash
git clone https://github.com/cjx5d666/f250-maritime-uav-sim.git
cd f250-maritime-uav-sim
./check_release_package.sh
```

SSH also works if you have GitHub keys configured:

```bash
git clone git@github.com:cjx5d666/f250-maritime-uav-sim.git
```

## 4. Apply the F250 PX4 Patch

This repository provides only the F250-specific additions. Run the patch script to copy them into your PX4 v1.16.0 installation.

```bash
./apply_px4_f250_patch.sh /path/to/PX4-Autopilot
```

It installs:

```text
ROMFS/px4fmu_common/init.d-posix/airframes/10020_gazebo-classic_f250
Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/f250/
```

If you already built PX4 before applying the patch, rebuild it:

```bash
cd /path/to/PX4-Autopilot
make px4_sitl gazebo-classic
```

If your PX4 folder isn't in a standard location, export its path before running the simulation:

```bash
export F250_PX4_ROOT=/path/to/PX4-Autopilot
```

## 5. Build the Runtime

The runtime is a ROS catkin workspace. Building it creates the `build/` and `devel/` folders that ROS needs.

```bash
cd /path/to/f250-maritime-uav-sim/runtime
./maintenance/build_catkin_ws.sh
```

These build folders are local outputs and shouldn't be committed:

```text
runtime/catkin_ws/build/
runtime/catkin_ws/devel/
```

## 6. Run the Simulation

The easiest way to start is the control panel:

```bash
cd /path/to/f250-maritime-uav-sim/runtime
./catkin_ws/src/f250_maritime_uav_sim/scripts/f250_open_control_panel.sh
```

Optionally install a desktop shortcut:

```bash
./catkin_ws/src/f250_maritime_uav_sim/scripts/f250_install_desktop_shortcut.sh
```

Command-line options:

```bash
# Start LiDAR P0 hover with Gazebo GUI, RViz, and PX4 GUI
./catkin_ws/src/f250_maritime_uav_sim/scripts/f250_start_to_p0_hover.sh --sensor lidar

# Start Depth P0 hover
./catkin_ws/src/f250_maritime_uav_sim/scripts/f250_start_to_p0_hover.sh --sensor depth

# Run the P0-P8 route after P0 is ready
./catkin_ws/src/f250_maritime_uav_sim/scripts/f250_run_p0_p8_route.sh

# Run FC Metric 3.10 from a fresh P0 hover
./catkin_ws/src/f250_maritime_uav_sim/scripts/f250_run_fc_3_10_steady_state.sh

# Stop all runtime processes
./catkin_ws/src/f250_maritime_uav_sim/scripts/f250_stop_all.sh
```

LiDAR is the default. Use `--sensor depth` or select it in the control panel to switch to Depth mode.

## 7. Verify Your Setup

Start with static checks:

```bash
cd /path/to/f250-maritime-uav-sim
./check_release_package.sh
cd runtime
./maintenance/check_package.sh
./maintenance/build_catkin_ws.sh --dry-run
```

Then run a quick smoke test:

1. Start LiDAR P0 and wait until ready.
2. Run the P0-P8 route.
3. Check that `runtime/map_authority/p0p8_clean_scene/route_result.png` updated.
4. Stop all processes.
5. Start Depth P0 and confirm the Depth perception chain works.

Successful route and FC runs write validation data to:

```text
runtime/evidence/current/
```

Successful route runs update:

```text
runtime/map_authority/p0p8_clean_scene/route_result.png
```

The external delivery package isn't automatically updated. Build it manually when needed:

```bash
cd runtime
./maintenance/build_delivery_package.py
```

Default output location:

```text
~/delivery_package
```

## 8. Generated Files and Cleanup

Runtime scripts create local state automatically:

```text
runtime/runtime_state/
  active_sensor.env
  active_task.env
  work/
```

Safe to remove after stopping the simulation:

```text
runtime/catkin_ws/build/
runtime/catkin_ws/devel/
runtime/runtime_state/
~/delivery_package
~/.ros/log/
```

If you delete `build/` or `devel/`, rebuild before launching:

```bash
cd runtime
./maintenance/build_catkin_ws.sh
```

## 9. Troubleshooting

PX4 not found:

```bash
export F250_PX4_ROOT=/path/to/PX4-Autopilot
```

F250 vehicle or model not found:

```bash
./apply_px4_f250_patch.sh /path/to/PX4-Autopilot
cd /path/to/PX4-Autopilot
make px4_sitl gazebo-classic
```

Catkin can't find ROS packages:

```bash
source /opt/ros/noetic/setup.bash
cd runtime
./maintenance/build_catkin_ws.sh
```

Old processes still running:

```bash
cd runtime
./catkin_ws/src/f250_maritime_uav_sim/scripts/f250_stop_all.sh
```

## 中文说明

这是一个完整的 F250 海事场景无人机仿真环境，基于 PX4 v1.16.0、Gazebo Classic、MAVROS、ROS Noetic 和 EGO-Planner 构建，支持在复杂海事环境中进行航线规划，提供 LiDAR 和 Depth 两种感知模式。

仓库包含两部分：

- `runtime/`：仿真系统，包括 ROS 包、场景、模型、RViz 配置、航线脚本、验证脚本和结果图。
- `px4_f250_patch/`：F250 机型和 Gazebo 模型的 PX4 补丁文件。

PX4 本体需要单独安装。你需要先安装 PX4 v1.16.0，然后用本仓库的脚本应用 F250 补丁。

### 1. 准备系统环境

需要 Ubuntu 20.04.x 桌面系统，然后安装 ROS Noetic、Gazebo Classic、MAVROS 和依赖包：

```bash
sudo apt update
sudo apt install -y git build-essential cmake python3-pip python3-rosdep python3-catkin-tools screen xterm
sudo apt install -y ros-noetic-desktop-full ros-noetic-mavros ros-noetic-mavros-extras
sudo apt install -y ros-noetic-pcl-ros ros-noetic-cv-bridge ros-noetic-image-transport
sudo apt install -y ros-noetic-dynamic-reconfigure ros-noetic-nodelet ros-noetic-laser-geometry ros-noetic-cmake-modules
```

安装 MAVROS GeographicLib 数据：

```bash
sudo /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh
```

安装地图处理和验证脚本需要的 Python 包：

```bash
python3 -m pip install --user numpy pyyaml matplotlib pillow trimesh
```

### 2. 安装 PX4 v1.16.0

PX4 需要单独放置，不要放在本仓库目录内：

```bash
git clone --recursive https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
git checkout v1.16.0
git submodule update --init --recursive
bash ./Tools/setup/ubuntu.sh --no-nuttx
make px4_sitl gazebo-classic
```

### 3. 下载本仓库

HTTPS 克隆：

```bash
git clone https://github.com/cjx5d666/f250-maritime-uav-sim.git
cd f250-maritime-uav-sim
./check_release_package.sh
```

SSH 克隆（需要配置 GitHub 密钥）：

```bash
git clone git@github.com:cjx5d666/f250-maritime-uav-sim.git
```

### 4. 应用 F250 补丁到 PX4

在本仓库根目录执行：

```bash
./apply_px4_f250_patch.sh /path/to/PX4-Autopilot
```

把 `/path/to/PX4-Autopilot` 替换成你的 PX4 安装路径。

脚本会将 F250 机型配置和 Gazebo 模型复制到 PX4。复制后需要重新编译 PX4：

```bash
cd /path/to/PX4-Autopilot
make px4_sitl gazebo-classic
```

如果 PX4 不在标准路径，启动仿真前需要设置环境变量：

```bash
export F250_PX4_ROOT=/path/to/PX4-Autopilot
```

### 5. 编译运行环境

`runtime/` 是一个 ROS catkin 工作区。编译会生成 `build/` 和 `devel/` 目录，这些是本地生成文件，不需要提交到 Git。

```bash
cd /path/to/f250-maritime-uav-sim/runtime
./maintenance/build_catkin_ws.sh
```

### 6. 启动仿真

推荐使用控制面板：

```bash
cd /path/to/f250-maritime-uav-sim/runtime
./catkin_ws/src/f250_maritime_uav_sim/scripts/f250_open_control_panel.sh
```

可选安装桌面快捷方式：

```bash
./catkin_ws/src/f250_maritime_uav_sim/scripts/f250_install_desktop_shortcut.sh
```

命令行启动方式：

```bash
# LiDAR 模式启动到 P0 悬停（打开 Gazebo、RViz 和 PX4 GUI）
./catkin_ws/src/f250_maritime_uav_sim/scripts/f250_start_to_p0_hover.sh --sensor lidar

# Depth 模式启动到 P0 悬停
./catkin_ws/src/f250_maritime_uav_sim/scripts/f250_start_to_p0_hover.sh --sensor depth

# P0 就绪后运行 P0-P8 航线
./catkin_ws/src/f250_maritime_uav_sim/scripts/f250_run_p0_p8_route.sh

# 从 P0 悬停运行 FC Metric 3.10
./catkin_ws/src/f250_maritime_uav_sim/scripts/f250_run_fc_3_10_steady_state.sh

# 停止所有仿真进程
./catkin_ws/src/f250_maritime_uav_sim/scripts/f250_stop_all.sh
```

默认使用 LiDAR。切换到 Depth 需在控制面板选择或使用 `--sensor depth` 参数。

### 7. 验证复现结果

先执行静态检查：

```bash
cd /path/to/f250-maritime-uav-sim
./check_release_package.sh
cd runtime
./maintenance/check_package.sh
./maintenance/build_catkin_ws.sh --dry-run
```

然后进行实际测试：

1. 启动 LiDAR P0，等待系统就绪。
2. 运行 P0-P8 航线。
3. 检查 `runtime/map_authority/p0p8_clean_scene/route_result.png` 是否更新。
4. 停止所有进程。
5. 启动 Depth P0，确认 Depth 感知链正常工作。

航线和 FC 验证成功后，结果数据会写入：

```text
runtime/evidence/current/
```

航线结果图会更新到：

```text
runtime/map_authority/p0p8_clean_scene/route_result.png
```

交付包 `~/delivery_package` 不会自动生成。如需单独生成，手动执行：

```bash
cd runtime
./maintenance/build_delivery_package.py
```

### 8. 自动生成的文件

运行仿真会自动生成：

```text
runtime/runtime_state/
runtime/catkin_ws/build/
runtime/catkin_ws/devel/
~/.ros/log/
```

这些是本地运行产生的文件，不属于源码。停止仿真后可以清理。如果删除了 `build/` 或 `devel/`，下次运行前重新编译即可：

```bash
cd runtime
./maintenance/build_catkin_ws.sh
```

### 9. 常见问题

找不到 PX4：

```bash
export F250_PX4_ROOT=/path/to/PX4-Autopilot
```

找不到 F250 机型或 Gazebo 模型：

```bash
./apply_px4_f250_patch.sh /path/to/PX4-Autopilot
cd /path/to/PX4-Autopilot
make px4_sitl gazebo-classic
```

catkin 编译找不到 ROS 包：

```bash
source /opt/ros/noetic/setup.bash
cd runtime
./maintenance/build_catkin_ws.sh
```

仿真进程未完全停止：

```bash
cd runtime
./catkin_ws/src/f250_maritime_uav_sim/scripts/f250_stop_all.sh
```