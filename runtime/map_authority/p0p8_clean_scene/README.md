# P0-P8 Clean Scene Map Authority

这个目录是地图权威包。`evidence/current` 保存当前 accepted 机器证据；地图图件和最终结果图只放在这里。

## 怎么重新生成

- `python3 render_map.py --target base`：只画底图。
- `python3 render_map.py --target obstacle`：画底图和浮标等人看地图需要显示的障碍。
- `python3 render_map.py --target route`：画底图、障碍和规划路线。
- `python3 render_map.py --target result`：再叠加 `evidence/current/route_p0_p8/measurements/actual_trajectory.csv` 里的实际飞行轨迹。
- `python3 render_map.py --target all`：生成全部四张图。

## 文件说明

- `render_map.py`：唯一画图入口，同一套图层画底图、障碍图、路线图和结果图。
- `README.md`：说明这个地图包怎么用、每个文件是什么。
- `sources/scene.yaml`：当前真实运行场景定义，记录船、岛、桥、三台风机、浮标、D1 动态小船和障碍配置。
- `sources/route_waypoints.csv`：规划路线点，记录 P0 到 P8 的位置、朝向、半径和停留时间。
- `sources/visual_mesh_footprints.csv`：视觉物体占地范围，告诉画图脚本船、桥、岛、风机在哪里、占多大。
- `sources/planner_obstacles.csv`：规划和安全检查看到的障碍边界，不等同于人眼看到的船外观。
- `sources/layer_index.csv`：每张地图图件用到了哪些数据层。
- `sources/map_manifest.json`：地图包总索引，记录地图范围、来源和输出文件。
- `base_world.png`：第一层图，只画水面、岛、船、桥、风机等视觉场景。
- `obstacle_map.png`：第二层图，在底图上加当前浮标障碍。
- `route_map.png`：第三层图，在障碍图上加规划路线 P0-P8。
- `route_result.png`：第四层图，在路线图上加实际飞行轨迹；轨迹数据从 `evidence/current` 读取，不复制到本目录。

## 图层规则

- 共同场景层：水面、视觉物体和 D1/W1-W3 等标号。
- 共同障碍层：人看地图需要显示的浮标障碍；D1 碰撞盒保留在数据里但不画到 PNG。
- 共同路线层：P0-P8 规划路线。
- 结果图只额外叠加实际飞行轨迹。
- 不手工改 PNG；需要变化时改来源数据后重新运行 `render_map.py`。
