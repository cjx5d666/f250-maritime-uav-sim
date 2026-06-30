cmake_minimum_required(VERSION 3.0.2)
project(f250_maritime_uav_sim)

find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  geometry_msgs
  quadrotor_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
  tf2_ros
  visualization_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/gazebo_truth_to_mavros_vision.py
  scripts/generate_maritime_world.py
  scripts/mavros_odom_to_tf.py
  scripts/maritime_sensor_cloud_adapter.py
  scripts/maritime_laser_scan_adapter.py
  scripts/maritime_lidar_follow_odom.py
  scripts/maritime_scene_markers.py
  scripts/maritime_inflated_obstacle_markers.py
  scripts/maritime_metric_monitor.py
  scripts/maritime_dynamic_obstacles.py
  scripts/maritime_goal_sequence.py
  scripts/maritime_flight_path.py
  scripts/position_cmd_to_mavros_setpoint.py
  scripts/f250_fc_3_10_steady_state.py
  scripts/f250_check_perception_gate.py
  scripts/f250_control_panel.py
  scripts/f250_route_profiles.py
  scripts/f250_route_human_summary.py
  scripts/f250_layout_windows.py
  scripts/f250_quick_complex_record.py
  scripts/f250_prealign_yaw.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/start_maritime_sim.sh
  scripts/f250_start_to_p0_hover.sh
  scripts/f250_run_p0_p8_route.sh
  scripts/f250_run_fc_3_10_steady_state.sh
  scripts/f250_open_control_panel.sh
  scripts/f250_generate_plan_plot.sh
  scripts/f250_generate_result_plot.sh
  scripts/f250_stop_all.sh
  scripts/f250_cleanup_runs.sh
  scripts/f250_install_desktop_shortcut.sh
  scripts/start_demo_waypoints.sh
  scripts/f250_paths.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  scripts/maritime_scene_utils.py
  scripts/maritime_clearance.py
  scripts/maritime_metric_core.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  config
  launch
  models
  rviz
  worlds
  resources
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
