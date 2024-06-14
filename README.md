# Group 9's custom turtlebot4 repository

This is a fork of the [turtlebot4](https://github.com/gdesimone97/turtlebot4) repository. We forked it to add some custom functionalities to the original codebase, namely:
- We added a 3D world of the DIEM to simulate the robot's navigation in Gazebo;
- We added a custom launch file to launch the robot in Gazebo without the dock;
- We added a new 2D map of the DIEM without the door stops;

## Launch the simulation with the DIEM world
```bash
ros2 launch diem_gazebo turtlebot4_ignition_no_dock.launch.py rviz:=true nav2:=true localization:=true map:=./src/turtlebot4/diem_turtlebot_ws/src/map/diem_map.yaml world:=diem_map params_file:=src/mobile-robots-project-work/autonomous_navigation_challenge_2024/autonomous_navigation_challenge_2024/config/nav2.yaml
```

## Launch the localization stack with the new map
```bash
ros2 launch turtlebot4_navigation localization.launch.py map:=diem_map_clean.yaml
```