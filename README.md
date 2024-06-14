# Mobile Robots For Critical Missions - fork of turtlebot4 repo: https://github.com/gdesimone97/turtlebot4

This is the forked repo of Turtlebot 4 by Group 9 composed of:
- Langella Antonio: a.langella31@studenti.unisa.it
- Marsico Michele: m.marsico10@studenti.unisa.it
- Paolino Salvatore: s.paolino6@studenti.unisa.it
- Trotta Prisco: p.trotta12@studenti.unisa.it

# Undock by cli
```bash
ros2 action send_goal /undock irobot_create_msgs/action/Undock {}
```
# Dock by cli
```bash
ros2 action send_goal /dock irobot_create_msgs/action/Dock {}
```
# Poweroff
```bash
ros2 service call /robot_power irobot_create_msgs/srv/RobotPower
```
# Teleop
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
# Rviz
```bash
ros2 launch turtlebot4_viz view_robot.launch.py
```
# SLAM
```bash
ros2 launch turtlebot4_navigation slam.launch.py
ros2 launch turtlebot4_viz view_robot.launch.py
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "name:
  data: 'test'"
```

# Navigation
```bash
ros2 launch turtlebot4_navigation localization.launch.py map:=test.yaml
ros2 launch turtlebot4_navigation nav2.launch.py
ros2 launch turtlebot4_viz view_robot.launch.py
```

# Simulation
```bash
ros2 launch diem_gazebo turtlebot4_ignition_no_dock.launch.py world:=square
```

# Navigation without door stops into diem map
```bash
ros2 launch turtlebot4_navigation localization.launch.py map:=diem_map_clean.yaml
ros2 launch turtlebot4_navigation nav2.launch.py
ros2 launch turtlebot4_viz view_robot.launch.py
```

