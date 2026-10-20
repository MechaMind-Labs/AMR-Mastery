# MechaBot — Autonomous Mobile Robot with Vision Docking (ROS 2)

[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-22314E?logo=ros)](https://docs.ros.org/en/humble/)
[![Nav2](https://img.shields.io/badge/Nav2-SLAM%20%2B%20navigation-1F6FEB)](https://docs.nav2.org/)
[![ESP32](https://img.shields.io/badge/MCU-ESP32-E7352C)](https://www.espressif.com/en/products/socs/esp32)
[![Real hardware](https://img.shields.io/badge/runs%20on-real%20hardware-2EA44F)](#demo)

**MechaBot** is a two-wheel differential-drive robot that maps a room, localizes, patrols waypoints, watches its battery, and returns to its charging dock by itself — finding the dock with a QR marker and stopping on LiDAR. It is the reference robot of the MechaMind Labs **AMR Mastery** program, and this repository holds the program's code in the order it was taught, from the first ROS 2 node to the physical robot.

Designed and built by [Kumar Utkarsh](https://github.com/Curious-Utkarsh): chassis CAD, URDF, ESP32 firmware, the C++ `ros2_control` interface, the Nav2 setup and the docking logic. Learners in two cohorts rebuilt it in simulation and on their own hardware.

## Demo

<table>
<tr>
<td align="center"><a href="https://www.youtube.com/shorts/rBTisGu6iEY"><img src="https://img.youtube.com/vi/rBTisGu6iEY/hqdefault.jpg" width="300" alt="Meet MechaBot"></a><br><sub>Meet MechaBot — self-docking AMR</sub></td>
<td align="center"><a href="https://www.youtube.com/shorts/oXZ9rYnfgRw"><img src="https://img.youtube.com/vi/oXZ9rYnfgRw/hqdefault.jpg" width="300" alt="How MechaBot was built"></a><br><sub>How the robot was built</sub></td>
</tr>
</table>

- [MechaBot running on real hardware (LinkedIn, April 2026)](https://www.linkedin.com/posts/kumar-utkarsh-124aab227_robotics-ros2-engineering-activity-7447948478909169665-3SPB) · [from design to a real robot (LinkedIn)](https://www.linkedin.com/feed/update/urn:li:activity:7450153133218910208/) · Instagram reels: [1](https://www.instagram.com/reel/DbIE1rgiRIr/) · [2](https://www.instagram.com/reel/DbX73znCI5l/)
- Design: [custom AMR design pipeline](https://www.youtube.com/shorts/jlUMFFk9gu0) · [designing the DC motor bracket in Fusion 360](https://www.youtube.com/shorts/LaVOZSdebRM)
- Program: [Day 0 → Day 45 build diary (playlist)](https://www.youtube.com/playlist?list=PLdMuvhY3uTKvNg6r5yPQSBe_NIYhc0Hwn) · [ROS 2 URDF tutorial](https://youtu.be/Lgmdcwq2P3M) · [ROS 2 foundations capstone: polygon drawing in TurtleSim](https://www.youtube.com/shorts/QHqWY9Gi4mw)

## Key features

- **Motor control** — ESP32 firmware with per-wheel PID, interrupt-driven quadrature encoders and an L298N driver; 500 kbaud link to ROS.
- **C++ `ros2_control` hardware interface** — the same DiffDriveController (100 Hz) drives Gazebo and the real robot.
- **Mapping, localization, navigation** — slam_toolbox, AMCL, Nav2 (Smac Planner 2D + Regulated Pure Pursuit) with replanning and recovery, waypoint following via the Simple Commander API.
- **Autonomous docking** — Nav2 to a pre-dock pose → proportional alignment on the QR code's pixel offset → forward until LiDAR reads < 0.15 m.
- **Battery-aware patrol** — cancels the patrol at ≤ 5 % on `/battery_status` and returns to dock, in a multi-threaded executor.
- **Also included** — QR maze solver (QR + IMU turns + LiDAR), obstacle avoidance, Gazebo worlds (QR maze built in Blender, furnished house), Dockerfile.

## Hardware

| | |
|---|---|
| Compute | Raspberry Pi (ROS 2, SLAM, Nav2) |
| Motion | ESP32, 2 × encoder DC gear motors, L298N |
| Sensing | SLAMTEC RPLiDAR, MPU6050 IMU, camera, HC-SR04 |
| Body | 3D-printed chassis, top lid and dock designed in Fusion 360; 16×2 LCD for status |

## Software stack

ROS 2 Humble · ros2_control · Nav2 · slam_toolbox · AMCL · Gazebo Fortress · OpenCV · rplidar_ros · Docker · C++ · Python

## Repository layout

```
ROS2_Foundation/turtle_ws/     ROS 2 basics: nodes, topics, parameters, custom msgs/srvs
mechabot_ws/src/
  mechabot_description/        URDF from CAD, sensors, Gazebo worlds, dock mesh
  mechabot_firmware/           ESP32 sketches, C++ hardware interface, sensor drivers
  mechabot_controller/         diff-drive controller, twist_mux, joystick
  mechabot_mapping/ _localization/ _navigation/   SLAM, AMCL, Nav2
  mechabot_scripts/            docking, patrol, maze solver, obstacle avoidance
  mechabot_bringup/            simulated_robot / real_robot launch
```

## Run

```bash
git clone --recursive https://github.com/MechaMind-Labs/AMR-Mastery.git
cd AMR-Mastery/mechabot_ws && rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install && source install/setup.bash

ros2 launch mechabot_bringup simulated_robot.launch.py          # Gazebo + AMCL + Nav2 + RViz
ros2 run mechabot_scripts auto_docking_with_battery             # patrol, then return and dock
ros2 topic pub --once /battery_status std_msgs/msg/Int32 "{data: 5}"   # simulate low battery
```

On the robot: flash `mechabot_firmware/firmware/mechabot_control_firmware.ino` to the ESP32, then `ros2 launch mechabot_bringup real_robot.launch.py`. A Dockerfile is in `mechabot_ws/src/`.

## Related

- [Mobi](https://github.com/Curious-Utkarsh/mobi-service-robot) — the larger service robot built before MechaBot
- [TurtleBot3 ArUco docking](https://github.com/Curious-Utkarsh/turtlebot3-aruco-docking) — the first version of this docking idea
- [Edge Avoiding Robot](https://github.com/MechaMind-Labs/edge-avoiding-robot) · [TurtleSim basics](https://github.com/MechaMind-Labs/ros2-basics-with-turtlesim) — earlier MechaMind Labs workshops
- Program details: [mechamindlabs.com](https://www.mechamindlabs.com/)

---
[MechaMind Labs](https://www.mechamindlabs.com/) · [YouTube](https://www.youtube.com/@MechaMindLabs) · [LinkedIn](https://www.linkedin.com/company/mechamindlabs/)
