# AMR Mastery · <your name>

My submission for the **AMR Mastery Certificate of Completion** (MechaMind Labs).
Course: [ROS 2 Full Course | AMR Mastery](https://www.youtube.com/playlist?list=PLae-2ikc0DUY) · Course code: [MechaMind-Labs/AMR-Mastery](https://github.com/MechaMind-Labs/AMR-Mastery)

| | |
|---|---|
| Track | Simulation Track ☐ · Hardware Track ☐ |
| Capstone demo | <!-- unlisted YouTube link --> |
| Hardware demo | <!-- optional --> |
| Report | [docs/report.md](docs/report.md) |
| Environment | Ubuntu 22.04 · ROS 2 Humble · Gazebo Fortress |

## Summary
<!-- 3–5 sentences: what your robot does and what you changed. -->

## Assignments
| ID | Folder | Status |
|---|---|---|
| AS01 | [assignments/AS01_turtle_spiral](assignments/AS01_turtle_spiral) | ☐ |
<!-- one row per assignment, AS01 … AS22 -->

## Build and run
```bash
cd mechabot_ws && rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install && source install/setup.bash
```

## Credits
Based on the MechaMind Labs AMR Mastery course. Reused code is credited in each assignment README.
