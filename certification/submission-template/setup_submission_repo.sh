#!/usr/bin/env bash
# Creates the AMR Mastery submission repo structure in the current folder.
# Usage (inside your new, empty Git repo):
#   bash /path/to/AMR-Mastery/certification/submission-template/setup_submission_repo.sh
set -euo pipefail
T="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

mkdir -p cad/exploded_view cad/docking_station capstone/worlds capstone/maps docs media mechabot_ws/src
[ -f README.md ]      || cp "$T/README_TEMPLATE.md" README.md
[ -f docs/report.md ] || cp "$T/REPORT_TEMPLATE.md" docs/report.md
[ -f .gitignore ]     || cp "$T/gitignore.txt" .gitignore
[ -f capstone/README.md ] || cp "$T/CAPSTONE_TEMPLATE.md" capstone/README.md
touch media/.gitkeep capstone/worlds/.gitkeep capstone/maps/.gitkeep mechabot_ws/src/.gitkeep \
      cad/exploded_view/.gitkeep cad/docking_station/.gitkeep

while IFS='|' read -r id lesson title; do
  [ -z "$id" ] && continue
  d="assignments/${id}"
  mkdir -p "$d"
  esc="${title//&/\\&}"
  [ -f "$d/README.md" ] || sed -e "s/{{ID}}/${id%%_*}/" -e "s/{{LESSON}}/${lesson}/" -e "s/{{TITLE}}/${esc}/" \
      "$T/ASSIGNMENT_TEMPLATE.md" > "$d/README.md"
done <<'LIST'
AS01_turtle_spiral|L06|Turtle Spiral Drawer – ROS 2 Motion Control
AS02_turtle_spawner|L09|Random Turtle Spawner & Auto Destroyer – ROS 2 Services
AS03_dont_follow_me|L09|Don't Follow Me – Turtle Tracking & Dynamic Pen Color
AS04_catch_them_all|L09|Catch Them All – Autonomous Turtle Hunter
AS05_polygon_drawer|L08|Turtlesim Polygon Drawer – Custom Shape Generator
AS06_exploded_view|L15|MechaBot Exploded Assembly View – CAD Visualization
AS07_docking_station|L15|Custom MechaBot Docking Station – CAD Design
AS08_urdf_rviz|L20|MechaBot URDF Visualization in RViz2
AS09_obstacle_avoidance|L24|MechaBot Obstacle Avoidance in Gazebo
AS10_wall_following|L24|MechaBot Wall Following in Gazebo
AS11_qr_maze_solver|L27|QR Code Guided Maze Solver
AS12_qr_maze_navigator|L27|QR Maze Navigator – Enter, Solve & Exit
AS13_polygon_path|L25|MechaBot Polygon Path Drawer in Gazebo
AS14_slam_mapping|L28|MechaBot Warehouse Mapping using SLAM Toolbox
AS15_map_saving|L28|MechaBot Map Saving & Verification
AS16_nav2_navigation|L30|MechaBot Localization & Autonomous Navigation using Nav2
AS17_patrol_docking|L33|Autonomous Warehouse Patrol & Auto-Docking Mission
AS18_hw_bringup|L38|(Hardware track) Hardware Bring-Up & Wireless Teleoperation
AS19_hw_mapping|L38|(Hardware track) Real-World Mapping using SLAM Toolbox
AS20_hw_navigation|L39|(Hardware track) Real-World Autonomous Navigation using Nav2
AS21_hw_patrol_docking|L39|(Hardware track) Autonomous Patrol, Docking & Undocking
AS22_github|L40|Publish Your MechaBot Project to GitHub
LIST

echo "Done. Next: fill README.md, commit, and keep committing as you work."
echo "Simulation-track learners can delete AS18–AS21."
