# ROS 2 - Workspace & Package Development

A comprehensive guide to creating ROS 2 workspaces and developing Python packages with practical examples.

## 📋 Overview

This tutorial covers:
- ROS 2 workspace creation and structure
- Python package development
- Node creation with parameters
- Turtlesim control examples
- VS Code setup for ROS 2 development

---

## 🚀 Quick Start

### 1. Create ROS 2 Workspace

```bash
# Create workspace directory structure
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Build the workspace
colcon build

# Source the workspace
source install/setup.bash
```

### 2. Create Python Package

```bash
# Navigate to src directory
cd ~/ros2_ws/src

# Create a new Python package with dependencies
ros2 pkg create turtle_py --build-type ament_python --dependencies rclpy geometry_msgs

# Navigate back to workspace root
cd ~/ros2_ws
```

---

## 🛠️ VS Code Setup

### Install VS Code

```bash
# Download VS Code for Linux
wget https://code.visualstudio.com/sha/download?build=stable&os=linux-deb-x64 -O code.deb

# Install the package
sudo dpkg -i code.deb

# Fix any dependency issues (if needed)
sudo apt-get install -f

# Launch VS Code in current directory
code .
```

### Recommended Extensions

Install these extensions from VS Code marketplace:
- ROS (Microsoft)
- Python (Microsoft)
- CMake (twxs)
- XML Tools
- URDF Previewer
- C/C++ (Microsoft)

---

## 📦 Building Packages

### Build Specific Package

```bash
cd ~/ros2_ws
colcon build --packages-select turtle_py
source install/setup.bash
```

### Build All Packages

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### Clean Build

```bash
cd ~/ros2_ws
rm -rf build install log
colcon build
source install/setup.bash
```

---

## 🐢 Running Turtlesim Examples

### Start Turtlesim Node

```bash
ros2 run turtlesim turtlesim_node
```

### Run Circle Drawer (in a new terminal)

```bash
# Source the workspace first
source ~/ros2_ws/install/setup.bash

# Run with default parameters
ros2 run turtle_py turtle_circle
```

### Run with Custom Parameters

```bash
# Custom linear speed
ros2 run turtle_py turtle_circle --ros-args -p linear_speed:=3.0

# Custom angular speed
ros2 run turtle_py turtle_circle --ros-args -p angular_speed:=2.0

# Both parameters
ros2 run turtle_py turtle_circle --ros-args -p linear_speed:=1.5 -p angular_speed:=0.5
```

---

## 🔧 Parameter Management

### List All Parameters

```bash
ros2 param list
```

### Get Parameter Value

```bash
ros2 param get /circle_mover linear_speed
ros2 param get /circle_mover angular_speed
```

### Set Parameter Dynamically

```bash
ros2 param set /circle_mover linear_speed 4.0
ros2 param set /circle_mover angular_speed 1.5
```

### Dump All Parameters

```bash
ros2 param dump /circle_mover
```

---

## 📊 ROS 2 Introspection Commands

### List Running Nodes

```bash
ros2 node list
```

### Node Information

```bash
ros2 node info /circle_mover
```

### List Topics

```bash
ros2 topic list
```

### Echo Topic Messages

```bash
ros2 topic echo /turtle1/cmd_vel
```

### Topic Information

```bash
ros2 topic info /turtle1/cmd_vel
ros2 topic hz /turtle1/cmd_vel
```

### Interface Information

```bash
ros2 interface show geometry_msgs/msg/Twist
```

---

## 📁 Package Structure

```
turtle_py/
├── turtle_py/
│   ├── __init__.py
│   └── turtle_circle.py
├── resource/
│   └── turtle_py
├── test/
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
├── setup.py
├── setup.cfg
└── package.xml
```

---

## 🔑 Important Files

### setup.py (Entry Points)

```python
entry_points={
    'console_scripts': [
        'turtle_circle = turtle_py.turtle_circle:main',
    ],
},
```

### package.xml (Dependencies)

```xml
<depend>rclpy</depend>
<depend>geometry_msgs</depend>
```

---

## 🐛 Troubleshooting

### Workspace Not Sourced

If `ros2 run` doesn't find your package:

```bash
source ~/ros2_ws/install/setup.bash
```

To make it permanent, add to `~/.bashrc`:

```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Package Not Found After Build

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select turtle_py
source install/setup.bash
```

### Python Script Not Executable

```bash
chmod +x ~/ros2_ws/src/turtle_py/turtle_py/turtle_circle.py
```

---

## 📝 Useful Aliases

Add these to your `~/.bashrc` for convenience:

```bash
# ROS 2 workspace aliases
alias ws='cd ~/ros2_ws'
alias cb='cd ~/ros2_ws && colcon build && source install/setup.bash'
alias cbs='cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash'
alias src='source ~/ros2_ws/install/setup.bash'

# Apply changes
source ~/.bashrc
```

---

## 🎯 Learning Objectives

By the end of this tutorial, you will understand:

✅ ROS 2 workspace structure and organization  
✅ Creating and building Python packages  
✅ Writing ROS 2 nodes with publishers and timers  
✅ Using parameters for runtime configuration  
✅ Working with Twist messages for robot control  
✅ ROS 2 introspection and debugging tools  

---

## 📚 Additional Resources

- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [ROS 2 Python Client Library (rclpy) API](https://docs.ros2.org/latest/api/rclpy/)
- [Geometry Messages](https://docs.ros2.org/latest/api/geometry_msgs/)

---

## 📧 Contact

**MechaMind Labs**  
Website: [mechamindlabs.com](https://mechamindlabs.com)

---

## 📄 License

This tutorial is provided for educational purposes.

---

**Happy Learning! 🚀**
