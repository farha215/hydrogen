# Hydrogen AUV - RoboSub 2026

![ROS 2](https://img.shields.io/badge/ROS%202-Humble-orange)
![Ignition](https://img.shields.io/badge/Ignition-Fortress-red)

This repository contains the software stack for the **Hydrogen** AUV, developed for the RoboSub 2026 competition. The system is built on ROS 2 Humble and utilizes Ignition Gazebo for simulation.

## 1. System Architecture

The system utilizes a modular architecture designed for stability and mission flexibility:

- **Mission Control**: BehaviorTree.CPP manages high-level mission phases.
- **Vision**: YOLOv8 + Depth Fusion for real-time 3D object detection.
- **Control**: 6-DOF PID controller (`PicoController`) with thruster allocation.
- **Simulation**: High-fidelity physical simulation in Ignition Gazebo.

### Packages
| Package | Description |
|---|---|
| `hydrogen` | Robot URDF/Xacro, Gazebo worlds, and vision nodes. |
| `control_system` | PID controllers and thruster allocation matrix. |
| `prequalification_bt` | C++ Behavior Tree implementation for mission tasks. |
| `custom_interfaces` | Custom message and service definitions. |

## 2. Prequalification Mission

The prequalification mission is implemented as a sequence of high-level actions:
1. **Initialize**: Submerge to target depth and stabilize.
2. **Gate Mission**: Detect and pass through the starting gate.
3. **Pole Mission**: Locate the red pole and perform a circular orbit.
4. **Return Home**: Navigate back through the gate to the starting point.

## 3. Getting Started

### Prerequisites
- ROS 2 Humble
- Ignition Gazebo Fortress
- BehaviorTree.CPP v4
- Ultralytics (YOLOv8)
- OpenCV & CvBridge

### Installation
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <repository_url> .
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### Running Simulation
```bash
# Terminal 1: Launch Gazebo and core nodes
ros2 launch hydrogen model.launch.py

# Terminal 2: Start the autonomous mission
ros2 run prequalification_bt prequalification
```

## 4. Key Configurations
- **Thruster Allocation**: Defined in `src/control_system/control_system/allocation_matrix.py`.
- **PID Gains**: Tunable via parameters in `src/control_system/control_system/pico_controller.py`.
- **Mission Tree**: XML configuration at `src/prequalification_bt/config/prequalification.xml`.
- **Vision Thresholds**: Adjusted in `src/hydrogen/hydrogen/vision_fusion_node.py`.

## 5. Development Status
See [STATUS.md](STATUS.md) for the latest updates on detection pipelines, mission logic, and simulation optimizations.

---

