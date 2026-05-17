# Robosub Project Status - May 17, 2026

## 1. Accomplished
- **Detection Pipeline**: 
    - Transitioned from HSV-based to **ML-based (YOLO)** detection for both Gate and Pole.
    - **Optimization**: Silenced high-frequency console logging and disabled redundant image publishing to reduce CPU overhead.
    - Balanced confidence thresholds: **0.6 for Gate** and **0.3 for Pole**.
- **Orbit Logic (Sway-less)**:
    - Implemented a robust **8-step tangential orbit**.
    - **Fine-Tuning**: Reduced step surge duration to **2.5s** to compensate for improved simulation performance and prevent overshooting.
- **Behavior Tree Optimization**:
    - Refactored into **4 high-level "Smart Actions"** (`ActionInitialize`, `ActionPassGate`, `ActionOrbitPole`, `ActionReturnHome`).
    - **Bug Fix**: Resolved `BT::RuntimeError` by correcting the installation path for XML files in `CMakeLists.txt`.
- **Return Trip Fine-Tuning**:
    - Extended the final gate pass surge to **10.0m** (~20s total) to ensure the robot completely clears the gate structure during the return journey.
    - Maintained a standard **2.0m** surge for the initial gate entry.
- **Simulation Lag Mitigation**:
    - Removed high-bandwidth **PointCloud2** topics from the `ros_gz_bridge` configuration.
    - Optimized node spinning logic in the Behavior Tree implementation to reduce redundant processing.

## 2. Current Configuration
- **Architecture**: Phase-Oriented Behavior Tree (7 steps total).
- **Target Depth**: 1.5m
- **Orbit Radius**: 2.0m (8 steps @ 2.5s surge).
- **Gate Clearing**: 2.0m (Initial) / 10.0m (Return).
- **Vision**: 30 FPS RGB/Depth maintained with detection GUI active.

## 3. Immediate Next Steps
- **Simulation Validation**: Perform a full mission run to verify the 10m return surge and 2.5s orbit steps.
- **Hardware Integration**: Deploy the optimized BT to the physical AUV for pool testing.
- **Stability Monitoring**: Monitor Gazebo Real Time Factor (RTF) to ensure lag is permanently resolved.

## 4. How to Resume
- **Build**: `colcon build --packages-select prequalification_bt`
- **Execute**: `ros2 run prequalification_bt prequalification`
- **Source Code**: `src/prequalification_bt/bt_nodes.cpp`
- **Mission Logic**: `src/prequalification_bt/config/prequalification.xml`
- **Visualizer**: `/home/farha/Downloads/Groot2-v1.9.0-x86_64.AppImage --file src/prequalification_bt/config/prequalification.xml`
