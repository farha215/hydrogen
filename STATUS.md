# Robosub Project Status - May 17, 2026

## 1. Accomplished
- **Detection Pipeline**: 
    - Transitioned from HSV-based to **ML-based (YOLO)** detection for both Gate and Pole.
    - Optimized `yolo_node.py` to publish only the **highest-confidence detection per class**, eliminating visual clutter.
    - Balanced confidence thresholds: **0.6 for Gate** (to avoid pool walls) and **0.3 for Pole** (for stable tracking).
- **Orbit Logic (Sway-less)**:
    - Implemented a robust **8-step tangential orbit** in the `NavigateAround` node.
    - Added **Active Radial Correction**: Robot dynamically adjusts its turn angle (base 90°) based on distance to the pole (2.0m target) to prevent collisions.
    - Standardized on **Anti-clockwise searching** to re-acquire the pole after each tangential surge.
- **Behavior Tree Optimization (Phase-Oriented Architecture)**:
    - Refactored the granular BT structure from ~18 individual nodes into **4 high-level "Smart Actions"**:
        1. `ActionInitialize`: Unified systems check and diving.
        2. `ActionPassGate`: Integrated searching, alignment, and stabilized surge.
        3. `ActionOrbitPole`: Combined search, approach, and radial-corrected orbit.
        4. `ActionReturnHome`: Sequenced blind transit and final visual gate pass.
    - Moved complex search/align state-machine logic into C++ for better robustness and visual simplicity.
- **Return Trip Fine-Tuning**:
    - Implemented a **Two-Stage Return**: 10s blind transit followed by visual lock.
    - Increased final gate pass surge from **4.0m to 7.0m** (~18s total) to ensure the robot completely clears the gate structure.
- **Workspace Organization & Infrastructure**:
    - Reorganized mission files into a dedicated `src/prequalification_bt/config/` directory.
    - Consolidated Groot2 visualization data into a single modern XML format.
    - Synchronized all local changes with the GitHub repository (`prequal-task`).

## 2. Current Configuration
- **Architecture**: Phase-Oriented Behavior Tree (7 steps total).
- **Target Depth**: 1.5m
- **Orbit Radius**: 2.0m (8 steps @ 3.0s surge, 10.0 power).
- **Gate Alignment**: Requires 1.0s of stable centering before commitment.
- **Return Logic**: Visual-guided through the gate after a 10s blind approach; **7.0m final surge**.

## 3. Immediate Next Steps
- **Simulation Validation**: Run the optimized tree in Gazebo to ensure state transitions are smooth.
- **Hardware Integration**: Deploy the optimized BT to the physical AUV for pool testing.
- **YOLO Robustness**: Test the system with varying ambient lighting to verify YOLO confidence stability.

## 4. How to Resume
- **Build**: `colcon build --packages-select prequalification_bt`
- **Execute**: `ros2 run prequalification_bt prequalification`
- **Source Code**: `src/prequalification_bt/bt_nodes.cpp`
- **Mission Logic**: `src/prequalification_bt/config/prequalification.xml`
- **Visualizer**: `/home/farha/Downloads/Groot2-v1.9.0-x86_64.AppImage --file src/prequalification_bt/config/prequalification.xml`
