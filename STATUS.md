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
- **Return Trip Optimization**:
    - Implemented a **Two-Stage Return**: 
        1. **Blind Transit**: 10s timed surge using reversed gate heading (`T1`) to close the distance.
        2. **Visual Lock**: Search and precise alignment once closer to the gate.
    - Tuned final gate pass surge to **4.0m** to ensure the robot stops safely before the pool wall.
- **Infrastructure**:
    - Performed a clean rebuild and synced YOLO models between `src` and `install`.
    - Configured Git LFS and created a backup repository at `https://github.com/farha215/prequal-task.git`.
- **Behavior Tree Optimization**:
    - Refactored the granular BT structure into a **Phase-Oriented Architecture**.
    - Consolidated ~18 nodes into 4 high-level "Smart Actions": `ActionInitialize`, `ActionPassGate`, `ActionOrbitPole`, and `ActionReturnHome`.
    - Moved search and alignment logic into C++ node internal states for better robustness and visual simplicity.
    - Updated `bt_nodes_model.xml` and synced `prequalification_groot.xml` for seamless visualization in Groot2.
- **Return Trip Fine-Tuning**:
    - Increased the final gate pass surge from **4.0m to 7.0m** to ensure the robot completely clears the gate structure before the final stop.

- **Workspace Reorganization**:
    - Moved all Behavior Tree XML files into a dedicated `src/prequalification_bt/config/` directory for better organization.
    - Cleaned up the root directory and consolidated visualization logic into `prequalification.xml`.

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
Run: `ros2 run prequalification_bt prequalification`
Check: `src/prequalification_bt/bt_nodes.cpp` for core logic and `src/prequalification_bt/config/prequalification.xml` for the mission sequence.
Visualizer: `/home/farha/Downloads/Groot2-v1.9.0-x86_64.AppImage --file /home/farha/robosub/src/prequalification_bt/config/prequalification.xml`
