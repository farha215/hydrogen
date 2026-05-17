# Robosub Project Status - May 16, 2026

## 1. Accomplished
- **Infrastructure**: Refactored `prequalification_bt` to use a unified `/to_pico` interface instead of competing for `/cmd_vel`.
- **Controller**: Updated `pico_controller.py` to support 6-DOF (added Sway PID) and `ToPico.msg` to include `delta_s`.
- **Depth**: Synchronized all BT nodes to a shared `target_depth` (1.5m) to prevent the robot from floating up during tasks.
- **Gate Task**: 
    - Updated gate mesh to `preq_task_green.dae`.
    - Transitioned from HSV-based detection to ML-based (YOLO) detection for improved robustness.
    - Implemented a stable, heading-locked drive phase that successfully clears the gate.
- **Pole Task**: 
    - Implemented a "Blind Surge" approach to 2.0m to eliminate visual jitter.
    - Implemented a "Step-Based Orbit" (Face -> Turn -> Surge) since the robot lacks physical sway authority.

## 2. Current Configuration
- **Target Depth**: 1.5m
- **Surge Power**: High (`delta_d = 10.0` or `surge = 6.0` in orbit).
- **Orbit Direction**: Clockwise (Negative Yaw turns).
- **Search Direction**: Positive (Left).
- **Gate Detection**: YOLO (class_id 0 -> `preq_gate`).
- **Pole Detection**: YOLO (class_id 1 -> `preq_pole`).


## 3. Next Steps
- **Verify Orbit**: Run the Behavior Tree and check if the Turn-Surge sequence correctly circles the pole.
- **Adjust Timing**: If the surge is too long/short, tweak `duration` in the `orbit_sequence` inside `prequalification.xml`.
- **Home Navigation**: Verify the `nav_back_home` node accurately faces the starting point (`T0`) and surges back.

## 4. How to Resume
Run: `ros2 run prequalification_bt prequalification` to test current behavior.
Read: `src/prequalification_bt/bt_nodes.cpp` for core control logic.
