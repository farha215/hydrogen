# RoboSub Project Status - May 31, 2026 (Pre-Restart)

## 🎯 Current Mission State: Pre-Qualification
The workspace has been fully reset to the `master` branch and optimized for testing the pre-qualification behavior tree.

### ✅ Recent Accomplishments (This Session)
- **Anti-Cheating Drive Logic**: Refactored `ActionPassGate` and `ActionReturnHome` to use real camera depth (`oz`) for calculating surge durations. The robot no longer relies on hardcoded timers for gate clearance.
- **Alignment Optimization**: 
    - Standardized centering gains to **2.0** for both GATE and POLE.
    - Reduced stabilization wait time from **1.0s to 0.5s** for snappier transitions.
    - Added terminal feedback logs for alignment phase transitions.
- **Blackboard Safety Fix**: Removed fictional seeding of `T1` and `T2` in `main.cpp`. The return-home staging point is now derived purely from real-time detections.
- **Vision Synchronization**: 
    - `vision_fusion_node.py` is configured for `prequal.pt`.
    - BT logic is now case-insensitive and mapped to "GATE" and "POLE" labels.
    - OpenCV debug window is active; verbose terminal logging is silenced.
- **Build Verification**: Clean `colcon build` of all packages (`custom_interfaces`, `hydrogen`, `control_system`, `prequalification_bt`) is verified.

### ⚠️ Pending Hardware Issue
- **GPU Visibility**: The OS currently cannot see the NVIDIA discrete GPU (Intel iGPU is active). 
- **Action Item**: Restart and switch BIOS/MUX to "Discrete Graphics" or disable "Hybrid Mode" in Lenovo Vantage.

### 🚀 Launch Commands (Post-Restart)
```bash
# Terminal 1: Simulation
ros2 launch hydrogen model.launch.py

# Terminal 2: Perception
ros2 run hydrogen vision_fusion_node.py

# Terminal 3: Mission logic
ros2 run prequalification_bt prequalification
```
