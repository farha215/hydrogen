#include "bt_nodes.h"


// Retrieve the shared RobotContext from the blackboard
static std::shared_ptr<RobotContext> getCtx(const BT::NodeConfig& cfg) {
    std::shared_ptr<RobotContext> ctx;
    (void)cfg.blackboard->get("robot_context", ctx);
    return ctx;
}

// ─── 1. AllSystemsOK ─────────────────────────────────────────────────────────
BT::NodeStatus AllSystemsOK::tick() {
    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);

    if (!ctx->imu_received) {
        RCLCPP_WARN_THROTTLE(ctx->node->get_logger(),
                             *ctx->node->get_clock(), 2000,
                             "[AllSystemsOK] Waiting for /imu ...");
        return BT::NodeStatus::FAILURE;
    }
    RCLCPP_INFO(ctx->node->get_logger(), "[AllSystemsOK] Systems nominal (IMU + Altimeter active).");
    return BT::NodeStatus::SUCCESS;
}

// ─── 2. SaveToBlackboard ──────────────────────────────────────────────────────
BT::NodeStatus SaveToBlackboard::tick() {
    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);

    Pose current = ctx->getCurrentPose();
    setOutput("key", current);

    RCLCPP_INFO(ctx->node->get_logger(),
                "[SaveToBlackboard] Saved reference yaw=%.3f rad (Depth=%.2f)",
                current.yaw, current.z);
    return BT::NodeStatus::SUCCESS;
}

// ─── 3. DiveToDepth ───────────────────────────────────────────────────────────
BT::NodeStatus DiveToDepth::onStart() {
    auto depth_in = getInput<double>("target_depth");
    if (!depth_in)
        throw BT::RuntimeError("DiveToDepth: missing required port [target_depth]");
    target_z_ = depth_in.value();

    auto ctx = getCtx(config());
    ctx->target_depth = target_z_; // Save to context for other nodes
    RCLCPP_INFO(ctx->node->get_logger(),
                "[DiveToDepth] Commanding dive to z = %.2f m via pico_controller", target_z_);
    
    ctx->publishToPico(0.0f, 0.0f, 0.0f, (float)target_z_, 0);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus DiveToDepth::onRunning() {
    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);

    double current_z = ctx->getCurrentPose().z;
    double dz        = std::abs(target_z_ - current_z);

    if (dz < depth_tolerance_) {
        RCLCPP_INFO(ctx->node->get_logger(),
                    "[DiveToDepth] Reached target depth (current z = %.2f m)", current_z);
        return BT::NodeStatus::SUCCESS;
    }

    // Maintain setpoint
    ctx->publishToPico(0.0f, 0.0f, 0.0f, (float)target_z_, 0);
    return BT::NodeStatus::RUNNING;
}

void DiveToDepth::onHalted() {
    auto ctx = getCtx(config());
    ctx->stopMotion();
    RCLCPP_WARN(ctx->node->get_logger(), "[DiveToDepth] Halted.");
}

// ─── 4. IsObjectSeen ──────────────────────────────────────────────────────────
BT::NodeStatus IsObjectSeen::tick() {
    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);

    auto obj = getInput<std::string>("object");
    if (!obj)
        throw BT::RuntimeError("IsObjectSeen: missing required port [object]");

    bool seen = ctx->isObjectSeen(obj.value());
    RCLCPP_INFO_THROTTLE(ctx->node->get_logger(),
                         *ctx->node->get_clock(), 1000,
                         "[IsObjectSeen] %s → %s",
                         obj.value().c_str(), seen ? "SEEN ✓" : "not seen");
    return seen ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ─── 5. Do360Turn ─────────────────────────────────────────────────────────────
BT::NodeStatus Do360Turn::onStart() {
    auto obj = getInput<std::string>("success_when_seen");
    if (!obj)
        throw BT::RuntimeError("Do360Turn: missing required port [success_when_seen]");
    target_object_ = obj.value();

    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);

    prev_yaw_        = ctx->getCurrentPose().yaw;
    accumulated_yaw_ = 0.0;

    RCLCPP_INFO(ctx->node->get_logger(),
                "[Do360Turn] Searching for %s (360° sweep)", target_object_.c_str());
    
    ctx->publishToPico(0.5f, 0.0f, 0.0f, (float)ctx->target_depth, 0);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Do360Turn::onRunning() {
    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);

    if (ctx->isObjectSeen(target_object_)) {
        ctx->stopMotion();
        RCLCPP_INFO(ctx->node->get_logger(),
                    "[Do360Turn] %s found — stopping rotation.", target_object_.c_str());
        return BT::NodeStatus::SUCCESS;
    }

    double current_yaw = ctx->getCurrentPose().yaw;
    double delta       = std::abs(normalizeAngle(current_yaw - prev_yaw_));
    accumulated_yaw_  += delta;
    prev_yaw_          = current_yaw;

    if (accumulated_yaw_ >= (2.0 * M_PI)) {
        ctx->stopMotion();
        RCLCPP_WARN(ctx->node->get_logger(),
                    "[Do360Turn] Full rotation complete — %s not found.",
                    target_object_.c_str());
        return BT::NodeStatus::FAILURE;
    }

    ctx->publishToPico(0.5f, 0.0f, 0.0f, (float)ctx->target_depth, 0);
    return BT::NodeStatus::RUNNING;
}

void Do360Turn::onHalted() {
    auto ctx = getCtx(config());
    ctx->stopMotion();
}

// ─── 6. DriveThruGate ────────────────────────────────────────────────────────
BT::NodeStatus DriveThruGate::onStart() {
    auto gd = getInput<double>("gate_depth");
    gate_depth_ = gd ? gd.value() : 3.0;

    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);

    entry_pose_ = ctx->getCurrentPose();
    phase_      = Phase::ALIGN;

    RCLCPP_INFO(ctx->node->get_logger(), "[DriveThruGate] Starting Visual ALIGN phase.");
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus DriveThruGate::onRunning() {
    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);

    double ox, oy, oz;
    bool gate_seen = ctx->getObjectPosition("GATE", ox, oy, oz);

    if (phase_ == Phase::ALIGN) {
        if (!gate_seen) {
            ctx->publishToPico(0.3f, 0.0f, 0.0f, (float)ctx->target_depth, 0);
            align_start_time_ = 0.0;
            return BT::NodeStatus::RUNNING;
        }

        double norm_x = ox / std::max(oz, 0.5);

        if (std::abs(norm_x) < ALIGN_TOL) {
            if (align_start_time_ == 0.0) {
                align_start_time_ = ctx->node->get_clock()->now().seconds();
            }
            double aligned_duration = ctx->node->get_clock()->now().seconds() - align_start_time_;
            
            if (aligned_duration >= 1.0) { // Require 1s of stable alignment
                phase_ = Phase::DRIVE;
                start_time_ = ctx->node->get_clock()->now().seconds();
                gate_drive_time_ = (oz + gate_depth_) / 0.5; // Slower speed assumption for longer surge
                entry_pose_ = ctx->getCurrentPose(); 
                RCLCPP_INFO(ctx->node->get_logger(),
                            "[DriveThruGate] STABILIZED ALIGNMENT. Locked heading=%.2f. Driving for %.2f s",
                            entry_pose_.yaw, gate_drive_time_);
            } else {
                ctx->publishToPico(0.0f, 0.0f, 0.0f, (float)ctx->target_depth, 0); // Hold still while stabilizing
            }
        } else {
            align_start_time_ = 0.0;
            ctx->publishToPico(-(float)norm_x * 0.8f, 0.0f, 0.0f, (float)ctx->target_depth, 0);
        }
        return BT::NodeStatus::RUNNING;
    }

    double now = ctx->node->get_clock()->now().seconds();
    double elapsed = now - start_time_;

    if (elapsed >= gate_drive_time_) {
        ctx->stopMotion();
        setOutput("entry_pose", entry_pose_);
        RCLCPP_INFO(ctx->node->get_logger(), "[DriveThruGate] DRIVE phase complete.");
        return BT::NodeStatus::SUCCESS;
    }

    double current_yaw = ctx->getCurrentPose().yaw;
    double yaw_err     = normalizeAngle(entry_pose_.yaw - current_yaw);
    float delta_yaw_setpoint = (float)yaw_err;

    ctx->publishToPico(delta_yaw_setpoint, 10.0f, 0.0f, (float)ctx->target_depth, 0);
    return BT::NodeStatus::RUNNING;
}

void DriveThruGate::onHalted() {
    auto ctx = getCtx(config());
    ctx->stopMotion();
}

// ─── 7. NavigateTo ────────────────────────────────────────────────────────────
BT::NodeStatus NavigateTo::onStart() {
    auto to  = getInput<Pose>("to");
    auto rev = getInput<bool>("reverse");
    auto dur = getInput<double>("duration");
    if (!to) throw BT::RuntimeError("NavigateTo: missing target Pose [to]");
    
    target_ = to.value();
    if (rev && rev.value()) {
        target_.yaw = normalizeAngle(target_.yaw + M_PI);
        RCLCPP_INFO(getCtx(config())->node->get_logger(), "[NavigateTo] REVERSING direction for return.");
    }
    
    duration_ = dur ? dur.value() : 20.0;
    
    auto ctx = getCtx(config());
    start_time_ = 0.0;
    
    RCLCPP_INFO(ctx->node->get_logger(), "[NavigateTo] Target Heading: %.2f rad (%.1f deg), Duration: %.1f s", 
                target_.yaw, target_.yaw * 180.0 / M_PI, duration_);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateTo::onRunning() {
    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);
    Pose cur = ctx->getCurrentPose();

    double yaw_err  = normalizeAngle(target_.yaw - cur.yaw);

    if (start_time_ == 0.0) {
        if (std::abs(yaw_err) < 0.1) {
            start_time_ = ctx->node->get_clock()->now().seconds();
            RCLCPP_INFO(ctx->node->get_logger(), "[NavigateTo] ALIGNED. Starting timed surge return...");
        } else {
            ctx->publishToPico((float)yaw_err * 2.0f, 0.0f, 0.0f, (float)ctx->target_depth, 0);
            return BT::NodeStatus::RUNNING;
        }
    }

    double elapsed = ctx->node->get_clock()->now().seconds() - start_time_;
    if (elapsed >= duration_) { 
        ctx->stopMotion();
        return BT::NodeStatus::SUCCESS;
    }

    ctx->publishToPico((float)yaw_err * 2.0f, 10.0f, 0.0f, (float)ctx->target_depth, 0);
    return BT::NodeStatus::RUNNING;
}

void NavigateTo::onHalted() {
    auto ctx = getCtx(config());
    ctx->stopMotion();
}

// ─── 8. NavigateAround (Sway-less Step Orbit) ───────────────────────────────
BT::NodeStatus NavigateAround::onStart() {
    auto obj = getInput<std::string>("object");
    auto thr = getInput<double>("threshold");
    if (!obj || !thr) throw BT::RuntimeError("NavigateAround: missing ports");
    target_object_ = obj.value();
    threshold_     = thr.value();
    
    auto ctx = getCtx(config());
    steps_completed_ = 0;
    phase_ = Phase::ALIGN;
    
    RCLCPP_INFO(ctx->node->get_logger(), "[NavigateAround] Starting 8-Step Orbit around %s (radius approx %.2f)", 
                target_object_.c_str(), threshold_);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateAround::onRunning() {
    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);
    Pose cur = ctx->getCurrentPose();

    if (steps_completed_ >= 8) {
        ctx->stopMotion();
        RCLCPP_INFO(ctx->node->get_logger(), "[NavigateAround] ORBIT COMPLETE (8 steps).");
        return BT::NodeStatus::SUCCESS;
    }

    double ox, oy, oz;
    bool seen = ctx->getObjectPosition(target_object_, ox, oy, oz);

    if (phase_ == Phase::ALIGN) {
        if (!seen) {
            // Search Anti-clockwise (Positive)
            ctx->publishToPico(0.4f, 0.0f, 0.0f, (float)ctx->target_depth, 0);
            return BT::NodeStatus::RUNNING;
        }
        double norm_x = ox / std::max(oz, 0.5);
        if (std::abs(norm_x) < 0.06) {
            phase_ = Phase::TURN;
            
            // --- GEOMETRY REFINED ---
            // 90 is tangent. 85 points slightly IN, but motion pushes it OUT.
            // This makes the orbit look more "connected" to the pole.
            double dist_err = threshold_ - oz; 
            double correction = dist_err * 0.5; // Reduced gain (from 0.8)
            correction = clampVal(correction, -0.4, 0.4); 
            
            double turn_angle = (85.0 * M_PI / 180.0) + correction; 
            target_yaw_ = normalizeAngle(cur.yaw - turn_angle); 
            
            RCLCPP_INFO(ctx->node->get_logger(), 
                        "[NavigateAround] Step %d/8: ALIGNED (dist=%.2f). Turn: %.1f deg", 
                        steps_completed_ + 1, oz, turn_angle * 180.0 / M_PI);
        } else {
            ctx->publishToPico(-(float)norm_x * 1.5f, 0.0f, 0.0f, (float)ctx->target_depth, 0);
        }
        return BT::NodeStatus::RUNNING;
    }

    if (phase_ == Phase::TURN) {
        double yaw_err = normalizeAngle(target_yaw_ - cur.yaw);
        if (std::abs(yaw_err) < 0.08) {
            phase_ = Phase::SURGE;
            start_time_ = ctx->node->get_clock()->now().seconds();
            locked_yaw_ = cur.yaw;
            surge_duration_ = 3.0; // Optimized for 90 deg tangent
            RCLCPP_INFO(ctx->node->get_logger(), "[NavigateAround] TANGENT. Surging %.1f s...", surge_duration_);
        } else {
            ctx->publishToPico((float)yaw_err * 2.0f, 0.0f, 0.0f, (float)ctx->target_depth, 0);
        }
        return BT::NodeStatus::RUNNING;
    }

    if (phase_ == Phase::SURGE) {
        double elapsed = ctx->node->get_clock()->now().seconds() - start_time_;
        if (elapsed >= surge_duration_) {
            phase_ = Phase::ALIGN;
            steps_completed_++;
            RCLCPP_INFO(ctx->node->get_logger(), "[NavigateAround] STEP COMPLETE. Progress: %d/8", steps_completed_);
            ctx->stopMotion(); 
        } else {
            double yaw_err = normalizeAngle(locked_yaw_ - cur.yaw);
            ctx->publishToPico((float)yaw_err * 2.0f, 10.0f, 0.0f, (float)ctx->target_depth, 0);
        }
        return BT::NodeStatus::RUNNING;
    }

    return BT::NodeStatus::RUNNING;
}

void NavigateAround::onHalted() {
    auto ctx = getCtx(config());
    ctx->stopMotion();
}

// ─── 9. ApproachObject (Heading-Locked Surge) ────────────────────────────────
BT::NodeStatus ApproachObject::onStart() {
    auto obj = getInput<std::string>("object");
    auto thr = getInput<double>("threshold");
    if (!obj || !thr) throw BT::RuntimeError("ApproachObject: missing ports");
    target_object_ = obj.value();
    threshold_     = thr.value();

    auto ctx = getCtx(config());
    locked_yaw_ = ctx->getCurrentPose().yaw; // Lock immediately
    
    RCLCPP_INFO(ctx->node->get_logger(), "[ApproachObject] Heading locked at %.2f. Surging towards %s", locked_yaw_, target_object_.c_str());
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ApproachObject::onRunning() {
    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);

    double ox, oy, oz;
    bool seen = ctx->getObjectPosition(target_object_, ox, oy, oz);

    if (seen && oz <= threshold_) {
        ctx->stopMotion();
        RCLCPP_INFO(ctx->node->get_logger(), "[ApproachObject] Threshold reached (oz=%.2f).", oz);
        return BT::NodeStatus::SUCCESS;
    }

    double current_yaw = ctx->getCurrentPose().yaw;
    double yaw_err = normalizeAngle(locked_yaw_ - current_yaw);
    ctx->publishToPico(2.0f * (float)yaw_err, 8.0f, 0.0f, (float)ctx->target_depth, 0);
    return BT::NodeStatus::RUNNING;
}

void ApproachObject::onHalted() {
    getCtx(config())->stopMotion();
}

// ─── 10. AlignWithObject (Stationary Centering) ───────────────────────────────
BT::NodeStatus AlignWithObject::onStart() {
    auto obj = getInput<std::string>("object");
    if (!obj) throw BT::RuntimeError("AlignWithObject: missing object port");
    target_object_ = obj.value();
    RCLCPP_INFO(getCtx(config())->node->get_logger(), "[AlignWithObject] Stationary centering on %s", target_object_.c_str());
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus AlignWithObject::onRunning() {
    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);

    double ox, oy, oz;
    if (!ctx->getObjectPosition(target_object_, ox, oy, oz)) {
        // If lost, just hold still and wait to see it again rather than spinning away
        ctx->stopMotion(); 
        RCLCPP_INFO_THROTTLE(ctx->node->get_logger(), *ctx->node->get_clock(), 1000,
                            "[AlignWithObject] %s lost, waiting...", target_object_.c_str());
        return BT::NodeStatus::RUNNING;
    }

    double norm_x = ox / std::max(oz, 0.5);
    RCLCPP_INFO_THROTTLE(ctx->node->get_logger(), *ctx->node->get_clock(), 500,
                        "[AlignWithObject] Centering %s: norm_x=%.3f (dist=%.2f)", 
                        target_object_.c_str(), norm_x, oz);

    if (std::abs(norm_x) < 0.05) {
        ctx->stopMotion();
        RCLCPP_INFO(ctx->node->get_logger(), "[AlignWithObject] %s Centered.", target_object_.c_str());
        return BT::NodeStatus::SUCCESS;
    }

    // Centering gain
    ctx->publishToPico(-(float)norm_x * 1.5f, 0.0f, 0.0f, (float)ctx->target_depth, 0);
    return BT::NodeStatus::RUNNING;
}

void AlignWithObject::onHalted() {
    getCtx(config())->stopMotion();
}

// ─── 11. MoveRelatively (Stationary Timed Move) ───────────────────────────────
BT::NodeStatus MoveRelatively::onStart() {
    auto surge_in = getInput<double>("surge");
    auto sway_in  = getInput<double>("sway");
    auto dur_in   = getInput<double>("duration");
    if (!surge_in || !sway_in || !dur_in) throw BT::RuntimeError("MoveRelatively: missing ports");
    
    surge_ = surge_in.value();
    sway_  = sway_in.value();
    duration_ = dur_in.value();
    start_time_ = std::chrono::steady_clock::now();

    RCLCPP_INFO(getCtx(config())->node->get_logger(), "[MoveRelatively] Moving: surge=%.2f sway=%.2f for %.2f s", surge_, sway_, duration_);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveRelatively::onRunning() {
    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);
    
    auto now = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(now - start_time_).count();

    if (elapsed >= duration_) {
        ctx->stopMotion();
        return BT::NodeStatus::SUCCESS;
    }

    // Send surge/sway errors to the pico controller
    ctx->publishToPico(0.0f, (float)surge_, (float)sway_, (float)ctx->target_depth, 0);

    return BT::NodeStatus::RUNNING;
}

void MoveRelatively::onHalted() {
    getCtx(config())->stopMotion();
}

// ─── 12. RelativeTurn (Stationary Angle Turn) ──────────────────────────────────
BT::NodeStatus RelativeTurn::onStart() {
    auto angle_in = getInput<double>("angle");
    if (!angle_in) throw BT::RuntimeError("RelativeTurn: missing [angle] port");
    
    angle_ = angle_in.value();
    auto ctx = getCtx(config());
    target_yaw_ = normalizeAngle(ctx->getCurrentPose().yaw + angle_);

    RCLCPP_INFO(ctx->node->get_logger(), "[RelativeTurn] Turning relative %.2f to target %.2f", angle_, target_yaw_);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus RelativeTurn::onRunning() {
    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);

    double current_yaw = ctx->getCurrentPose().yaw;
    double yaw_err     = normalizeAngle(target_yaw_ - current_yaw);

    if (std::abs(yaw_err) < 0.05) {
        ctx->stopMotion();
        RCLCPP_INFO(ctx->node->get_logger(), "[RelativeTurn] Turn complete.");
        return BT::NodeStatus::SUCCESS;
    }

    ctx->publishToPico((float)yaw_err, 0.0f, 0.0f, (float)ctx->target_depth, 0);
    return BT::NodeStatus::RUNNING;
}

void RelativeTurn::onHalted() {
    getCtx(config())->stopMotion();
}

// ─── 13. StayStill ────────────────────────────────────────────────────────────
BT::NodeStatus StayStill::onStart() {
    auto d = getInput<double>("duration");
    duration_ = d ? d.value() : 2.0;
    start_time_ = std::chrono::steady_clock::now();
    auto ctx = getCtx(config());
    RCLCPP_INFO(ctx->node->get_logger(), "[StayStill] Holding for %.1f s", duration_);
    ctx->stopMotion();
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus StayStill::onRunning() {
    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);
    
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(now - start_time_).count() >= duration_) {
        ctx->stopMotion();
        return BT::NodeStatus::SUCCESS;
    }
    ctx->stopMotion();
    return BT::NodeStatus::RUNNING;
}

void StayStill::onHalted() {
    auto ctx = getCtx(config());
    ctx->stopMotion();
}

// ─── Consolidated Action Implementations ──────────────────────────────────────

// 1. ActionInitialize (Systems Check + Dive)
BT::NodeStatus ActionInitialize::onStart() {
    target_depth_ = getInput<double>("target_depth").value_or(1.5);
    phase_ = Phase::CHECK;
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ActionInitialize::onRunning() {
    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);

    if (phase_ == Phase::CHECK) {
        if (!ctx->imu_received) {
            RCLCPP_WARN_THROTTLE(ctx->node->get_logger(), *ctx->node->get_clock(), 2000, "[ActionInitialize] Waiting for IMU...");
            return BT::NodeStatus::RUNNING;
        }
        phase_ = Phase::DIVE;
        ctx->target_depth = target_depth_;
        RCLCPP_INFO(ctx->node->get_logger(), "[ActionInitialize] Systems OK. Diving to %.2fm", target_depth_);
    }

    double current_z = ctx->getCurrentPose().z;
    if (std::abs(target_depth_ - current_z) < 0.15) {
        RCLCPP_INFO(ctx->node->get_logger(), "[ActionInitialize] Ready at %.2fm", current_z);
        return BT::NodeStatus::SUCCESS;
    }

    ctx->publishToPico(0.0f, 0.0f, 0.0f, (float)target_depth_, 0);
    return BT::NodeStatus::RUNNING;
}

void ActionInitialize::onHalted() { getCtx(config())->stopMotion(); }

// 2. ActionPassGate (Search + Align + Drive)
BT::NodeStatus ActionPassGate::onStart() {
    gate_depth_ = getInput<double>("gate_depth").value_or(6.0);
    phase_ = Phase::SEARCH;
    auto ctx = getCtx(config());
    accum_yaw_ = 0.0;
    prev_yaw_ = ctx->getCurrentPose().yaw;
    RCLCPP_INFO(ctx->node->get_logger(), "[ActionPassGate] Starting. Phase: SEARCH");
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ActionPassGate::onRunning() {
    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);
    Pose cur = ctx->getCurrentPose();

    double ox, oy, oz;
    bool seen = ctx->getObjectPosition("GATE", ox, oy, oz);

    if (phase_ == Phase::SEARCH) {
        if (seen) {
            phase_ = Phase::ALIGN;
            align_start_time_ = 0.0;
            RCLCPP_INFO(ctx->node->get_logger(), "[ActionPassGate] Gate seen. Phase: ALIGN");
            return BT::NodeStatus::RUNNING;
        }
        double delta = std::abs(normalizeAngle(cur.yaw - prev_yaw_));
        accum_yaw_ += delta;
        prev_yaw_ = cur.yaw;
        if (accum_yaw_ >= 2.0 * M_PI) return BT::NodeStatus::FAILURE;
        ctx->publishToPico(0.5f, 0.0f, 0.0f, (float)ctx->target_depth, 0);
    } 
    else if (phase_ == Phase::ALIGN) {
        if (!seen) {
            phase_ = Phase::SEARCH; // Re-search if lost
            return BT::NodeStatus::RUNNING;
        }
        double norm_x = ox / std::max(oz, 0.5);
        if (std::abs(norm_x) < 0.04) {
            if (align_start_time_ == 0.0) align_start_time_ = ctx->node->get_clock()->now().seconds();
            if (ctx->node->get_clock()->now().seconds() - align_start_time_ >= 1.0) {
                phase_ = Phase::DRIVE;
                start_time_ = ctx->node->get_clock()->now().seconds();
                entry_pose_ = cur;
                RCLCPP_INFO(ctx->node->get_logger(), "[ActionPassGate] Aligned. Phase: DRIVE (%.1fs surge)", (oz + gate_depth_) / 0.5);
            }
        } else {
            align_start_time_ = 0.0;
            ctx->publishToPico(-(float)norm_x * 0.8f, 0.0f, 0.0f, (float)ctx->target_depth, 0);
        }
    } 
    else if (phase_ == Phase::DRIVE) {
        double elapsed = ctx->node->get_clock()->now().seconds() - start_time_;
        if (elapsed >= (gate_depth_ + 2.0) / 0.5) { // Rough time estimate
            setOutput("entry_pose", entry_pose_);
            return BT::NodeStatus::SUCCESS;
        }
        double yaw_err = normalizeAngle(entry_pose_.yaw - cur.yaw);
        ctx->publishToPico((float)yaw_err, 10.0f, 0.0f, (float)ctx->target_depth, 0);
    }
    return BT::NodeStatus::RUNNING;
}

void ActionPassGate::onHalted() { getCtx(config())->stopMotion(); }

// 3. ActionOrbitPole (Search + Align + Approach + Orbit)
BT::NodeStatus ActionOrbitPole::onStart() {
    radius_ = getInput<double>("radius").value_or(2.0);
    phase_ = Phase::SEARCH;
    auto ctx = getCtx(config());
    accum_yaw_ = 0.0;
    prev_yaw_ = ctx->getCurrentPose().yaw;
    steps_completed_ = 0;
    RCLCPP_INFO(ctx->node->get_logger(), "[ActionOrbitPole] Starting. Phase: SEARCH");
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ActionOrbitPole::onRunning() {
    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);
    Pose cur = ctx->getCurrentPose();
    double ox, oy, oz;
    bool seen = ctx->getObjectPosition("POLE", ox, oy, oz);

    if (phase_ == Phase::SEARCH) {
        if (seen) { phase_ = Phase::ALIGN; return BT::NodeStatus::RUNNING; }
        double delta = std::abs(normalizeAngle(cur.yaw - prev_yaw_));
        accum_yaw_ += delta;
        prev_yaw_ = cur.yaw;
        if (accum_yaw_ >= 2.0 * M_PI) return BT::NodeStatus::FAILURE;
        ctx->publishToPico(0.5f, 0.0f, 0.0f, (float)ctx->target_depth, 0);
    }
    else if (phase_ == Phase::ALIGN) {
        if (!seen) { phase_ = Phase::SEARCH; return BT::NodeStatus::RUNNING; }
        double norm_x = ox / std::max(oz, 0.5);
        if (std::abs(norm_x) < 0.05) { phase_ = Phase::APPROACH; locked_yaw_ = cur.yaw; }
        else ctx->publishToPico(-(float)norm_x * 1.5f, 0.0f, 0.0f, (float)ctx->target_depth, 0);
    }
    else if (phase_ == Phase::APPROACH) {
        if (seen && oz <= radius_) { phase_ = Phase::ORBIT_STEP_ALIGN; }
        else {
            double yaw_err = normalizeAngle(locked_yaw_ - cur.yaw);
            ctx->publishToPico(2.0f * (float)yaw_err, 8.0f, 0.0f, (float)ctx->target_depth, 0);
        }
    }
    else if (phase_ == Phase::ORBIT_STEP_ALIGN) {
        if (steps_completed_ >= 8) return BT::NodeStatus::SUCCESS;
        if (!seen) { ctx->publishToPico(0.4f, 0.0f, 0.0f, (float)ctx->target_depth, 0); }
        else {
            double norm_x = ox / std::max(oz, 0.5);
            if (std::abs(norm_x) < 0.06) {
                phase_ = Phase::ORBIT_STEP_TURN;
                double correction = clampVal((radius_ - oz) * 0.5, -0.4, 0.4);
                target_yaw_ = normalizeAngle(cur.yaw - (85.0 * M_PI / 180.0) - correction);
            } else ctx->publishToPico(-(float)norm_x * 1.5f, 0.0f, 0.0f, (float)ctx->target_depth, 0);
        }
    }
    else if (phase_ == Phase::ORBIT_STEP_TURN) {
        double yaw_err = normalizeAngle(target_yaw_ - cur.yaw);
        if (std::abs(yaw_err) < 0.08) { phase_ = Phase::ORBIT_STEP_SURGE; start_time_ = ctx->node->get_clock()->now().seconds(); locked_yaw_ = cur.yaw; }
        else ctx->publishToPico((float)yaw_err * 2.0f, 0.0f, 0.0f, (float)ctx->target_depth, 0);
    }
    else if (phase_ == Phase::ORBIT_STEP_SURGE) {
        if (ctx->node->get_clock()->now().seconds() - start_time_ >= 3.0) { phase_ = Phase::ORBIT_STEP_ALIGN; steps_completed_++; }
        else ctx->publishToPico(normalizeAngle(locked_yaw_ - cur.yaw) * 2.0f, 10.0f, 0.0f, (float)ctx->target_depth, 0);
    }
    return BT::NodeStatus::RUNNING;
}

void ActionOrbitPole::onHalted() { getCtx(config())->stopMotion(); }

// 4. ActionReturnHome (Transit + Search + Align + Drive)
BT::NodeStatus ActionReturnHome::onStart() {
    auto hp = getInput<Pose>("home_pose");
    if (!hp) return BT::NodeStatus::FAILURE;
    home_pose_ = hp.value();
    home_pose_.yaw = normalizeAngle(home_pose_.yaw + M_PI);
    transit_dur_ = getInput<double>("transit_duration").value_or(10.0);
    gate_depth_ = getInput<double>("gate_depth").value_or(4.0);
    phase_ = Phase::TRANSIT_TURN;
    RCLCPP_INFO(getCtx(config())->node->get_logger(), "[ActionReturnHome] Starting. Phase: TRANSIT_TURN");
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ActionReturnHome::onRunning() {
    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);
    Pose cur = ctx->getCurrentPose();

    if (phase_ == Phase::TRANSIT_TURN) {
        double yaw_err = normalizeAngle(home_pose_.yaw - cur.yaw);
        if (std::abs(yaw_err) < 0.1) { phase_ = Phase::TRANSIT_SURGE; start_time_ = ctx->node->get_clock()->now().seconds(); }
        else ctx->publishToPico((float)yaw_err * 2.0f, 0.0f, 0.0f, (float)ctx->target_depth, 0);
    }
    else if (phase_ == Phase::TRANSIT_SURGE) {
        if (ctx->node->get_clock()->now().seconds() - start_time_ >= transit_dur_) { phase_ = Phase::SEARCH; accum_yaw_ = 0.0; prev_yaw_ = cur.yaw; }
        else ctx->publishToPico(normalizeAngle(home_pose_.yaw - cur.yaw) * 2.0f, 10.0f, 0.0f, (float)ctx->target_depth, 0);
    }
    else if (phase_ == Phase::SEARCH) {
        double ox, oy, oz;
        if (ctx->getObjectPosition("GATE", ox, oy, oz)) { phase_ = Phase::ALIGN; align_start_time_ = 0.0; }
        else {
            double delta = std::abs(normalizeAngle(cur.yaw - prev_yaw_));
            accum_yaw_ += delta; prev_yaw_ = cur.yaw;
            if (accum_yaw_ >= 2.0 * M_PI) return BT::NodeStatus::FAILURE;
            ctx->publishToPico(0.5f, 0.0f, 0.0f, (float)ctx->target_depth, 0);
        }
    }
    else if (phase_ == Phase::ALIGN) {
        double ox, oy, oz;
        if (!ctx->getObjectPosition("GATE", ox, oy, oz)) { phase_ = Phase::SEARCH; return BT::NodeStatus::RUNNING; }
        double norm_x = ox / std::max(oz, 0.5);
        if (std::abs(norm_x) < 0.04) {
            if (align_start_time_ == 0.0) align_start_time_ = ctx->node->get_clock()->now().seconds();
            if (ctx->node->get_clock()->now().seconds() - align_start_time_ >= 1.0) { phase_ = Phase::DRIVE; start_time_ = ctx->node->get_clock()->now().seconds(); home_pose_ = cur; }
        } else ctx->publishToPico(-(float)norm_x * 0.8f, 0.0f, 0.0f, (float)ctx->target_depth, 0);
    }
    else if (phase_ == Phase::DRIVE) {
        if (ctx->node->get_clock()->now().seconds() - start_time_ >= (gate_depth_ + 2.0) / 0.5) return BT::NodeStatus::SUCCESS;
        ctx->publishToPico(normalizeAngle(home_pose_.yaw - cur.yaw), 10.0f, 0.0f, (float)ctx->target_depth, 0);
    }
    return BT::NodeStatus::RUNNING;
}

void ActionReturnHome::onHalted() { getCtx(config())->stopMotion(); }
