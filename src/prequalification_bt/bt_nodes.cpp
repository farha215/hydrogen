#include "bt_nodes.h"

/**
 * @brief Helper to retrieve the shared RobotContext from the blackboard.
 */
static std::shared_ptr<RobotContext> getCtx(const BT::NodeConfig& cfg) {
    std::shared_ptr<RobotContext> ctx;
    if (!cfg.blackboard->get("robot_context", ctx)) {
        throw BT::RuntimeError("MISSING robot_context on blackboard.");
    }
    return ctx;
}

// ─── 1. AllSystemsOK ─────────────────────────────────────────────────────────
BT::NodeStatus AllSystemsOK::tick() {
    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);

    if (!ctx->imu_received) {
        RCLCPP_WARN_THROTTLE(ctx->node->get_logger(), *ctx->node->get_clock(), 2000, "[AllSystemsOK] Waiting for /imu ...");
        return BT::NodeStatus::RUNNING;
    }
    RCLCPP_INFO(ctx->node->get_logger(), "[AllSystemsOK] Systems nominal.");
    return BT::NodeStatus::SUCCESS;
}

// ─── 2. SaveToBlackboard ──────────────────────────────────────────────────────
BT::NodeStatus SaveToBlackboard::tick() {
    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);

    Pose current = ctx->getCurrentPose();
    setOutput("key", current);

    RCLCPP_INFO(ctx->node->get_logger(), "[SaveToBlackboard] Saved reference yaw=%.3f rad", current.yaw);
    return BT::NodeStatus::SUCCESS;
}

// ─── 3. DiveToDepth ───────────────────────────────────────────────────────────
BT::NodeStatus DiveToDepth::onStart() {
    auto depth_in = getInput<double>("target_depth");
    if (!depth_in) throw BT::RuntimeError("DiveToDepth: missing [target_depth]");
    target_z_ = depth_in.value();

    auto ctx = getCtx(config());
    ctx->target_depth = target_z_;
    RCLCPP_INFO(ctx->node->get_logger(), "[DiveToDepth] Diving to z = %.2f m", target_z_);
    
    ctx->publishToPico(0.0f, 0.0f, 0.0f, (float)target_z_, 0);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus DiveToDepth::onRunning() {
    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);

    double current_z = ctx->getCurrentPose().z;
    if (std::abs(target_z_ - current_z) < depth_tolerance_) {
        RCLCPP_INFO(ctx->node->get_logger(), "[DiveToDepth] Target depth reached.");
        return BT::NodeStatus::SUCCESS;
    }

    ctx->publishToPico(0.0f, 0.0f, 0.0f, (float)target_z_, 0);
    return BT::NodeStatus::RUNNING;
}

void DiveToDepth::onHalted() { getCtx(config())->stopMotion(); }

// ─── 4. IsObjectSeen ──────────────────────────────────────────────────────────
BT::NodeStatus IsObjectSeen::tick() {
    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);

    auto obj = getInput<std::string>("object");
    if (!obj) throw BT::RuntimeError("IsObjectSeen: missing [object]");

    return ctx->isObjectSeen(obj.value()) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ─── 5. Do360Turn ─────────────────────────────────────────────────────────────
BT::NodeStatus Do360Turn::onStart() {
    auto obj = getInput<std::string>("success_when_seen");
    if (!obj) throw BT::RuntimeError("Do360Turn: missing [success_when_seen]");
    target_object_ = obj.value();

    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);

    prev_yaw_ = ctx->getCurrentPose().yaw;
    accumulated_yaw_ = 0.0;

    RCLCPP_INFO(ctx->node->get_logger(), "[Do360Turn] Searching for %s...", target_object_.c_str());
    ctx->publishToPico(0.5f, 0.0f, 0.0f, (float)ctx->target_depth, 0);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Do360Turn::onRunning() {
    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);

    if (ctx->isObjectSeen(target_object_)) {
        ctx->stopMotion();
        RCLCPP_INFO(ctx->node->get_logger(), "[Do360Turn] %s found.", target_object_.c_str());
        return BT::NodeStatus::SUCCESS;
    }

    double current_yaw = ctx->getCurrentPose().yaw;
    accumulated_yaw_ += std::abs(normalizeAngle(current_yaw - prev_yaw_));
    prev_yaw_ = current_yaw;

    if (accumulated_yaw_ >= (2.0 * M_PI)) {
        ctx->stopMotion();
        RCLCPP_WARN(ctx->node->get_logger(), "[Do360Turn] Full rotation complete. %s not found.", target_object_.c_str());
        return BT::NodeStatus::FAILURE;
    }

    ctx->publishToPico(0.5f, 0.0f, 0.0f, (float)ctx->target_depth, 0);
    return BT::NodeStatus::RUNNING;
}

void Do360Turn::onHalted() { getCtx(config())->stopMotion(); }

// ─── 6. DriveThruGate ────────────────────────────────────────────────────────
BT::NodeStatus DriveThruGate::onStart() {
    gate_depth_ = getInput<double>("gate_depth").value_or(3.0);
    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);
    entry_pose_ = ctx->getCurrentPose();
    phase_ = Phase::ALIGN;
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
        if (std::abs(norm_x) < 0.04) {
            if (align_start_time_ == 0.0) align_start_time_ = ctx->node->get_clock()->now().seconds();
            if (ctx->node->get_clock()->now().seconds() - align_start_time_ >= 1.0) {
                phase_ = Phase::DRIVE;
                start_time_ = ctx->node->get_clock()->now().seconds();
                gate_drive_time_ = (oz + gate_depth_) / 0.5;
                entry_pose_ = ctx->getCurrentPose(); 
                RCLCPP_INFO(ctx->node->get_logger(), "[DriveThruGate] Aligned. Surging for %.2f s", gate_drive_time_);
            } else {
                ctx->stopMotion();
            }
        } else {
            align_start_time_ = 0.0;
            ctx->publishToPico(-(float)norm_x * 0.8f, 0.0f, 0.0f, (float)ctx->target_depth, 0);
        }
        return BT::NodeStatus::RUNNING;
    }

    double elapsed = ctx->node->get_clock()->now().seconds() - start_time_;
    if (elapsed >= gate_drive_time_) {
        ctx->stopMotion();
        setOutput("entry_pose", entry_pose_);
        return BT::NodeStatus::SUCCESS;
    }

    double yaw_err = normalizeAngle(entry_pose_.yaw - ctx->getCurrentPose().yaw);
    ctx->publishToPico((float)yaw_err, 10.0f, 0.0f, (float)ctx->target_depth, 0);
    return BT::NodeStatus::RUNNING;
}

void DriveThruGate::onHalted() { getCtx(config())->stopMotion(); }

// ─── 7. NavigateTo ────────────────────────────────────────────────────────────
BT::NodeStatus NavigateTo::onStart() {
    auto to = getInput<Pose>("to");
    auto rev = getInput<bool>("reverse");
    auto dur = getInput<double>("duration");
    if (!to) throw BT::RuntimeError("NavigateTo: missing target [to]");
    
    target_ = to.value();
    if (rev && rev.value()) target_.yaw = normalizeAngle(target_.yaw + M_PI);
    duration_ = dur ? dur.value() : 20.0;
    
    auto ctx = getCtx(config());
    start_time_ = 0.0;
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateTo::onRunning() {
    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);
    Pose cur = ctx->getCurrentPose();
    double yaw_err = normalizeAngle(target_.yaw - cur.yaw);

    if (start_time_ == 0.0) {
        if (std::abs(yaw_err) < 0.1) {
            start_time_ = ctx->node->get_clock()->now().seconds();
            RCLCPP_INFO(ctx->node->get_logger(), "[NavigateTo] Aligned. Starting timed surge...");
        } else {
            ctx->publishToPico((float)yaw_err * 2.0f, 0.0f, 0.0f, (float)ctx->target_depth, 0);
            return BT::NodeStatus::RUNNING;
        }
    }

    if (ctx->node->get_clock()->now().seconds() - start_time_ >= duration_) { 
        ctx->stopMotion();
        return BT::NodeStatus::SUCCESS;
    }

    ctx->publishToPico((float)yaw_err * 2.0f, 10.0f, 0.0f, (float)ctx->target_depth, 0);
    return BT::NodeStatus::RUNNING;
}

void NavigateTo::onHalted() { getCtx(config())->stopMotion(); }

// ─── 8. NavigateAround (Sway-less Step Orbit) ───────────────────────────────
BT::NodeStatus NavigateAround::onStart() {
    auto obj = getInput<std::string>("object");
    auto thr = getInput<double>("threshold");
    if (!obj || !thr) throw BT::RuntimeError("NavigateAround: missing ports");
    target_object_ = obj.value();
    threshold_ = thr.value();
    
    steps_completed_ = 0;
    phase_ = Phase::ALIGN;
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateAround::onRunning() {
    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);
    Pose cur = ctx->getCurrentPose();

    if (steps_completed_ >= 8) {
        ctx->stopMotion();
        RCLCPP_INFO(ctx->node->get_logger(), "[NavigateAround] Orbit complete.");
        return BT::NodeStatus::SUCCESS;
    }

    double ox, oy, oz;
    bool seen = ctx->getObjectPosition(target_object_, ox, oy, oz);

    if (phase_ == Phase::ALIGN) {
        if (!seen) {
            ctx->publishToPico(0.4f, 0.0f, 0.0f, (float)ctx->target_depth, 0);
            return BT::NodeStatus::RUNNING;
        }
        double norm_x = ox / std::max(oz, 0.5);
        if (std::abs(norm_x) < 0.06) {
            phase_ = Phase::TURN;
            double correction = clampVal((threshold_ - oz) * 0.5, -0.4, 0.4); 
            target_yaw_ = normalizeAngle(cur.yaw - (85.0 * M_PI / 180.0) - correction); 
            RCLCPP_INFO(ctx->node->get_logger(), "[NavigateAround] Step %d/8: Turning to tangent.", steps_completed_ + 1);
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
        } else {
            ctx->publishToPico((float)yaw_err * 2.0f, 0.0f, 0.0f, (float)ctx->target_depth, 0);
        }
        return BT::NodeStatus::RUNNING;
    }

    if (phase_ == Phase::SURGE) {
        if (ctx->node->get_clock()->now().seconds() - start_time_ >= 2.5) {
            phase_ = Phase::ALIGN;
            steps_completed_++;
            ctx->stopMotion(); 
        } else {
            double yaw_err = normalizeAngle(locked_yaw_ - cur.yaw);
            ctx->publishToPico((float)yaw_err * 2.0f, 10.0f, 0.0f, (float)ctx->target_depth, 0);
        }
        return BT::NodeStatus::RUNNING;
    }
    return BT::NodeStatus::RUNNING;
}

void NavigateAround::onHalted() { getCtx(config())->stopMotion(); }

// ─── 9. StayStill ────────────────────────────────────────────────────────────
BT::NodeStatus StayStill::onStart() {
    duration_ = getInput<double>("duration").value_or(2.0);
    start_time_ = std::chrono::steady_clock::now();
    getCtx(config())->stopMotion();
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus StayStill::onRunning() {
    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);
    if (std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time_).count() >= duration_) {
        ctx->stopMotion();
        return BT::NodeStatus::SUCCESS;
    }
    ctx->stopMotion();
    return BT::NodeStatus::RUNNING;
}

void StayStill::onHalted() { getCtx(config())->stopMotion(); }

// ─── High-Level Action Implementations ──────────────────────────────────────

BT::NodeStatus ActionInitialize::onStart() {
    target_depth_ = getInput<double>("target_depth").value_or(1.5);
    phase_ = Phase::DIVE;
    auto ctx = getCtx(config());
    ctx->target_depth = target_depth_;
    RCLCPP_INFO(ctx->node->get_logger(), "[ActionInitialize] Diving to %.2f m", target_depth_);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ActionInitialize::onRunning() {
    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);

    if (phase_ == Phase::DIVE) {
        if (std::abs(target_depth_ - ctx->getCurrentPose().z) < 0.15) {
            phase_ = Phase::STAY_STILL;
            start_time_ = ctx->node->get_clock()->now().seconds();
        } else {
            ctx->publishToPico(0.0f, 0.0f, 0.0f, (float)target_depth_, 0);
        }
    } else {
        if (ctx->node->get_clock()->now().seconds() - start_time_ >= 2.0) return BT::NodeStatus::SUCCESS;
        ctx->stopMotion();
    }
    return BT::NodeStatus::RUNNING;
}

void ActionInitialize::onHalted() { getCtx(config())->stopMotion(); }

BT::NodeStatus ActionPassGate::onStart() {
    gate_depth_ = getInput<double>("gate_depth").value_or(6.0);
    phase_ = Phase::SEARCH;
    auto ctx = getCtx(config());
    accum_yaw_ = 0.0;
    prev_yaw_ = ctx->getCurrentPose().yaw;
    RCLCPP_INFO(ctx->node->get_logger(), "[ActionPassGate] Searching for Gate...");
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ActionPassGate::onRunning() {
    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);
    Pose cur = ctx->getCurrentPose();
    double ox, oy, oz;
    bool seen = ctx->getObjectPosition("GATE", ox, oy, oz);

    if (phase_ == Phase::SEARCH) {
        if (seen) { phase_ = Phase::ALIGN; align_start_time_ = 0.0; return BT::NodeStatus::RUNNING; }
        accum_yaw_ += std::abs(normalizeAngle(cur.yaw - prev_yaw_));
        prev_yaw_ = cur.yaw;
        if (accum_yaw_ >= 2.0 * M_PI) return BT::NodeStatus::FAILURE;
        ctx->publishToPico(0.5f, 0.0f, 0.0f, (float)ctx->target_depth, 0);
    } 
    else if (phase_ == Phase::ALIGN) {
        if (!seen) { phase_ = Phase::SEARCH; return BT::NodeStatus::RUNNING; }
        double norm_x = ox / std::max(oz, 0.5);
        if (std::abs(norm_x) < 0.04) {
            if (align_start_time_ == 0.0) align_start_time_ = ctx->node->get_clock()->now().seconds();
            if (ctx->node->get_clock()->now().seconds() - align_start_time_ >= 1.0) {
                phase_ = Phase::DRIVE;
                start_time_ = ctx->node->get_clock()->now().seconds();
                entry_pose_ = cur;
                RCLCPP_INFO(ctx->node->get_logger(), "[ActionPassGate] Driving through gate.");
            }
        } else {
            align_start_time_ = 0.0;
            ctx->publishToPico(-(float)norm_x * 0.8f, 0.0f, 0.0f, (float)ctx->target_depth, 0);
        }
    } 
    else if (phase_ == Phase::DRIVE) {
        if (ctx->node->get_clock()->now().seconds() - start_time_ >= (gate_depth_ + 2.0) / 0.5) {
            phase_ = Phase::STAY_STILL;
            start_time_ = ctx->node->get_clock()->now().seconds();
        } else {
            double yaw_err = normalizeAngle(entry_pose_.yaw - cur.yaw);
            ctx->publishToPico((float)yaw_err, 10.0f, 0.0f, (float)ctx->target_depth, 0);
        }
    }
    else {
        if (ctx->node->get_clock()->now().seconds() - start_time_ >= 3.0) {
            setOutput("entry_pose", entry_pose_);
            return BT::NodeStatus::SUCCESS;
        }
        ctx->stopMotion();
    }
    return BT::NodeStatus::RUNNING;
}

void ActionPassGate::onHalted() { getCtx(config())->stopMotion(); }

BT::NodeStatus ActionOrbitPole::onStart() {
    radius_ = getInput<double>("radius").value_or(2.0);
    phase_ = Phase::SEARCH;
    auto ctx = getCtx(config());
    accum_yaw_ = 0.0;
    prev_yaw_ = ctx->getCurrentPose().yaw;
    steps_completed_ = 0;
    RCLCPP_INFO(ctx->node->get_logger(), "[ActionOrbitPole] Searching for Pole...");
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
        accum_yaw_ += std::abs(normalizeAngle(cur.yaw - prev_yaw_));
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
        if (steps_completed_ >= 8) { phase_ = Phase::STAY_STILL; start_time_ = ctx->node->get_clock()->now().seconds(); return BT::NodeStatus::RUNNING; }
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
        if (ctx->node->get_clock()->now().seconds() - start_time_ >= 2.5) { phase_ = Phase::ORBIT_STEP_ALIGN; steps_completed_++; }
        else ctx->publishToPico(normalizeAngle(locked_yaw_ - cur.yaw) * 2.0f, 10.0f, 0.0f, (float)ctx->target_depth, 0);
    }
    else {
        if (ctx->node->get_clock()->now().seconds() - start_time_ >= 3.0) return BT::NodeStatus::SUCCESS;
        ctx->stopMotion();
    }
    return BT::NodeStatus::RUNNING;
}

void ActionOrbitPole::onHalted() { getCtx(config())->stopMotion(); }

BT::NodeStatus ActionReturnHome::onStart() {
    auto hp = getInput<Pose>("home_pose");
    if (!hp) return BT::NodeStatus::FAILURE;
    home_pose_ = hp.value();
    home_pose_.yaw = normalizeAngle(home_pose_.yaw + M_PI);
    transit_dur_ = getInput<double>("transit_duration").value_or(10.0);
    gate_depth_ = getInput<double>("gate_depth").value_or(4.0);
    phase_ = Phase::TRANSIT_TURN;
    RCLCPP_INFO(getCtx(config())->node->get_logger(), "[ActionReturnHome] Returning home...");
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
            accum_yaw_ += std::abs(normalizeAngle(cur.yaw - prev_yaw_)); prev_yaw_ = cur.yaw;
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
        if (ctx->node->get_clock()->now().seconds() - start_time_ >= (gate_depth_ + 10.0) / 0.5) {
            phase_ = Phase::STAY_STILL;
            start_time_ = ctx->node->get_clock()->now().seconds();
        } else {
            ctx->publishToPico(normalizeAngle(home_pose_.yaw - cur.yaw), 10.0f, 0.0f, (float)ctx->target_depth, 0);
        }
    }
    else {
        if (ctx->node->get_clock()->now().seconds() - start_time_ >= 3.0) return BT::NodeStatus::SUCCESS;
        ctx->stopMotion();
    }
    return BT::NodeStatus::RUNNING;
}

void ActionReturnHome::onHalted() { getCtx(config())->stopMotion(); }
