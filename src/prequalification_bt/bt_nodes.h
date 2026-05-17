#pragma once

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "custom_interfaces/msg/to_pico.hpp"

#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <chrono>

// ─── Waypoint type ────────────────────────────────────────────────────────────
struct Pose {
    double x = 0.0, y = 0.0, z = 0.0, yaw = 0.0;
};

namespace BT {
template <> inline Pose convertFromString(StringView str) {
    auto parts = splitString(str, ';');
    if (parts.size() != 4)
        throw RuntimeError("invalid Pose: expected 'x;y;z;yaw'");
    Pose p;
    p.x   = convertFromString<double>(parts[0]);
    p.y   = convertFromString<double>(parts[1]);
    p.z   = convertFromString<double>(parts[2]);
    p.yaw = convertFromString<double>(parts[3]);
    return p;
}
}  // namespace BT

// ─── Shared robot interface ───────────────────────────────────────────────────
struct RobotContext {
    rclcpp::Node::SharedPtr node;

    std::mutex mtx;

    sensor_msgs::msg::Imu::SharedPtr               latest_imu;
    double                                         latest_altimeter = 0.0;
    double                                         target_depth = 0.0;
    vision_msgs::msg::Detection3DArray::SharedPtr  latest_detections;

    bool imu_received  = false;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr     cmd_vel_pub;
    rclcpp::Publisher<custom_interfaces::msg::ToPico>::SharedPtr pico_pub;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr              imu_sub;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr             alt_sub;
    rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr det_sub;

    // ── Helpers ───────────────────────────────────────────────────────────────

    Pose getCurrentPose() {
        std::lock_guard<std::mutex> g(mtx);
        Pose p;
        p.x = 0.0;
        p.y = 0.0;
        p.z = latest_altimeter;

        if (latest_imu) {
            tf2::Quaternion q(latest_imu->orientation.x,
                              latest_imu->orientation.y,
                              latest_imu->orientation.z,
                              latest_imu->orientation.w);
            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
            p.yaw = yaw;
        }
        return p;
    }

    bool isObjectSeen(const std::string& object) {
        std::lock_guard<std::mutex> g(mtx);
        if (!latest_detections) return false;
        for (const auto& det : latest_detections->detections) {
            if (det.results.empty()) continue;
            const auto& hyp = det.results[0].hypothesis;
            const auto& id = hyp.class_id;
            const auto& score = hyp.score;

            if (object == "GATE" && id == "preq_gate") {
                if (score >= 0.6) return true;
                RCLCPP_DEBUG(node->get_logger(), "Saw GATE with low confidence: %.2f", score);
            }
            if (object == "POLE" && id == "preq_pole") {
                if (score >= 0.3) return true;
                RCLCPP_DEBUG(node->get_logger(), "Saw POLE with low confidence: %.2f", score);
            }
        }
        return false;
    }

    bool getObjectPosition(const std::string& object,
                           double& ox, double& oy, double& oz) {
        std::lock_guard<std::mutex> g(mtx);
        if (!latest_detections) return false;

        for (const auto& det : latest_detections->detections) {
            if (det.results.empty()) continue;
            const auto& hyp = det.results[0].hypothesis;
            const auto& id = hyp.class_id;
            const auto& score = hyp.score;

            if (object == "GATE" && id == "preq_gate" && score >= 0.6) {
                ox = det.bbox.center.position.x;
                oy = det.bbox.center.position.y;
                oz = det.bbox.center.position.z;
                return true;
            }
            if (object == "POLE" && id == "preq_pole" && score >= 0.3) {
                ox = det.bbox.center.position.x;
                oy = det.bbox.center.position.y;
                oz = det.bbox.center.position.z;
                return true;
            }
        }
        return false;
    }

    void publishToPico(float delta_yaw, float delta_d, float delta_s, float target_depth_val, uint8_t stop_bit) {
        custom_interfaces::msg::ToPico msg;
        msg.delta_yaw = delta_yaw;
        msg.delta_d = delta_d;
        msg.delta_s = delta_s;
        msg.target_depth = target_depth_val;
        msg.stop_bit = stop_bit;
        pico_pub->publish(msg);
    }

    void publishCmdVel(double surge, double sway, double heave,
                       double roll_r, double pitch_r, double yaw_r) {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x  = surge;
        cmd.linear.y  = sway;
        cmd.linear.z  = heave;
        cmd.angular.x = roll_r;
        cmd.angular.y = pitch_r;
        cmd.angular.z = yaw_r;
        cmd_vel_pub->publish(cmd);
    }

    void stopMotion() { 
        publishToPico(0.0f, 0.0f, 0.0f, (float)target_depth, 1); 
    }
};

// ─── Math utilities ───────────────────────────────────────────────────────────
inline double clampVal(double v, double lo, double hi) {
    return std::max(lo, std::min(hi, v));
}
inline double normalizeAngle(double a) {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

// ─── Node declarations ────────────────────────────────────────────────────────

class AllSystemsOK : public BT::ConditionNode {
public:
    AllSystemsOK(const std::string& name, const BT::NodeConfig& config)
        : BT::ConditionNode(name, config) {}
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus tick() override;
};

class SaveToBlackboard : public BT::SyncActionNode {
public:
    SaveToBlackboard(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}
    static BT::PortsList providedPorts() {
        return { BT::OutputPort<Pose>("key", "Blackboard key to store current pose") };
    }
    BT::NodeStatus tick() override;
};

class DiveToDepth : public BT::StatefulActionNode {
public:
    DiveToDepth(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config) {}
    static BT::PortsList providedPorts() {
        return { BT::InputPort<double>("target_depth", "Target z in metres") };
    }
    BT::NodeStatus onStart()   override;
    BT::NodeStatus onRunning() override;
    void           onHalted()  override;
private:
    double target_z_        = 0.0;
    double depth_tolerance_ = 0.15;
};

class IsObjectSeen : public BT::ConditionNode {
public:
    IsObjectSeen(const std::string& name, const BT::NodeConfig& config)
        : BT::ConditionNode(name, config) {}
    static BT::PortsList providedPorts() {
        return { BT::InputPort<std::string>("object", "GATE or POLE") };
    }
    BT::NodeStatus tick() override;
};

class Do360Turn : public BT::StatefulActionNode {
public:
    Do360Turn(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config) {}
    static BT::PortsList providedPorts() {
        return { BT::InputPort<std::string>("success_when_seen",
                     "Return SUCCESS when this object appears: GATE or POLE") };
    }
    BT::NodeStatus onStart()   override;
    BT::NodeStatus onRunning() override;
    void           onHalted()  override;
private:
    std::string target_object_;
    double      prev_yaw_        = 0.0;
    double      accumulated_yaw_ = 0.0;
    static constexpr double FULL_CIRCLE = 2.0 * M_PI;
};

class DriveThruGate : public BT::StatefulActionNode {
public:
    DriveThruGate(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config) {}
    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<double>("gate_depth",
                "How far to drive after gate disappears from view (metres)"),
            BT::OutputPort<Pose>("entry_pose",
                "Pose saved at gate entry — use as T1 blackboard key")
        };
    }
    BT::NodeStatus onStart()   override;
    BT::NodeStatus onRunning() override;
    void           onHalted()  override;
private:
    enum class Phase { ALIGN, DRIVE };
    Phase  phase_            = Phase::ALIGN;
    Pose   entry_pose_;
    double gate_depth_       = 6.0; // Increased from 3.0
    double start_time_       = 0.0;
    double align_start_time_ = 0.0;
    double gate_drive_time_  = 0.0;
    static constexpr double ALIGN_TOL  = 0.04; // Slightly tighter
};

class NavigateTo : public BT::StatefulActionNode {
public:
    NavigateTo(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config) {}
    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<Pose>("from",     "Start waypoint (informational)"),
            BT::InputPort<Pose>("to",       "Target waypoint"),
            BT::InputPort<bool>("reverse",  "If true, face opposite of target yaw (180 deg flip)"),
            BT::InputPort<double>("duration", "Timed surge duration in seconds")
        };
    }
    BT::NodeStatus onStart()   override;
    BT::NodeStatus onRunning() override;
    void           onHalted()  override;
    private:
    Pose   target_;
    double start_time_ = 0.0;
    double duration_   = 15.0;
    };

class NavigateAround : public BT::StatefulActionNode {
public:
    NavigateAround(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config) {}
    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("object",       "Object to orbit: POLE"),
            BT::InputPort<double>     ("threshold",    "Orbit radius in metres")
        };
    }
    BT::NodeStatus onStart()   override;
    BT::NodeStatus onRunning() override;
    void           onHalted()  override;
private:
    enum class Phase { ALIGN, TURN, SURGE };
    Phase phase_ = Phase::ALIGN;

    std::string target_object_;
    double      threshold_     = 1.5;
    int         steps_completed_ = 0;
    double      target_yaw_      = 0.0;
    double      locked_yaw_      = 0.0;
    double      start_time_      = 0.0;
    double      surge_duration_  = 2.5;
};

class ApproachObject : public BT::StatefulActionNode {
public:
    ApproachObject(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config) {}
    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("object",    "GATE or POLE"),
            BT::InputPort<double>     ("threshold", "Distance to stop at")
        };
    }
    BT::NodeStatus onStart()   override;
    BT::NodeStatus onRunning() override;
    void           onHalted()  override;
private:
    std::string target_object_;
    double      threshold_ = 2.0;
    double      locked_yaw_ = 0.0;
};

class AlignWithObject : public BT::StatefulActionNode {
public:
    AlignWithObject(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config) {}
    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("object", "GATE or POLE")
        };
    }
    BT::NodeStatus onStart()   override;
    BT::NodeStatus onRunning() override;
    void           onHalted()  override;
private:
    std::string target_object_;
};

class MoveRelatively : public BT::StatefulActionNode {
public:
    MoveRelatively(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config) {}
    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<double>("surge", "Forward error for P-controller"),
            BT::InputPort<double>("sway", "Rightward error for P-controller"),
            BT::InputPort<double>("duration", "Seconds to move")
        };
    }
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
private:
    double surge_ = 0.0, sway_ = 0.0, duration_ = 0.0;
    std::chrono::steady_clock::time_point start_time_;
};

class RelativeTurn : public BT::StatefulActionNode {
public:
    RelativeTurn(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config) {}
    static BT::PortsList providedPorts() {
        return { BT::InputPort<double>("angle", "Relative yaw in radians") };
    }
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
private:
    double target_yaw_ = 0.0;
    double angle_ = 0.0;
};

class StayStill : public BT::StatefulActionNode {
public:
    StayStill(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config) {}
    static BT::PortsList providedPorts() {
        return { BT::InputPort<double>("duration", "Seconds to stay still") };
    }
    BT::NodeStatus onStart()   override;
    BT::NodeStatus onRunning() override;
    void           onHalted()  override;
private:
    std::chrono::steady_clock::time_point start_time_;
    double duration_ = 2.0;
};

// ─── Consolidated Phase Nodes ───────────────────────────────────────────

class ActionInitialize : public BT::StatefulActionNode {
public:
    ActionInitialize(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config) {}
    static BT::PortsList providedPorts() {
        return { BT::InputPort<double>("target_depth", "Depth to dive to") };
    }
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
private:
    enum class Phase { CHECK, DIVE };
    Phase  phase_ = Phase::CHECK;
    double target_depth_ = 1.5;
};

class ActionPassGate : public BT::StatefulActionNode {
public:
    ActionPassGate(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config) {}
    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<double>("gate_depth", "Distance to drive after gate"),
            BT::OutputPort<Pose>("entry_pose", "Saved pose for return")
        };
    }
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
private:
    enum class Phase { SEARCH, ALIGN, DRIVE };
    Phase  phase_ = Phase::SEARCH;
    double gate_depth_ = 6.0;
    double start_time_ = 0.0;
    double align_start_time_ = 0.0;
    double accum_yaw_ = 0.0, prev_yaw_ = 0.0;
    Pose   entry_pose_;
};

class ActionOrbitPole : public BT::StatefulActionNode {
public:
    ActionOrbitPole(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config) {}
    static BT::PortsList providedPorts() {
        return { BT::InputPort<double>("radius", "Orbit radius in metres") };
    }
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
private:
    enum class Phase { SEARCH, ALIGN, APPROACH, ORBIT_STEP_ALIGN, ORBIT_STEP_TURN, ORBIT_STEP_SURGE };
    Phase  phase_ = Phase::SEARCH;
    double radius_ = 2.0;
    double accum_yaw_ = 0.0, prev_yaw_ = 0.0;
    double start_time_ = 0.0, target_yaw_ = 0.0, locked_yaw_ = 0.0;
    int    steps_completed_ = 0;
};

class ActionReturnHome : public BT::StatefulActionNode {
public:
    ActionReturnHome(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config) {}
    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<Pose>("home_pose", "The T1 pose to return to"),
            BT::InputPort<double>("transit_duration", "Blind surge time"),
            BT::InputPort<double>("gate_depth", "Final pass distance")
        };
    }
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
private:
    enum class Phase { TRANSIT_TURN, TRANSIT_SURGE, SEARCH, ALIGN, DRIVE };
    Phase  phase_ = Phase::TRANSIT_TURN;
    Pose   home_pose_;
    double transit_dur_ = 10.0, gate_depth_ = 4.0;
    double start_time_ = 0.0, align_start_time_ = 0.0;
    double accum_yaw_ = 0.0, prev_yaw_ = 0.0;
};

inline void registerAllNodes(BT::BehaviorTreeFactory& factory) {
    factory.registerNodeType<AllSystemsOK>("AllSystemsOK");
    factory.registerNodeType<SaveToBlackboard>("SaveToBlackboard");
    factory.registerNodeType<DiveToDepth>("DiveToDepth");
    factory.registerNodeType<IsObjectSeen>("IsObjectSeen");
    factory.registerNodeType<Do360Turn>("Do360Turn");
    factory.registerNodeType<DriveThruGate>("DriveThruGate");
    factory.registerNodeType<NavigateTo>("NavigateTo");
    factory.registerNodeType<NavigateAround>("NavigateAround");
    factory.registerNodeType<ApproachObject>("ApproachObject");
    factory.registerNodeType<AlignWithObject>("AlignWithObject");
    factory.registerNodeType<MoveRelatively>("MoveRelatively");
    factory.registerNodeType<RelativeTurn>("RelativeTurn");
    factory.registerNodeType<StayStill>("StayStill");

    // Consolidated nodes
    factory.registerNodeType<ActionInitialize>("ActionInitialize");
    factory.registerNodeType<ActionPassGate>("ActionPassGate");
    factory.registerNodeType<ActionOrbitPole>("ActionOrbitPole");
    factory.registerNodeType<ActionReturnHome>("ActionReturnHome");
}
