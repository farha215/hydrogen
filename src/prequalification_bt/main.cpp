#include "bt_nodes.h"
#include <behaviortree_cpp/xml_parsing.h>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <fstream>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // ── ROS2 node ──────────────────────────────────────────────────────────────
    auto node = std::make_shared<rclcpp::Node>("prequalification_bt");

    // ── Shared robot context ───────────────────────────────────────────────────
    auto ctx       = std::make_shared<RobotContext>();
    ctx->node      = node;

    // Actuator publishers
    ctx->cmd_vel_pub =
        node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    ctx->pico_pub =
        node->create_publisher<custom_interfaces::msg::ToPico>("/to_pico", 10);

    // /imu   — orientation / yaw (BNO055, 10 Hz)
    ctx->imu_sub =
        node->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10,
            [ctx](const sensor_msgs::msg::Imu::SharedPtr msg) {
                std::lock_guard<std::mutex> g(ctx->mtx);
                ctx->latest_imu   = msg;
                ctx->imu_received = true;
            });

    // /altimeter — altitude above pool floor (Float64, from Gazebo bridge)
    ctx->alt_sub =
        node->create_subscription<std_msgs::msg::Float64>(
            "/altimeter", 10,
            [ctx](const std_msgs::msg::Float64::SharedPtr msg) {
                std::lock_guard<std::mutex> g(ctx->mtx);
                ctx->latest_altimeter = msg->data;
            });

    // /detections_3d — 3-D bounding boxes from data_distance_node (YOLO + depth fusion)
    ctx->det_sub =
        node->create_subscription<vision_msgs::msg::Detection3DArray>(
            "/detections_3d", 10,
            [ctx](const vision_msgs::msg::Detection3DArray::SharedPtr msg) {
                std::lock_guard<std::mutex> g(ctx->mtx);
                ctx->latest_detections = msg;
            });

    // ── BT factory and node registration ──────────────────────────────────────
    BT::BehaviorTreeFactory factory;
    registerAllNodes(factory);
    // ── Dump TreeNodesModel XML for Groot2 ────────────────────────────────────
{
    std::string xml_models = BT::writeTreeNodesModelXML(factory);
    std::ofstream model_file("bt_nodes_model.xml");
    model_file << xml_models;
    RCLCPP_INFO(node->get_logger(), "[main] Written bt_nodes_model.xml for Groot2");
}

    // ── Locate the XML tree file ───────────────────────────────────────────────
    std::string xml_path;
    if (argc > 1) {
        xml_path = argv[1];
    } else {
        try {
            xml_path = ament_index_cpp::get_package_share_directory("prequalification_bt")
                       + "/prequalification.xml";
        } catch (const std::exception& e) {
            RCLCPP_FATAL(node->get_logger(),
                         "Cannot find prequalification.xml: %s\n"
                         "Build and source the package, or pass the path as argv[1].",
                         e.what());
            rclcpp::shutdown();
            return 1;
        }
    }
    RCLCPP_INFO(node->get_logger(), "Loading behaviour tree: %s", xml_path.c_str());

    auto tree = factory.createTreeFromFile(xml_path);

    // ── Inject shared context into the blackboard ──────────────────────────────
    tree.rootBlackboard()->set("robot_context", ctx);

    // ── Pre-set waypoints (replace with measured field coordinates) ───────────
    // Note: Waypoints like T1 are saved dynamically by the behavior tree nodes at runtime.
    // T1: staging point between gate and pole (through the gate)
    // T2: exit waypoint on the far side (back through the gate toward home)
    Pose t1, t2;
    t1.x = 5.0;  t1.y = 0.0;  t1.z = 2.5;  t1.yaw = 0.0;
    t2.x = 1.5;  t2.y = 0.0;  t2.z = 2.5;  t2.yaw = 0.0;
    tree.rootBlackboard()->set("T1", t1);
    tree.rootBlackboard()->set("T2", t2);

    // ── Seed callbacks before first tick ──────────────────────────────────────
    rclcpp::spin_some(node);

    // ── Tick loop (10 Hz) ─────────────────────────────────────────────────────
    RCLCPP_INFO(node->get_logger(), "=== Starting Pre-Qualification Maneuver ===");

    constexpr auto TICK_PERIOD = std::chrono::milliseconds(100);
    BT::NodeStatus status      = BT::NodeStatus::RUNNING;

    while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
        status = tree.tickOnce();
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(TICK_PERIOD);
    }

    ctx->stopMotion();

    if (status == BT::NodeStatus::SUCCESS) {
        RCLCPP_INFO(node->get_logger(), "=== Pre-Qualification COMPLETE ===");
    } else {
        RCLCPP_WARN(node->get_logger(), "=== Pre-Qualification FAILED ===");
    }

    rclcpp::shutdown();
    return (status == BT::NodeStatus::SUCCESS) ? 0 : 1;
}
