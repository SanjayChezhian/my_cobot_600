#include <memory>
#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <std_msgs/msg/bool.hpp>
#include <chrono> // For introducing delays

struct Waypoint {
    double x, y, z, roll, pitch, yaw;
};

std::vector<Waypoint> parseWaypointsFromFile(const std::string &file_path) {
    std::vector<Waypoint> waypoints;
    std::ifstream input_file(file_path);

    if (!input_file.is_open()) {
        throw std::runtime_error("Failed to open file: " + file_path);
    }

    std::string line;
    while (std::getline(input_file, line)) {
        if (line.empty() || line.find_first_not_of(" \t\n\r") == std::string::npos) {
            continue;
        }

        std::istringstream line_stream(line);
        Waypoint wp;

        if (line_stream >> wp.x >> wp.y >> wp.z >> wp.roll >> wp.pitch >> wp.yaw) {
            waypoints.push_back(wp);
        } else {
            RCLCPP_WARN(rclcpp::get_logger("waypoint_loader"), "Ignoring malformed line: %s", line.c_str());
        }
    }

    input_file.close();
    return waypoints;
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
        "waypoint_navigation_node",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    auto logger = rclcpp::get_logger("waypoint_navigation_node");

    auto status_publisher = node->create_publisher<std_msgs::msg::Bool>("waypoint_status", 10);

    if (argc < 2) {
        RCLCPP_ERROR(logger, "Usage: %s <waypoints_file>", argv[0]);
        return 1;
    }

    std::string file_path = argv[1];
    std::vector<Waypoint> waypoints;

    try {
        waypoints = parseWaypointsFromFile(file_path);
        RCLCPP_INFO(logger, "Loaded %lu waypoints successfully.", waypoints.size());
    } catch (const std::exception &e) {
        RCLCPP_ERROR(logger, "Error reading file: %s", e.what());
        return 1;
    }

    using moveit::planning_interface::MoveGroupInterface;
    MoveGroupInterface move_group(node, "mycobot_arm");

    move_group.setPoseReferenceFrame("base");
    move_group.setStartStateToCurrentState();
    move_group.setPlanningTime(20.0);

    for (const auto &wp : waypoints) {
        tf2::Quaternion quaternion;
        quaternion.setRPY(wp.roll, wp.pitch, wp.yaw);

        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.frame_id = "base";
        target_pose.pose.position.x = wp.x;
        target_pose.pose.position.y = wp.y;
        target_pose.pose.position.z = wp.z;
        target_pose.pose.orientation.x = quaternion.x();
        target_pose.pose.orientation.y = quaternion.y();
        target_pose.pose.orientation.z = quaternion.z();
        target_pose.pose.orientation.w = quaternion.w();

        move_group.setPoseTarget(target_pose);
        moveit::planning_interface::MoveGroupInterface::Plan motion_plan;
        bool plan_success = (move_group.plan(motion_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        if (plan_success) {
            move_group.move();
            std::cout << "Reached waypoint: (" << wp.x << ", " << wp.y << ", " << wp.z << ")\n";

            std_msgs::msg::Bool status_msg;
            status_msg.data = true;
            status_publisher->publish(status_msg);
        } else {
            RCLCPP_ERROR(logger, "Failed to reach waypoint: (%f, %f, %f).", wp.x, wp.y, wp.z);
            break;
        }

        std::cout << "Pausing for 1 second before next waypoint...\n";
        rclcpp::sleep_for(std::chrono::seconds(1));
    }

    rclcpp::shutdown();
    return 0;
}
