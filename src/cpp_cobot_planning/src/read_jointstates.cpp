#include <memory>
#include <iostream>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <cmath> // For radian-to-degree conversion
#include <chrono> // For timestamp generation
#include <filesystem>

namespace fs = std::filesystem;

class JointStateRecorder : public rclcpp::Node {
public:
    JointStateRecorder() : Node("joint_state_recorder"), waypoint_reached_(false) {
        // Subscribe to waypoint notifications
        waypoint_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
            "waypoint_reached", 10,
            std::bind(&JointStateRecorder::waypointReachedCallback, this, std::placeholders::_1)
        );

        // Subscribe to joint state updates
        joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&JointStateRecorder::jointStateCallback, this, std::placeholders::_1)
        );

        // Ensure directory exists for joint state files
        const std::string output_dir = "/home/sanjay/Desktop/cobot600";
        if (!fs::exists(output_dir)) {
            fs::create_directory(output_dir);
        }

        // Generate unique filename with timestamp
        auto current_time = std::chrono::system_clock::now();
        auto time_c = std::chrono::system_clock::to_time_t(current_time);
        std::tm *local_time = std::localtime(&time_c);

        char formatted_time[100];
        std::strftime(formatted_time, sizeof(formatted_time), "%Y-%m-%d_%H-%M-%S", local_time);

        std::string file_path = output_dir + "/joint_states_values.txt";
        
        // Open file for writing
        output_file_.open(file_path, std::ios::app);
        if (!output_file_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Unable to open file for saving joint states.");
        } else {
            RCLCPP_INFO(this->get_logger(), "Joint state file opened successfully: %s", file_path.c_str());
        }
    }

    ~JointStateRecorder() {
        if (output_file_.is_open()) {
            output_file_.close();
            RCLCPP_INFO(this->get_logger(), "Joint state file closed.");
        }
    }

private:
    bool waypoint_reached_;
    std::ofstream output_file_;

    void waypointReachedCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data && !waypoint_reached_) {
            std::cout << "Waypoint reached. Preparing to log joint states.\n";
            waypoint_reached_ = true;
        }
    }

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (waypoint_reached_ && !msg->position.empty()) {
            // Convert joint positions to degrees and log them
            std::cout << "Joint positions (degrees): ";
            for (const auto &position_rad : msg->position) {
                double position_deg = position_rad * (180.0 / M_PI);
                std::cout << position_deg << " ";

                if (output_file_.is_open()) {
                    output_file_ << position_deg << " ";
                }
            }
            std::cout << "\n";

            if (output_file_.is_open()) {
                output_file_ << "\n";
            }

            waypoint_reached_ = false; // Reset after logging
        }
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr waypoint_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointStateRecorder>());
    rclcpp::shutdown();
    return 0;
}
