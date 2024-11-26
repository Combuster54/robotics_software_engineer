/**
 * @file wall_follower.cpp
 * @brief Implementation of the WallFollower class
 *
 * This file contains the implementation of the WallFollower class, including the logic
 * for processing laser scan data, determining the robot's state, and publishing velocity commands.
 */

#include "wall_follower.hpp"

using std::placeholders::_1;

/**
 * @brief Constructor for the WallFollower class.
 */
WallFollower::WallFollower() : Node("wall_follower_node"), front_obstacle(0.0), right_obstacle(0.0), left_obstacle(0.0), state_(RobotState::MOVING_STRAIGHT) {
    // Initialize publisher for velocity commands
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Initialize subscriber for laser scan data
    laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&WallFollower::scanCallback, this, _1));

    RCLCPP_INFO(this->get_logger(), "Wall follower is ready");
}

/**
 * @brief Callback function for processing laser scan data.
 */
void WallFollower::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    analyzeObstacles(msg);
    determineState();
    publishVelocity();
}

/**
 * @brief Analyzes the distances to obstacles based on laser scan data.
 */
void WallFollower::analyzeObstacles(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    double front_distance_1 = *std::min_element(msg->ranges.begin() + 0, msg->ranges.begin() + 45);
    double front_distance_2 = *std::min_element(msg->ranges.begin() + 316, msg->ranges.begin() + 360);
    front_obstacle  = std::min(front_distance_1, front_distance_2);
    right_obstacle  = *std::min_element(msg->ranges.begin() + 225, msg->ranges.begin() + 315);
    left_obstacle = *std::min_element(msg->ranges.begin() + 46, msg->ranges.begin() + 135);

    RCLCPP_INFO(this->get_logger(), "Left: '%f' Front: '%f' Right: '%f'",
                left_obstacle, front_obstacle, right_obstacle);

}

/**
 * @brief Determines the robot's state based on obstacle distances.
 */
void WallFollower::determineState() {
    if ((0.40 >= right_obstacle && right_obstacle >= 0.45) && front_obstacle > 0.70) {
        state_ = RobotState::MOVING_STRAIGHT;
    } else if (front_obstacle < 0.70) {
        state_ = RobotState::TURN;
    } else if (right_obstacle > 0.45) {
        state_ = RobotState::TURNING_RIGHT;
    } else if (right_obstacle < 0.40) {
        state_ = RobotState::TURNING_LEFT;
    }
}

/**
 * @brief Publishes velocity commands based on the robot's state.
 */
void WallFollower::publishVelocity() {
    switch (state_) {
        case RobotState::MOVING_STRAIGHT:
            cmd_vel_.linear.x = 0.20;
            cmd_vel_.angular.z = 0.0;
            RCLCPP_INFO(this->get_logger(), "Moving straight");
            break;

        case RobotState::TURN:
            cmd_vel_.linear.x = 0.10;
            cmd_vel_.angular.z = 0.5;
            RCLCPP_INFO(this->get_logger(), "Turn");
            break;

        case RobotState::TURNING_LEFT:
            cmd_vel_.linear.x = 0.15;
            cmd_vel_.angular.z = 0.35;
            RCLCPP_INFO(this->get_logger(), "Turning left");
            break;

        case RobotState::TURNING_RIGHT:
            cmd_vel_.linear.x = 0.15;
            cmd_vel_.angular.z = -0.35;
            RCLCPP_INFO(this->get_logger(), "Turning right");
            break;
    }

    // Publish the command velocity
    cmd_vel_pub_->publish(cmd_vel_);
}

/**
 * @brief Main function for the WallFollower node.
 */
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallFollower>());
    rclcpp::shutdown();
    return 0;
}