/**
 * @file wall_follower.hpp
 * @brief Header file for the WallFollower class
 *
 * This file defines the WallFollower class, which implements a simple wall-following algorithm
 * using data from a laser scanner. The robot adjusts its movement based on obstacle distances
 * detected at the front, right, and left.
 */

#ifndef WALL_FOLLOWER_HPP_
#define WALL_FOLLOWER_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <algorithm>

using scanMsg = sensor_msgs::msg::LaserScan;

/**
 * @enum RobotState
 * @brief Represents the possible states of the robot during wall-following.
 */
enum class RobotState {
    MOVING_STRAIGHT, ///< The robot is moving straight.
    TURNING_LEFT,    ///< The robot is turning left.
    TURNING_RIGHT,   ///< The robot is turning right.
    TURN
};

/**
 * @class WallFollower
 * @brief A ROS 2 node that implements a wall-following algorithm.
 */
class WallFollower : public rclcpp::Node {
public:
    /**
     * @brief Constructor for the WallFollower class.
     * Initializes publishers, subscribers, and logs the node readiness.
     */
    WallFollower();

private:
    // ROS publishers and subscribers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_; ///< Publisher for velocity commands.
    rclcpp::Subscription<scanMsg>::SharedPtr laser_scan_sub_; ///< Subscriber for laser scan data.

    // Current command velocity
    geometry_msgs::msg::Twist cmd_vel_;

    // Distance to obstacles
    double front_obstacle; ///< Minimum distance to obstacles in front of the robot.
    double right_obstacle; ///< Minimum distance to obstacles to the right of the robot.
    double left_obstacle;  ///< Minimum distance to obstacles to the left of the robot.

    // Current state of the robot
    RobotState state_;

    /**
     * @brief Callback function for processing laser scan data.
     * Analyzes obstacles, determines the robot's state, and publishes velocity commands.
     * @param msg Shared pointer to the laser scan message.
     */
    void scanCallback(const scanMsg::SharedPtr msg);

    /**
     * @brief Analyzes the distances to obstacles based on laser scan data.
     * Extracts minimum distances for the front, right, and left sections.
     * @param msg Shared pointer to the laser scan message.
     */
    void analyzeObstacles(const scanMsg::SharedPtr msg);

    /**
     * @brief Determines the robot's state based on obstacle distances.
     * Updates the state_ variable to reflect the robot's behavior.
     */
    void determineState();

    /**
     * @brief Publishes velocity commands based on the robot's state.
     * Adjusts linear and angular velocities to execute the desired behavior.
     */
    void publishVelocity();
};

#endif // WALL_FOLLOWER_HPP_
