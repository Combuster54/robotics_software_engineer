#ifndef ROBOT_TEMPLATE
#define ROBOT_TEMPLATE

#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace robotNameSpace {

/**
 * @typedef CmdVel
 * Alias for geometry_msgs::msg::Twist.
 */
using CmdVel = geometry_msgs::msg::Twist;

/**
 * @typedef imgMsg
 * Alias for sensor_msgs::msg::Image.
 */
using imgMsg = sensor_msgs::msg::Image;

using odomMsg = nav_msgs::msg::Odometry;

using scanMsg = sensor_msgs::msg::LaserScan;

/**
 * @struct RobotPose
 * @brief Structure for storing robot pose information in 2D space.
 */
struct RobotPose {
    double x;
    double y;
    double theta;
};

/**
 * @struct Goal
 * @brief Structure for storing goal pose information in 2D space.
 */
struct Goal {
    double x_goal;
    double y_goal;
    double theta_goal;
};

} // namespace robot_types

#endif // ROBOT_TEMPLATE