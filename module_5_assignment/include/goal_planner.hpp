#ifndef OPTIMIZE_GOAL_HPP
#define OPTIMIZE_GOAL_HPP

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <memory>
#include "robot_template.hpp"

using robotNameSpace::RobotPose;
using robotNameSpace::Goal;

/*
 * @class GoalPlanner
 * @brief ROS 2 Node for selecting optimal goals and path planning for a robot.
 *
 * This class implements the logic for selecting the closest goal, planning the shortest
 * path, and commanding the robot to move efficiently towards the selected goal.
 */
class GoalPlanner : public rclcpp::Node {
public:
    /**
     * @brief Constructor for the GoalPlanner Node.
     */
    GoalPlanner();

private:
    void declare_parameters();
    void init_publishers();
    void init_subscribers();
    void update_parameters();

    void GoalPlanner::odom_callback(const odomMsg::SharedPtr odom_msg);
    void GoalPlanner::scanCallback(const scanMsg::SharedPtr scan_msg);
    /**
     * @brief Plans the optimal path from the robot's position to a goal.
     * @param robot_position The robot's current position.
     * @param goal The target goal.
     */
    void planning_path(RobotPose robot_position, Goal goal);

    /**
     * @brief Selects the closest goal to the robot's position.
     * @param robot_position The current position of the robot based on /odom topic.
     * @param goals A set of potential goal points.
     * @return The closest goal based on Euclidean distance.
     */
    Goal choose_closest_goal(RobotPose robot_position, std::vector<Goal> goals);

    rclcpp::Publisher<CmdVel>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<scanMsg>::SharedPtr laser_scan_sub_;
    geometry_msgs::msg::Twist cmd_vel_;

};

#endif  // OPTIMIZE_GOAL_HPP