#ifndef LQR_NODE_HPP
#define LQR_NODE_HPP

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <lqr_lib.hpp>
#include <vector>

struct State {

    double x;
    double y;
    double theta;

    State() = default;
    State(double x_, double y_, double theta_) : x(x_), y(y_), theta(theta_){}
};

struct Input {

    double v;
    double w;

    Input() = default;
    Input(double v_, double w_) : v(v_), w(w_){}
};

class LqrNode : public rclcpp::Node {
public:
    LqrNode();

private:
    void robotPoseCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void controlLoopCallback();
    void publishVelocity(double v, double w);
    void optimiseHeading(std::vector<State>& waypoints);
    void angleNormalisation(double& angle);

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr robot_pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr control_input_pub_;
    rclcpp::TimerBase::SharedPtr control_loop_timer_;

    Eigen::MatrixXd Q_;
    Eigen::MatrixXd R_;
    Eigen::Vector3d state_error_;

    double dt_;
    double tolerance_;
    bool end_controller_;
    double max_linear_velocity_;
    double max_angular_velocity_;
    State actual_state_;
    Input control_input_;

    std::vector<State> waypoints_;
    int current_waypoint_;
    bool odom_received_;
    std::unique_ptr<LQR> lqr_;

};

#endif
