#include <angles/angles.h>
#include <cmath>
#include <rclcpp/logging.hpp>
#include <tuple>
#include <vector>
#include "lqr_node.hpp"

Input input_old = Input{0,0};

LqrNode::LqrNode() 
    : Node("LqrNode"), 
      dt_(0.03), 
      tolerance_(0.8), 
      end_controller_(false), 
      max_linear_velocity_(0.8), 
      max_angular_velocity_(M_PI / 2), 
      current_waypoint_(0), 
      odom_received_(false) 
{
    robot_pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&LqrNode::robotPoseCallback, this, std::placeholders::_1));
    
    control_input_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    
    control_loop_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(30), std::bind(&LqrNode::controlLoopCallback, this));

    Q_ = Eigen::MatrixXd::Zero(3, 3);
    R_ = Eigen::MatrixXd::Zero(2, 2);

    // Q_ << 0.8, 0, 0, 0, 0, 0, 0, 0.8;
    // R_ << 0.8, 0, 0, 0.8;

    // // First
    // Q_ << 1.0, 0, 0,
    //     0, 1.0, 0,
    //     0, 0, 1.0;

    // R_ << 0.5, 0,
    //     0, 0.5;

    // // Second
    // Q_ << 10.0, 0, 0,
    //     0, 10.0, 0,
    //     0, 0, 10.0;

    // R_ << 0.2, 0,
    //     0, 0.2;

    // Third
    Q_ << 0.3, 0, 0,
        0, 0.3, 0,
        0, 0, 0.3;

    R_ << 10.0, 0,
        0, 10.0;

    lqr_ = std::make_unique<LQR>(Q_, R_, 100);

    // Diagonal positive
    waypoints_ = {
        State(0, 0, M_PI / 4), State(1, 1, M_PI / 2),
        State(2, 2, M_PI), State(3, 3, 3 * M_PI / 2),
        State(4, 4, M_PI), State(5,5, -M_PI / 2),
        State(6, 6, M_PI), State(7, 7, M_PI / 2)
    };

    // // Rectangle
    // waypoints_ = {
    //     State(0, 0, M_PI / 4), State(1, 1, M_PI / 2),
    //     State(2, 2, M_PI), State(-1, 1, 3 * M_PI / 2),
    //     State(-1, -1, M_PI), State(1, -1, -M_PI / 2),
    //     State(0, 0, M_PI / 4)
    // };

    actual_state_ = State(0, 0, 0);

    optimiseHeading(waypoints_);
}

void LqrNode::robotPoseCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
    );
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    actual_state_ = State(
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        yaw
    );
    odom_received_ = true;
}

void LqrNode::publishVelocity(double v, double w) {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = v;
    msg.angular.z = w;
    RCLCPP_INFO(rclcpp::get_logger("LQR"), "Publishing control input: v=%f, w=%f", v, w);
    control_input_ = Input(v, w);
    input_old = Input(v, w);
    control_input_pub_->publish(msg);
}

void LqrNode::optimiseHeading(std::vector<State>& waypoints) {
    for (size_t i = 0; i < waypoints.size() - 1; ++i) {
        double dx = waypoints[i + 1].x - waypoints[i].x;
        double dy = waypoints[i + 1].y - waypoints[i].y;
        waypoints[i].theta = std::atan2(dy, dx);
    }
    waypoints.back().theta = waypoints[waypoints.size() - 2].theta;
}

void LqrNode::controlLoopCallback() {
    if (!odom_received_) {
        RCLCPP_INFO(rclcpp::get_logger("LQR"), "Waiting for odometry message...");
        return;
    }

    if (end_controller_) {
        RCLCPP_INFO(rclcpp::get_logger("LQR"), "Goal reached!");
        control_loop_timer_->cancel();
        return;
    }

    State desired_state = waypoints_[current_waypoint_];
    Eigen::Vector3d x_actual(actual_state_.x, actual_state_.y, actual_state_.theta);
    Eigen::Vector3d x_desired(desired_state.x, desired_state.y, desired_state.theta);
    state_error_ = x_actual - x_desired;

    if (current_waypoint_ == 2) {
        waypoints_[current_waypoint_ + 1] = State(-1, 3, M_PI);
    }

    RCLCPP_INFO(rclcpp::get_logger("LQR"), "Current Waypoint: %d", current_waypoint_);
    RCLCPP_INFO(rclcpp::get_logger("LQR"), "Actual state: x=%f, y=%f, theta=%f", 
                x_actual(0), x_actual(1), x_actual(2));
    RCLCPP_INFO(rclcpp::get_logger("LQR"), "Desired state: x=%f, y=%f, theta=%f", 
                x_desired(0), x_desired(1), x_desired(2));
    RCLCPP_INFO(rclcpp::get_logger("LQR"), "State error: x=%f, y=%f, theta=%f", 
                state_error_(0), state_error_(1), state_error_(2));
    RCLCPP_INFO(rclcpp::get_logger("LQR"), "Current goal: x=%f, y=%f, theta=%f", 
                waypoints_[current_waypoint_].x, waypoints_[current_waypoint_].y, 
                waypoints_[current_waypoint_].theta);

    auto A = lqr_->getA(actual_state_.theta, control_input_.v, dt_);
    auto B = lqr_->getB(actual_state_.theta, dt_);
    lqr_->updateMatrices(A, B);
    lqr_->computeRiccati(B, A);

    auto u = lqr_->computeOptimalInput(state_error_);

    Eigen::EigenSolver<Eigen::MatrixXd> solver(B * lqr_->K_ + A);
    auto eigenValues = solver.eigenvalues().real();
    RCLCPP_INFO(rclcpp::get_logger("LQR"), "Eigenvalues: %f, %f, %f", 
                eigenValues(0), eigenValues(1), eigenValues(2));

    publishVelocity(
        std::clamp(u(0), -max_linear_velocity_, max_linear_velocity_),
        std::clamp(u(1), -max_angular_velocity_, max_angular_velocity_)
    );
    if (state_error_.norm() < tolerance_) {
        RCLCPP_INFO(rclcpp::get_logger("LQR"), "Waypoint reached!");
        current_waypoint_++;
        if (current_waypoint_ >= static_cast<int>(waypoints_.size())) {
            end_controller_ = true;
            publishVelocity(0.0, 0.0);
        }
    } 
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto controller = std::make_shared<LqrNode>();
    rclcpp::spin(controller);
    rclcpp::shutdown();
    return 0;
}