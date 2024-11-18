#ifndef MOVEMENT_PATTERNS_BACK_AND_FORWARD_PATTERN_HPP
#define MOVEMENT_PATTERNS_BACK_AND_FORWARD_PATTERN_HPP

#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;
using CmdVel = geometry_msgs::msg::Twist;

namespace movement_patterns {

template <const char *NodeName, const char *TopicName>
class BackandFort : public rclcpp::Node
{
public:
    explicit BackandFort()
        : Node(NodeName)
    {

        this->declare_parameter<double>("l_velocity", 0.10);
        update_radius_from_param();

        publisher_ = this->create_publisher<CmdVel>(TopicName, 10);
        timer_ = this->create_wall_timer(500ms, std::bind(&BackandFort::timer_callback, this));
    }

private:


    void update_radius_from_param()
    {
        // Get the actuall value of l_velocity
        this->get_parameter("l_velocity", linear_speed_);
    }

    void timer_callback()
    {
        update_radius_from_param();
        auto message = CmdVel();
        message.linear.x = linear_speed_ * self_direction ;

        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Publishing: linear=%.2f, angular=%.2f", linear_speed_, angular_speed_);
        self_direction *= -1;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<CmdVel>::SharedPtr publisher_;
    double linear_speed_;
    double angular_speed_;
    int self_direction = 1;
};

// Static variables to define default node and topic names
inline constexpr char DefaultNodeName[] = "back_and_forward_pattern_publisher";
inline constexpr char DefaultTopicName[] = "/cmd_vel";

} // namespace movement_patterns

#endif // MOVEMENT_PATTERNS_BACK_AND_FORWARD_PATTERN_HPP
