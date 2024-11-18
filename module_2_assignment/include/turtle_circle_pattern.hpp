#ifndef MOVEMENT_PATTERNS_CIRCLE_PATTERN_HPP
#define MOVEMENT_PATTERNS_CIRCLE_PATTERN_HPP

#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;
using CmdVel = geometry_msgs::msg::Twist;

namespace movement_patterns {

template <const char *NodeName, const char *TopicName>
class CirclePatternPublisher : public rclcpp::Node
{
public:
    explicit CirclePatternPublisher()
        : Node(NodeName)
    {
        // Declare radius parameter
        this->declare_parameter<double>("radius", 0.10);
        update_radius_from_param();

        publisher_ = this->create_publisher<CmdVel>(TopicName, 10);
        timer_ = this->create_wall_timer(500ms, std::bind(&CirclePatternPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        update_radius_from_param();
        auto message = CmdVel();
        message.linear.x = linear_speed_;
        message.angular.z = angular_speed_;

        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Publishing: linear=%.2f, angular=%.2f, radius=%.2f", linear_speed_, angular_speed_, radius_);
    }

    void update_radius_from_param()
    {
        // Obtener el valor actual del parámetro "radius"
        this->get_parameter("radius", radius_);
        angular_speed_ = linear_speed_ / radius_;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<CmdVel>::SharedPtr publisher_;
    double linear_speed_ = 0.5;
    double angular_speed_;
    double radius_;
};

inline constexpr char DefaultNodeName[] = "circle_pattern_publisher";
inline constexpr char DefaultTopicName[] = "/cmd_vel";

} // namespace movement_patterns

#endif // MOVEMENT_PATTERNS_CIRCLE_PATTERN_HPP
