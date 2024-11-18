#include "turtle_circle_pattern.hpp"
#include "rclcpp/rclcpp.hpp"

// Define custom node and topic names
inline constexpr char CustomNodeName[] = "turtle1_node";
inline constexpr char CustomTopicName[] = "/turtle1/cmd_vel";

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<movement_patterns::CirclePatternPublisher<
        CustomNodeName, CustomTopicName>>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
