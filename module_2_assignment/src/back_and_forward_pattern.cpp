#include "backward_and_forward_pattern.hpp"
#include "rclcpp/rclcpp.hpp"

inline constexpr char CustomNodeName1[]  = "turtle1_node";
inline constexpr char CustomTopicName1[] = "/turtle1/cmd_vel";

inline constexpr char CustomNodeName2[] = "turtle4_node";
inline constexpr char CustomTopicName2[] = "/turtle4/cmd_vel";

inline constexpr char CustomNodeName3[] = "turtle3_node";
inline constexpr char CustomTopicName3[] = "/turtle3/cmd_vel";

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node1 = std::make_shared<movement_patterns::BackandFort<CustomNodeName1, CustomTopicName1>>();
    auto node2 = std::make_shared<movement_patterns::BackandFort<CustomNodeName2, CustomTopicName2>>();
    auto node3 = std::make_shared<movement_patterns::BackandFort<CustomNodeName3, CustomTopicName3>>();


    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node1);
    executor.add_node(node2);
    executor.add_node(node3);

    executor.spin();
    rclcpp::shutdown();
    return 0;
}