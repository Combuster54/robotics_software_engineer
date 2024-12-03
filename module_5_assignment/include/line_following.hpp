/*
 * @file line_following.hpp
 * @brief Header file for the LineFollowing ROS 2 Node.
 *
 * This header defines the LineFollowing class, which implements a ROS 2 Node
 * for line-following behavior using image processing with OpenCV.
 *
 * The class processes camera images to detect edges, calculate the center of a line,
 * and publishes velocity commands for the robot.
 *
 * @date 25/11/2024
 * @license LGPL v2.1
 */

#ifndef LINE_FOLLOWING_HPP_
#define LINE_FOLLOWING_HPP_

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <memory>

// Type aliases for message types
using CmdVel = geometry_msgs::msg::Twist;
using imgMsg = sensor_msgs::msg::Image;

/**
 * @struct EdgeCountResult
 * @brief Structure for storing edge detection results.
 */
struct EdgeCountResult {
    int count_1_edge;  ///< Count of single edge detections
    int count_2_edge;  ///< Count of two-edge detections
    int count_3_edge;  ///< Count of three-edge detections
    int count_4_edge;  ///< Count of four-edge detections
    std::vector<int> edges; ///< List of detected edge positions
};

/**
 * @class LineFollowing
 * @brief ROS 2 Node for line-following using image processing.
 *
 * This class subscribes to camera images, processes them to detect lines, calculates
 * the midpoint of the line and robot position, and publishes velocity commands
 * to follow the detected line.
 */
class LineFollowing : public rclcpp::Node {
public:
    /**
     * @brief Constructor for the LineFollowing Node.
     */
    LineFollowing();

private:
    // Parameters for image processing
    int lower_threshold_;   ///< Lower threshold for Canny edge detection
    int upper_threshold_;   ///< Upper threshold for Canny edge detection
    int line_extraction_;   ///< Row for extracting the line in the image
    int blur_width_;        ///< Width of the Gaussian blur kernel
    int blur_height_;       ///< Height of the Gaussian blur kernel
    double blur_sigma_;     ///< Sigma for Gaussian blur
    int kernel_width_;      ///< Width of the dilation kernel
    int kernel_height_;     ///< Height of the dilation kernel
    int kernel_shape_;      ///< Shape of the dilation kernel

    bool zero_edges= false; // Stop Turtlebot3 if there's any edge. Which means there's no mid_point
    // Edge detection counters
    EdgeCountResult edge_count_result_; ///< Edge detection results

    // ROS 2 Interfaces
    rclcpp::Publisher<CmdVel>::SharedPtr cmd_vel_pub_;     ///< Publisher for velocity commands
    rclcpp::Subscription<imgMsg>::SharedPtr subscription_; ///< Subscriber for camera images
    CmdVel vel_msg;                                        ///< Velocity message

    /**
     * @brief Declares the configurable parameters for the Node.
     */
    void declare_parameters();

    /**
     * @brief Initializes the publishers for the Node.
     */
    void init_publishers();

    /**
     * @brief Initializes the subscribers for the Node.
     */
    void init_subscribers();

    /**
     * @brief Updates the Node's parameters with the latest values.
     */
    void update_parameters();

    /**
     * @brief Callback function for processing camera images.
     * @param camera_msg Shared pointer to the received camera image message.
     */
    void cameraCallback(const imgMsg::SharedPtr camera_msg);

    /**
     * @brief Counts edges in the specified region of interest (ROI).
     * @param roi The region of interest to process.
     */
    void count_edges(const cv::Mat& roi);

    /**
     * @brief Calculates the midpoint of the detected line and the robot's center.
     * @param edges List of edge positions detected.
     * @param img_width Width of the image being processed.
     * @return Pair of integers representing the robot's center and line's midpoint.
     */
    std::pair<int, int> calculate_mid_points(const std::vector<int>& edges, int img_width);

    /**
     * @brief Implements the control algorithm for following the detected line.
     * @param robot_mid_point The robot's center position in the image.
     * @param mid_point The midpoint of the detected line in the image.
     */
    void control_algorithm(int robot_mid_point, int mid_point);

    /**
     * @brief Implements the control algorithm for following the detected line.
     * @param roi The region of interest to process.
     * @param robot_mid_point The robot's center position in the image.
     * @param mid_point The midpoint of the detected line in the image.
     * @param img_column column size of image
     * @param img_row Row size of image
     */
    bool draw_points(const cv::Mat& roi, int robot_mid_point, int mid_point, int img_column, int img_row);
};

#endif  // LINE_FOLLOWING_HPP_
