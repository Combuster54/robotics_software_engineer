#include "line_following.hpp"

/**
 * @brief Constructor for the LineFollowing Node.
 */
LineFollowing::LineFollowing() : Node("LineFollowing") {
    declare_parameters();
    update_parameters();
    init_publishers();
    init_subscribers();
}

void LineFollowing::declare_parameters() {

    // Image Processing Parameters
    this->declare_parameter<int>("lower_threshold", 100);
    this->declare_parameter<int>("upper_threshold", 200);
    this->declare_parameter<int>("line_extraction", 180);
    this->declare_parameter<int>("blur_width", 3);
    this->declare_parameter<int>("blur_height", 3);
    this->declare_parameter<double>("blur_sigma", 1.4);
    this->declare_parameter<int>("kernel_width", 1);
    this->declare_parameter<int>("kernel_height", 1);
    this->declare_parameter<int>("kernel_shape", 1);

    // Control Parameters
    this->declare_parameter<double>("Kp", 0.001);
    this->declare_parameter<double>("Ki", 0.0001);

}

void LineFollowing::init_publishers() {
    cmd_vel_pub_ = this->create_publisher<CmdVel>("/cmd_vel", 10);
}

void LineFollowing::init_subscribers() {
    subscription_ = this->create_subscription<imgMsg>(
        "/camera/image_raw", 10, std::bind(&LineFollowing::cameraCallback, this, std::placeholders::_1));
}

void LineFollowing::update_parameters() {
    lower_threshold_ = this->get_parameter("lower_threshold").as_int();
    upper_threshold_ = this->get_parameter("upper_threshold").as_int();
    line_extraction_ = this->get_parameter("line_extraction").as_int();
    blur_width_ = this->get_parameter("blur_width").as_int();
    blur_height_ = this->get_parameter("blur_height").as_int();
    blur_sigma_ = this->get_parameter("blur_sigma").as_double();
    kernel_width_ = this->get_parameter("kernel_width").as_int();
    kernel_height_ = this->get_parameter("kernel_height").as_int();
    kernel_shape_ = this->get_parameter("kernel_shape").as_int();

}

void LineFollowing::cameraCallback(const imgMsg::SharedPtr camera_msg) {
    update_parameters();
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(camera_msg, "bgr8");

    // Convert image to grayscale and apply ROI
    cv::Mat gray_image, canny_image;
    cv::cvtColor(cv_ptr->image, gray_image, cv::COLOR_BGR2GRAY);

    // ROI -> Region Of Interest
    int row = 700, column = 300;
    cv::Mat roi = gray_image(cv::Range(row, row + 380), cv::Range(column, column + 700));

    if (blur_width_ <= 0 || blur_width_ % 2 == 0 || blur_height_ <= 0 || blur_height_ % 2 == 0) {
        RCLCPP_ERROR(this->get_logger(),
            "Invalid Gaussian kernel size: width=%d, height=%d. Both must be positive and odd.",
            blur_width_, blur_height_);
        return;
    }

    // Apply Gaussian blur and Canny edge detection
    cv::GaussianBlur(roi, roi, cv::Size(blur_width_, blur_height_), blur_sigma_);
    cv::Canny(roi, roi, lower_threshold_, upper_threshold_);

    // Apply dilation filter
    auto structure_shape = cv::MORPH_RECT; // Default value

    if (kernel_shape_ == 1) {
        structure_shape = cv::MORPH_RECT;
    } else if (kernel_shape_ == 2) {
        structure_shape = cv::MORPH_CROSS;
    } else if (kernel_shape_ == 3) {
        structure_shape = cv::MORPH_ELLIPSE;
    } else{
        structure_shape = cv::MORPH_RECT;
    }

    if (kernel_width_ < 1 || kernel_height_ < 1) {
        RCLCPP_ERROR(this->get_logger(), "Invalid kernel size.");
        return;
    }

    cv::Mat kernel = cv::getStructuringElement(structure_shape, cv::Size(kernel_width_, kernel_height_));
    cv::dilate(roi, roi, kernel, cv::Point(-1, -1), 1);
    cv::Canny(roi, roi,lower_threshold_, upper_threshold_);

    // Count edges
    count_edges(roi);

    RCLCPP_INFO(this->get_logger(), "Count of 1 edge: %d", edge_count_result_.count_1_edge);
    RCLCPP_INFO(this->get_logger(), "Count of 2 edges: %d", edge_count_result_.count_2_edge);
    RCLCPP_INFO(this->get_logger(), "Count of 3 edges: %d", edge_count_result_.count_3_edge);
    RCLCPP_INFO(this->get_logger(), "Count of 4 edges: %d", edge_count_result_.count_4_edge);

    if (roi.empty()) {
        RCLCPP_INFO(this->get_logger(),  "ROI image is empty after processing.");
        return;
    }else{
        RCLCPP_INFO(this->get_logger(),  "Calculating mid point...");
        auto [robot_mid_point, mid_point] = calculate_mid_points(edge_count_result_.edges, roi.cols);
        bool navigation = draw_points(roi, robot_mid_point,mid_point , roi.cols, roi.rows);
        if(navigation){
            control_algorithm(robot_mid_point, mid_point);
        }
    }

    if(zero_edges){
        vel_msg.linear.x  = 0.0;
        vel_msg.angular.z = 0.0;
    }
    cmd_vel_pub_->publish(vel_msg);

    edge_count_result_.edges.clear();
    cv::imshow("ROI", roi);
    cv::waitKey(1);
}

bool LineFollowing::draw_points(const cv::Mat& roi, int robot_mid_point, int mid_point, int img_column, int img_row){

    if (mid_point >= 0 && mid_point < img_column && robot_mid_point >= 0 && robot_mid_point < img_row) {
        RCLCPP_INFO(this->get_logger(), "Mid Point: (%d, %d)", mid_point, line_extraction_);
        RCLCPP_INFO(this->get_logger(), "Robot Mid Point: (%d, %d)", robot_mid_point, line_extraction_);
        cv::circle(roi, cv::Point(mid_point, line_extraction_),  2, 
        cv::Scalar(255,0,255), 2, cv::LINE_AA);
        cv::circle(roi, cv::Point(robot_mid_point, line_extraction_), 5
        , cv::Scalar(255,255,0), 5, cv::LINE_AA);
        return true;
    } else {
        RCLCPP_WARN(this->get_logger(), "Mid Point is out of bounds: (%d, %d)", mid_point, 250);
        return false;
    }

}
void LineFollowing::count_edges(const cv::Mat& roi) {
    std::vector<int> edges;
    for (int i = 0; i < roi.cols; ++i) {
        if (roi.at<uchar>(line_extraction_, i) == 255) {
            edges.push_back(i);
        }
    }
    
    if (edges.size() < 2) {
        RCLCPP_WARN(this->get_logger(), "Not enough edges detected. Skipping frame.");
        zero_edges = true;
        return;
    }

    switch (edges.size()) {
        case 1: edge_count_result_.count_1_edge++; break;
        case 2: edge_count_result_.count_2_edge++; break;
        case 3: edge_count_result_.count_3_edge++; break;
        case 4: edge_count_result_.count_4_edge++; break;
    }

    edge_count_result_.edges = edges; // Updating edges
}

std::pair<int, int> LineFollowing::calculate_mid_points(const std::vector<int>& edges, int img_width) {
    if (edges.size() < 2) return {0, 0};
    int mid_point = edges[1] + (edges[2] - edges[1]) / 2;
    int robot_mid_point = img_width / 2;
    return {robot_mid_point, mid_point};
}

void LineFollowing::control_algorithm(int robot_mid_point, int mid_point) {

    // Calculate the error in the X-axis (difference between robot's midpoint and target midpoint)
    double error_x = static_cast<double>(robot_mid_point - mid_point);

    // Retrieve PI controller parameters
    double Kp = (this->get_parameter("Kp").as_double()); // Proportional gain
    double Ki = (this->get_parameter("Ki").as_double()); // Integral gain

    // Integral term (accumulates errors over time)
    static double integral_error_x = 0.0; // Persistent variable to accumulate error between calls

    // Update the integral term
    integral_error_x += error_x;

    // Limit the integral term to prevent windup (anti-windup mechanism)
    double max_integral = 10.0; // Adjust this value based on your system's needs
    integral_error_x = std::clamp(integral_error_x, -max_integral, max_integral);

    // Compute the angular control signal using PI controller
    double angular_control = (Kp * error_x) + (Ki * integral_error_x);
    //                        0.01* error_x + 0.0001 * integral_error_x

    // Debug information (optional)
    RCLCPP_INFO(this->get_logger(), "error_x: %f , integral_error_x = %f",
                error_x,integral_error_x);
    //                        0.01* 77 + 0.0001 *  );


    // Optionally adjust the linear velocity based on the error
    double max_linear_speed = 0.22;  // Maximum forward speed
    double min_linear_speed = 0.1;   // Minimum forward speed
    vel_msg.linear.x = std::clamp(max_linear_speed - std::abs(error_x) * 0.01, 
                                  min_linear_speed, max_linear_speed);

    // angular_control =
    // Set the angular velocity based on the control signal
    vel_msg.angular.z = angular_control;

    // Publish the velocity command to the robot
    cmd_vel_pub_->publish(vel_msg);

    // Debug information (optional)
    RCLCPP_INFO(this->get_logger(), "Error X: %f, Angular Control: %f, Linear Velocity: %f",
                error_x, angular_control, vel_msg.linear.x);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LineFollowing>());
    rclcpp::shutdown();
    return 0;
}
