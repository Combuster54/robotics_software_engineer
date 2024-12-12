#include <memory>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/callback_group.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>

// Own libraries
#include "ekf_lib.hpp"
#include "gps_utils.hpp"

typedef Eigen::Matrix<double, 2, 1> Vector2d;
typedef Eigen::Matrix<double, 4, 1> Vector4d;
typedef Eigen::Matrix<double, 4, 4> Matrix4d;
typedef Eigen::Matrix<double, 4, 5> Matrix45d;
typedef Eigen::Matrix<double, 5, 1> Vector5d;
typedef Eigen::Matrix<double, 5, 5> Matrix5d;
typedef Eigen::Matrix<double, 2, 5> Matrix25d;

using fixMsg = sensor_msgs::msg::NavSatFix;
using namespace GPSUtils;

class ExtendedKalmanFilter_Node : public rclcpp::Node {

public:

    ExtendedKalmanFilter_Node() : Node("ekf_node") {
        setMatrices();

        // Create callback groups
        gps_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        imu_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        // GPS Subscription
        rclcpp::SubscriptionOptions gps_options;
        gps_options.callback_group = gps_callback_group_;
        gps_sub_ = this->create_subscription<fixMsg>("gps/nav", 10, std::bind(&ExtendedKalmanFilter_Node::gps_callback, this, std::placeholders::_1), gps_options);

        // Define reference GPS point (latitude, longitude, altitude)
        ref_gps_ = {0.0, 0.0, 0.0};  // Change based on your reference
        ref_ecef_ = gpsToECEF(ref_gps_);

        // IMU Subscription
        rclcpp::SubscriptionOptions imu_options;
        imu_options.callback_group = imu_callback_group_;
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("imu", 10, std::bind(&ExtendedKalmanFilter_Node::imuCallback, this, std::placeholders::_1), imu_options);

        fusion_pub_ = this->create_publisher<geometry_msgs::msg::Point>("ekf_pose", 10);
        RCLCPP_INFO(this->get_logger(), "Node initialized and ready.");
    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        imu_msg = msg;
        RCLCPP_INFO(this->get_logger(), "IMU data received.");
    }

    void estimation() {
        RCLCPP_INFO(this->get_logger(), "--------> Starting EKF Iterations ");

        ekf.predict();
        RCLCPP_INFO(this->get_logger(), "Fusion state published: x = %f, y = %f", fusion_msg.x, fusion_msg.y);

        RCLCPP_INFO(this->get_logger(), "Prediction step completed. State and covariance matrices have been predicted.");

        measurements << gps_msg.x, gps_msg.y,
                       imu_msg->linear_acceleration.x, imu_msg->angular_velocity.z;
                       
        RCLCPP_INFO(this->get_logger(), "Measurements obtained from sensors: x = %f, y = %f, ax = %f, az = %f",
        measurements[0], measurements[1], measurements[2], measurements[3]);

        std::vector<double> cov_values = {cov_gps, cov_gps, cov_imu, cov_imu};
        ekf.updateR(cov_values);
        ekf.update(measurements);
        RCLCPP_INFO(this->get_logger(), "Update step completed. State and covariance matrices have been corrected.");
    }

    void setMatrices() {
        P_in << 1, 0, 0, 0, 0,
                0, 1, 0, 0, 0,
                0, 0, 1, 0, 0,
                0, 0, 0, 1, 0,
                0, 0, 0, 0, 1;
        // State Transition Matrix
        F_in << 1, 0, 0, 0, 0,
                0, 1, 0, 0, 0,
                0, 0, 1, 0, 0,
                0, 0, 0, 1, 0,
                0, 0, 0, 0, 1;
        // Measurement Matrix
        H_in.setZero();
        // Measurement Noise Covariance Matrix
        R_in.setZero();
        // Process Noise Covariance Matrix
        Q_in << 0.1, 0, 0, 0, 0,
                0, 0.1, 0, 0, 0,
                0, 0, 0.1, 0, 0,
                0, 0, 0, 0.1, 0,
                0, 0, 0, 0, 0.1;

        ekf.dt = 0.1;

        ekf.x_pred_ = x_in;
        ekf.z_pred_ = Vector4d::Zero();

        ekf.initialize(x_in, P_in, F_in, H_in, R_in, Q_in);
    }

    void gps_callback(const fixMsg::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "GPS data received.");

        GPSUtils::GPSData gps_data = {msg->latitude, msg->longitude, msg->altitude};
        GPSUtils::ECEF ecef_data = GPSUtils::gpsToECEF(gps_data);
        GPSUtils::ENU enu_data = GPSUtils::ecefToENU(ecef_data, ref_ecef_, ref_gps_);

        gps_msg.x = -1 * enu_data.east;
        gps_msg.y = -1 * enu_data.north;

        estimation();
        RCLCPP_INFO(this->get_logger(), "ENU -> East: %.2f, North: %.2f, Up: %.2f", gps_msg.x, gps_msg.y, enu_data.up);
        fusion_msg.x = ekf.x_pred_[0];
        fusion_msg.y = ekf.x_pred_[1];
        fusion_msg.z = ekf.x_pred_[4];

        fusion_pub_->publish(fusion_msg);
    }

    Vector5d x_in;
    Matrix5d P_in;
    Matrix5d F_in;
    Matrix45d H_in;
    Matrix4d R_in;
    Matrix5d Q_in;

    Vector4d measurements;

    double cov_imu = 0.012727922061358;
    double cov_gps = 0.028452340807104;

    ExtendedKalmanFilter ekf;

    GPSUtils::GPSData ref_gps_;
    GPSUtils::ECEF ref_ecef_;

    sensor_msgs::msg::Imu::SharedPtr imu_msg;
    geometry_msgs::msg::Point gps_msg;
    geometry_msgs::msg::Point fusion_msg;

    rclcpp::CallbackGroup::SharedPtr gps_callback_group_;
    rclcpp::CallbackGroup::SharedPtr imu_callback_group_;

    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr fusion_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<fixMsg>::SharedPtr gps_sub_;

};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto ekf_node = std::make_shared<ExtendedKalmanFilter_Node>();

    // Multi-threaded executor
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(ekf_node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
