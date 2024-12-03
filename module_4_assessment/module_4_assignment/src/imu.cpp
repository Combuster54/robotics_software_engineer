#include <chrono>
#include <memory>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/accel.hpp"
#include <Eigen/Dense>

using namespace std::chrono_literals;
using std::placeholders::_1;

using imuMsg = sensor_msgs::msg::Imu;
using odomMsg = nav_msgs::msg::Odometry;

class DataClass : public rclcpp::Node
{
public:
    DataClass()
        : Node("data_class_node"), prev_time_sec_(0), prev_time_nsec_(0), prev_time_odom_(rclcpp::Time(0, 0, RCL_ROS_TIME)),prev_odom_x_(0),prev_odom_y_(0)
    {
        // Suscripción al tópico de la IMU
        imu_sub_ = this->create_subscription<imuMsg>(
            "/imu", 10, std::bind(&DataClass::imuCallback, this, _1));

        odom_sub_ = this->create_subscription<odomMsg>(
            "/odom", 10, std::bind(&DataClass::odomCallback, this, _1));
        
        // Inicializamos la velocidad y aceleración en cero
        velocity_ = Eigen::Vector3d(0.0, 0.0, 0.0);
        prev_acceleration_ = Eigen::Vector3d(0.0, 0.0, 0.0);
    }

private:
    void odomCallback(const odomMsg::SharedPtr msg){
        float current_x_odom = msg->pose.pose.position.x;
        float current_y_odom = msg->pose.pose.position.y;
        rclcpp::Time current_time = msg->header.stamp;

        if (prev_time_nsec_ != 0) // Saltamos el primer mensaje
        {
            float dx = current_x_odom - prev_odom_x_;
            float dy = current_y_odom - prev_odom_y_;
            float dt = (current_time - prev_time_odom_).seconds();

            distance_ =
            sqrt(pow(dy, 2) +
                 pow(dx, 2));

            vel_ = distance_ / dt;
        }

        // Update variables
        prev_odom_x_ = current_x_odom;
        prev_odom_y_ = current_y_odom;
        prev_time_odom_ = current_time;
    }
    
    void imuCallback(const imuMsg::SharedPtr msg)
    {
        // Get current time
        double current_time_sec = msg->header.stamp.sec + msg->header.stamp.nanosec / 1e9;

        if (prev_time_sec_ != 0) // Ignore first message
        {
            double delta_t = current_time_sec - prev_time_sec_;

            Eigen::Vector3d current_acceleration(
                msg->linear_acceleration.x,
                msg->linear_acceleration.y,
                msg->linear_acceleration.z
            );

            // Instantaneous velocity
            Eigen::Vector3d linear_velocity_ = (current_acceleration - prev_acceleration_) / delta_t;

            // Print results
            RCLCPP_INFO(this->get_logger(),
                "\nLinear Acceleration:  [%.3f, %.3f, %.3f] \n"
                "------------------------------------------------------------------------------------\n"
                "Linear Velocity:      [%.3f, %.3f, %.3f] \n"
                "Angular Velocity:     [%.3f, %.3f, %.3f]",
                current_acceleration.x(), current_acceleration.y(), current_acceleration.z(),
                linear_velocity_.x(), linear_velocity_.y(), linear_velocity_.z(),
                msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z
            );

            // Update prev_acceleration
            prev_acceleration_ = current_acceleration;

        }


        // Update time
        prev_time_sec_ = current_time_sec;
        prev_time_nsec_ = msg->header.stamp.nanosec;
    }

    // Variables
    rclcpp::Subscription<imuMsg>::SharedPtr imu_sub_;
    rclcpp::Subscription<odomMsg>::SharedPtr odom_sub_;


    Eigen::Vector3d velocity_;
    Eigen::Vector3d prev_velocity_;
    Eigen::Vector3d prev_acceleration_;

    double prev_time_sec_;
    double prev_time_nsec_;

    // Acceleration variables
    double prev_linear_acc_x = 0.0;
    double prev_linear_acc_y = 0.0;
    double prev_linear_acc_z = 0.0;
    double prev_orientation_w = 1.0; // Identidad en un quaternion
    double prev_orientation_x = 0.0;
    double prev_orientation_y = 0.0;
    double prev_orientation_z = 0.0;


    geometry_msgs::msg::Accel prev_acc_;


    // Distance variables
    bool first_odom_received = false;
    rclcpp::Time prev_time_odom_;
    double prev_odom_x_;
    double prev_odom_y_;
    double distance_ = 0.0;
    double vel_ = 0.0;

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DataClass>());
    rclcpp::shutdown();
    return 0;
}
