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
#include <vector>

#include "gps_utils.hpp"

#include <tf2/LinearMath/Quaternion.h>


using namespace GPSUtils;
using fixMsg = sensor_msgs::msg::NavSatFix;

class MarkerPublisher : public rclcpp::Node {
public:
    MarkerPublisher()
    : Node("marker_publisher"),
      callback_group_(this->create_callback_group(rclcpp::CallbackGroupType::Reentrant)) {
        // Crear publicador para los marcadores
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "visualization_marker_array", 10);

        // Create callback groups
        gpsCallback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        imuCallback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        fusionCallback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        markerCallback_group_  = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        // GPS Subscription
        rclcpp::SubscriptionOptions gps_options;
        gps_options.callback_group = gpsCallback_group_;
        gps_sub_ = this->create_subscription<fixMsg>("gps/nav", 10, std::bind(&MarkerPublisher::gpsCallback, this, std::placeholders::_1), gps_options);

        // IMU Subscription
        rclcpp::SubscriptionOptions imu_options;
        imu_options.callback_group = imuCallback_group_;
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("imu", 10, std::bind(&MarkerPublisher::imuCallback, this, std::placeholders::_1), imu_options);

        // Fusion Subscription
        rclcpp::SubscriptionOptions fusion_options;
        fusion_options.callback_group = fusionCallback_group_;
        fusion_sub_ = this->create_subscription<geometry_msgs::msg::Point>("ekf_pose", 10, std::bind(&MarkerPublisher::ekfCallback, this, std::placeholders::_1), fusion_options);
        // Inicializar metas
        waypoints_ = {
            {0, 0, 0}, {0, 0, 0}
        };

        // Crear temporizador para publicar marcadores, con grupo de callbacks
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(40),
            std::bind(&MarkerPublisher::publishMarkers, this),
            markerCallback_group_);
    }

private:

    void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg){
        GPSData gps_data = {msg->latitude, msg->longitude, msg->altitude};
        ECEF ecef_data = gpsToECEF(gps_data);
        ENU enu_data = ecefToENU(ecef_data, ref_ecef_, ref_gps_);
        gps_msg.x =  -1 * enu_data.east;
        gps_msg.y = -1 * enu_data.north;
        RCLCPP_INFO(this->get_logger(), "GPS data received.");
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        imu_msg = msg;
        RCLCPP_INFO(this->get_logger(), "IMU data received.");
    }

    void ekfCallback(const geometry_msgs::msg::Point::SharedPtr msg){
        ekf_msg = msg;
        RCLCPP_INFO(this->get_logger(), "EKF data received.");
    }

    void update_waypoints(){
        RCLCPP_INFO(this->get_logger(), "Update");
        if (imu_msg && ekf_msg) {
            waypoints_ = {
                {gps_msg.x, gps_msg.y, imu_msg->angular_velocity.z}, 
                {ekf_msg->x, ekf_msg->y, ekf_msg->z}
            };
        }else{
            RCLCPP_ERROR(this->get_logger(), "IMU or EKF message is null. Waypoints not updated.");
        }
        RCLCPP_INFO(this->get_logger(), "After update");
    }

    void publishMarkers() {

        auto marker_array = visualization_msgs::msg::MarkerArray();
        // Publicar metas como esferas
        int id = 0;
        update_waypoints();
        for (const auto& pose : waypoints_) {
            auto marker = visualization_msgs::msg::Marker();
            tf2::Quaternion q;
            q.setRPY(0, 0, pose.z);
            marker.header.frame_id = "odom";
            marker.header.stamp = this->now();
            marker.ns = "goals";
            marker.id = id++;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = pose.x;
            marker.pose.position.y = pose.y;
            marker.pose.position.z = 0.0;
            marker.pose.orientation.x = q.x();
            marker.pose.orientation.y = q.y();
            marker.pose.orientation.z = q.z();
            marker.pose.orientation.w = q.w();
            marker.scale.x = 0.2;
            marker.scale.y = 0.2;
            marker.scale.z = 0.2;
            marker.color.a = 1.0;
            // Cambiar color según el índice
            if (id % 2 == 0) {  // Primer conjunto
                marker.color.r = 0.0;
                marker.color.g = 1.0;  // Verde
                marker.color.b = 0.0;
            } else {  // Segundo conjunto
                marker.color.r = 1.0;  // Rojo
                marker.color.g = 0.0;
                marker.color.b = 0.0;
            }
            marker_array.markers.push_back(marker);
        }

        // Publicar el array de marcadores
        marker_pub_->publish(marker_array);

        RCLCPP_INFO(this->get_logger(), "Published %zu goals and %zu path points",
                    waypoints_.size(), path_points_.size());
    }

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::CallbackGroup::SharedPtr gpsCallback_group_;
    rclcpp::CallbackGroup::SharedPtr imuCallback_group_;
    rclcpp::CallbackGroup::SharedPtr fusionCallback_group_;
    rclcpp::CallbackGroup::SharedPtr markerCallback_group_;

    std::vector<ECEF> waypoints_;
    std::vector<geometry_msgs::msg::Point> path_points_;


    GPSData ref_gps_;
    ECEF ref_ecef_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<fixMsg>::SharedPtr gps_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr fusion_sub_;
    sensor_msgs::msg::Imu::SharedPtr imu_msg;
    geometry_msgs::msg::Point gps_msg;
    geometry_msgs::msg::Point::SharedPtr ekf_msg;


};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    // Crear nodo
    auto node = std::make_shared<MarkerPublisher>();

    // Crear ejecutor multihilo
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    // Ejecutar
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
