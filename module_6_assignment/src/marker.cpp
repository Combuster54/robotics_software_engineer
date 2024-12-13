#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <vector>

// Own libraries
#include "gps_utils.hpp"


using fixMsg = sensor_msgs::msg::NavSatFix;
using namespace GPSUtils;

class MarkerPublisher : public rclcpp::Node {
public:
    MarkerPublisher()
    : Node("marker_publisher") {
        // Crear publicador para los marcadores
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "visualization_marker_array", 10);

        // GPS Subscription
        gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "gps/nav", 10, std::bind(&MarkerPublisher::gpsCallback, this, std::placeholders::_1));

        // EKF Subscription
        ekf_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "ekf_pose", 10, std::bind(&MarkerPublisher::ekfCallback, this, std::placeholders::_1));


        ref_gps_ = {0.0, 0.0, 0.0};  // Change based on your reference
        ref_ecef_ = gpsToECEF(ref_gps_);
        
        // Crear temporizador para publicar marcadores
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(40), std::bind(&MarkerPublisher::publishMarkers, this));
    }

private:
    void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {

        GPSData gps_data = {msg->latitude, msg->longitude, msg->altitude};
        ECEF ecef_data = gpsToECEF(gps_data);
        ENU enu_data = ecefToENU(ecef_data, ref_ecef_, ref_gps_);

        geometry_msgs::msg::Point gps_point;

        gps_point.x = -1 * enu_data.east;
        gps_point.y = -1 * enu_data.north;
        gps_point.z = 0.0;            // Altitud ignorada para simplificar

        // if (gps_imu_path_.empty() ||
        //     std::hypot(gps_imu_path_.back().x - gps_imu_.x,
        //             gps_imu_path_.back().y - gps_imu_.y) > 0.05) { // Distancia mínima de 5 cm
        //     gps_imu_path_.push_back(gps_imu_);
        // }

        // Evitar agregar puntos duplicados
        if (gps_imu_path_.empty() ||
            gps_imu_path_.back().x != gps_point.x ||
            gps_imu_path_.back().y != gps_point.y) {
            gps_imu_path_.push_back(gps_point);
            RCLCPP_INFO(this->get_logger(), "Added GPS point: x=%f, y=%f", gps_point.x, gps_point.y);
        }
    }

    void ekfCallback(const geometry_msgs::msg::Point::SharedPtr msg) {

        geometry_msgs::msg::Point ekf_points;
        ekf_points.x = msg->x;
        ekf_points.y = msg->y;
        ekf_points.z = 0.0;

        if (ekf_path_.empty() || ekf_path_.back().x != ekf_points.x || 
           ekf_path_.back().y != ekf_points.y) 
        { 
            ekf_path_.push_back(ekf_points);
            RCLCPP_INFO(this->get_logger(), "Added GPS point: x=%f, y=%f", ekf_points.x, ekf_points.y);
        }

    }

    void publishMarkers() {
        auto marker_array = visualization_msgs::msg::MarkerArray();

        // Variables to publish lines
        paths_.clear();
        paths_.push_back(gps_imu_path_);
        paths_.push_back(ekf_path_);

        int id = 0;
        for (const auto& path : paths_) {
            auto path_marker = visualization_msgs::msg::Marker();
            path_marker.header.frame_id = "odom";
            path_marker.header.stamp = this->now();
            path_marker.ns = "path";
            path_marker.id = id++;
            path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            path_marker.action = visualization_msgs::msg::Marker::ADD;
            path_marker.points = path;
            path_marker.scale.x = 0.02;  // Grosor de la línea
            path_marker.color.a = 1.0;
            // Cambiar colores según ID
            if (id % 2 == 0) {
                path_marker.color.r = 0.0; // Verde
                path_marker.color.g = 1.0;
                path_marker.color.b = 0.0;
            } else {
                path_marker.color.r = 1.0; // Rojo
                path_marker.color.g = 0.0;
                path_marker.color.b = 0.0;
            }
            path_marker.lifetime = rclcpp::Duration(0, 0); // Persistente

            marker_array.markers.push_back(path_marker);
        }

        // // Crear marcador para el path GPS
        // auto path_marker = visualization_msgs::msg::Marker();
        // path_marker.header.frame_id = "odom";
        // path_marker.header.stamp = this->now();
        // path_marker.ns = "gps_path";
        // path_marker.id = 0;
        // path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        // path_marker.action = visualization_msgs::msg::Marker::ADD;
        // path_marker.points = gps_imu_path_;
        // path_marker.scale.x = 0.05;  // Grosor de la línea
        // path_marker.color.a = 1.0;
        // path_marker.color.r = 0.0;
        // path_marker.color.g = 1.0;  // Verde
        // path_marker.color.b = 0.0;
        // path_marker.lifetime = rclcpp::Duration(0, 0); // Persistente

        // marker_array.markers.push_back(path_marker);

        // // Crear marcador para el path EKF
        // path_marker.header.frame_id = "odom";
        // path_marker.header.stamp = this->now();
        // path_marker.ns = "gps_path";
        // path_marker.id = 1;
        // path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        // path_marker.action = visualization_msgs::msg::Marker::ADD;
        // path_marker.points = ekf_path_;
        // path_marker.scale.x = 0.05;  // Grosor de la línea
        // path_marker.color.a = 1.0;
        // path_marker.color.r = 1.0;
        // path_marker.color.g = 0.0;  // Verde
        // path_marker.color.b = 0.0;
        // path_marker.lifetime = rclcpp::Duration(0, 0); // Persistente

        // marker_array.markers.push_back(path_marker);

        // Publicar el array de marcadores
        marker_pub_->publish(marker_array);
        RCLCPP_INFO(this->get_logger(), "Published GPS path with %zu points", gps_imu_path_.size());
    }

    GPSUtils::GPSData ref_gps_;
    GPSUtils::ECEF ref_ecef_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr ekf_sub_;

    std::vector<geometry_msgs::msg::Point> gps_imu_path_;
    std::vector<geometry_msgs::msg::Point> ekf_path_;
    std::vector<std::vector<geometry_msgs::msg::Point>> paths_;

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    // Crear nodo
    auto node = std::make_shared<MarkerPublisher>();

    // Ejecutar
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
