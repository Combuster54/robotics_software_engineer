#include <rclcpp/rclcpp.hpp>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <vector>

class MarkerPublisher : public rclcpp::Node {
public:
    MarkerPublisher()
    : Node("marker_publisher"),
      callback_group_(this->create_callback_group(rclcpp::CallbackGroupType::Reentrant)) {
        // Crear publicador para los marcadores
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "visualization_marker_array", 10);

        // Crear suscriptor para odometría con grupo de callbacks
        auto odom_sub_options = rclcpp::SubscriptionOptions();
        odom_sub_options.callback_group = callback_group_;
        odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&MarkerPublisher::odometryCallback, this, std::placeholders::_1),
            odom_sub_options);

        // Inicializar metas
        waypoints_ = {
            {0, 0}, {1, 1}, {2, 2}, {3, 3},
            {4, 4}, {5, 5}, {6, 6}, {7, 7}
        };

        // Crear temporizador para publicar marcadores, con grupo de callbacks
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&MarkerPublisher::publishMarkers, this),
            callback_group_);
    }

private:
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // Procesar datos de odometría
        geometry_msgs::msg::Point current_point;
        current_point.x = msg->pose.pose.position.x;
        current_point.y = msg->pose.pose.position.y;
        current_point.z = 0.0;

        // Agregar punto al camino si es nuevo
        if (path_points_.empty() ||
            (path_points_.back().x != current_point.x || path_points_.back().y != current_point.y)) {
            path_points_.push_back(current_point);
            RCLCPP_INFO(this->get_logger(), "Added point to path: x=%f, y=%f",
                        current_point.x, current_point.y);
        }
    }

    void publishMarkers() {
        auto marker_array = visualization_msgs::msg::MarkerArray();

        // Publicar metas como esferas
        int id = 0;
        for (const auto& point : waypoints_) {
            auto marker = visualization_msgs::msg::Marker();
            marker.header.frame_id = "odom";
            marker.header.stamp = this->now();
            marker.ns = "goals";
            marker.id = id++;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = point.first;
            marker.pose.position.y = point.second;
            marker.pose.position.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.2;
            marker.scale.y = 0.2;
            marker.scale.z = 0.2;
            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.lifetime = rclcpp::Duration(0, 0); // Persistente

            marker_array.markers.push_back(marker);
        }

        // Publicar camino como línea continua
        auto path_marker = visualization_msgs::msg::Marker();
        path_marker.header.frame_id = "odom";
        path_marker.header.stamp = this->now();
        path_marker.ns = "path";
        path_marker.id = id;
        path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        path_marker.action = visualization_msgs::msg::Marker::ADD;
        path_marker.points = path_points_;
        path_marker.scale.x = 0.05;  // Grosor de la línea
        path_marker.color.a = 1.0;
        path_marker.color.r = 1.0;
        path_marker.color.g = 0.0;
        path_marker.color.b = 0.0;
        path_marker.lifetime = rclcpp::Duration(0, 0); // Persistente

        marker_array.markers.push_back(path_marker);

        // Publicar el array de marcadores
        marker_pub_->publish(marker_array);

        RCLCPP_INFO(this->get_logger(), "Published %zu goals and %zu path points",
                    waypoints_.size(), path_points_.size());
    }

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    std::vector<std::pair<double, double>> waypoints_;
    std::vector<geometry_msgs::msg::Point> path_points_;
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
