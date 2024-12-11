#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/point.hpp>
#include "gps_utils.hpp"

using namespace GPSUtils;

class ExtendedKalmanFilter_Node : public rclcpp::Node
{
public:
    ExtendedKalmanFilter_Node()
    : Node("ekf_node")
    {
        gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "gps/nav", 10,
            std::bind(&ExtendedKalmanFilter_Node::gps_callback, this, std::placeholders::_1));

        enu_pub_ = this->create_publisher<geometry_msgs::msg::Point>("gps/enu", 10);

        // Define el punto de referencia (latitud, longitud, altitud)
        ref_gps_ = {0.0, 0.0, 0.0};  // Cambiar según tu referencia
        ref_ecef_ = gpsToECEF(ref_gps_);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr enu_pub_;

    GPSData ref_gps_;
    ECEF ref_ecef_;

    // Callback del GPS
    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        GPSData gps_data = {msg->latitude, msg->longitude, msg->altitude};
        ECEF ecef_data = gpsToECEF(gps_data);
        ENU enu_data = ecefToENU(ecef_data, ref_ecef_, ref_gps_);

        // Publicar ENU como geometry_msgs/Point
        auto enu_msg = geometry_msgs::msg::Point();
        enu_msg.x = -1 * enu_data.east;
        enu_msg.y = -1 * enu_data.north;
        enu_msg.z = enu_data.up;
        enu_pub_->publish(enu_msg);

        RCLCPP_INFO(this->get_logger(), "ENU -> East: %.2f, North: %.2f, Up: %.2f", enu_msg.x, enu_msg.y, enu_msg.z);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ExtendedKalmanFilter_Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
