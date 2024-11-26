#include <chrono>
#include <memory>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <Eigen/Dense>

using namespace std::chrono_literals;
using std::placeholders::_1;

class IMUProcessor : public rclcpp::Node
{
public:
    IMUProcessor()
        : Node("imu_processor"), prev_time_sec_(0), prev_time_nsec_(0)
    {
        // Suscripción al tópico de la IMU
        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10, std::bind(&IMUProcessor::imuCallback, this, _1));

        // Inicializamos la velocidad y aceleración en cero
        velocity_ = Eigen::Vector3d(0.0, 0.0, 0.0);
        prev_acceleration_ = Eigen::Vector3d(0.0, 0.0, 0.0);
    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Obtenemos el tiempo actual del mensaje
        double current_time_sec = msg->header.stamp.sec + msg->header.stamp.nanosec / 1e9;

        if (prev_time_sec_ != 0) // Saltamos el primer mensaje
        {
            double delta_t = current_time_sec - prev_time_sec_;

            // Transformar la aceleración lineal del marco local al global
            Eigen::Quaterniond orientation(msg->orientation.w, msg->orientation.x,
                                            msg->orientation.y, msg->orientation.z);
            Eigen::Vector3d acc_local(msg->linear_acceleration.x,
                                      msg->linear_acceleration.y,
                                      msg->linear_acceleration.z);
            Eigen::Vector3d acc_global = orientation * acc_local;

            // Resta la componente de gravedad
            Eigen::Vector3d gravity(0.0, 0.0, 9.81);
            Eigen::Vector3d acc_dynamic = acc_global - gravity;

            // Integramos para calcular la velocidad
            velocity_ += acc_dynamic * delta_t;

            // Calculamos la aceleración como el cambio de velocidad entre iteraciones
            Eigen::Vector3d acceleration = (velocity_ - prev_velocity_) / delta_t;

            // Imprimir los resultados
            RCLCPP_INFO(this->get_logger(),
                        "Delta Time: %.3f, Velocity: [%.3f, %.3f, %.3f], Acceleration: [%.3f, %.3f, %.3f]",
                        delta_t,
                        velocity_.x(), velocity_.y(), velocity_.z(),
                        acceleration.x(), acceleration.y(), acceleration.z());

            // Actualizamos la velocidad previa
            prev_velocity_ = velocity_;
            prev_acceleration_ = acc_dynamic;
        }

        // Actualizamos el tiempo previo
        prev_time_sec_ = current_time_sec;
        prev_time_nsec_ = msg->header.stamp.nanosec;
    }

    // Variables
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    Eigen::Vector3d velocity_;
    Eigen::Vector3d prev_velocity_;
    Eigen::Vector3d prev_acceleration_;
    double prev_time_sec_;
    double prev_time_nsec_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IMUProcessor>());
    rclcpp::shutdown();
    return 0;
}
