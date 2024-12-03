#include "goal_planner.hpp"

GoalPlanner::GoalPlanner()
    : Node("goal_planner_node")
{
    // Declarar parámetros
    declare_parameters();

    // Inicializar publishers y subscribers
    init_publishers();
    init_subscribers();
}

void GoalPlanner::declare_parameters() {

    this->declare_parameter<double>("Kp_angle", 0.5);
    this->declare_parameter<double>("Kp_dist", 0.5);

    this->declare_parameter<double>("max_linear_speed", 0.1);  // Velocidad máxima lineal
    this->declare_parameter<double>("min_linear_speed", 0.1);  // Velocidad máxima lineal

    this->declare_parameter<double>("max_angular_speed", 0.25); // Velocidad máxima angular
    this->declare_parameter<double>("min_angular_speed", 0.25); // Velocidad máxima angular
}

void GoalPlanner::init_publishers() {
    // Crear publisher para publicar comandos de velocidad
    cmd_vel_pub_ = this->create_publisher<CmdVel>("/cmd_vel", 10);
}

void GoalPlanner::init_subscribers() {
    // Crear subscriber para recibir datos de odometría o posición actual
    subscription_ = this->create_subscription<scanMsg>(
        "/scan", 10, std::bind(&GoalPlanner::scanCallback, this, std::placeholders::_1));
    subscription_ = this->create_subscription<odomMsg>(
            "odom", 10, std::bind(&GoalPlaner::odom_callback, this, std::placeholders::_1));
}

void GoalPlanner::scanCallback(const scanMsg::SharedPtr scan_msg) {
    // Este es un callback placeholder. Puedes agregar lógica para procesar imágenes si es necesario.
    RCLCPP_INFO(this->get_logger(), "Received scan data, processing...");
}

void GoalPlanner::odom_callback(const odomMsg::SharedPtr odom_msg){

    
}
void GoalPlanner::planning_path(RobotPose robot_position, Goal goal) {
    // Planificar un camino simple hacia el objetivo.
    RCLCPP_INFO(this->get_logger(), "Planning path to goal: (%.2f, %.2f)", goal.x_goal, goal.y_goal);

    CmdVel vel_msg;

    // Calcular las diferencias en x e y
    double dx = goal.x_goal - robot_position.x;
    double dy = goal.y_goal - robot_position.y;

    // Calcular la distancia al objetivo
    double distance = std::sqrt(dx * dx + dy * dy);

    // Calcular el ángulo deseado
    double target_theta = std::atan2(dy, dx);

    // Control proporcional para ajustar el ángulo
    double angular_error = target_theta - robot_position.theta;
    vel_msg.angular.z = 0.5 * angular_error;

    // Avanzar hacia el objetivo si está alineado
    if (std::abs(angular_error) < 0.1) {  // Si el error angular es pequeño, avanza
        vel_msg.linear.x = std::min(distance, 0.2);  // Límite de velocidad lineal
    } else {
        vel_msg.linear.x = 0.0;  // Detenerse hasta corregir el ángulo
    }

    // Publicar los comandos de velocidad
    cmd_vel_pub_->publish(vel_msg);
}

Goal GoalPlanner::choose_closest_goal(RobotPose robot_position, std::vector<Goal> goals) {
    Goal closest_goal;
    double min_distance = std::numeric_limits<double>::max();

    for (const auto &goal : goals) {
        // Calcular distancia euclidiana
        double distance = std::sqrt(
            std::pow(goal.x_goal - robot_position.x, 2) +
            std::pow(goal.y_goal - robot_position.y, 2)
        );

        // Actualizar el objetivo más cercano
        if (distance < min_distance) {
            min_distance = distance;
            closest_goal = goal;
        }
    }

    RCLCPP_INFO(this->get_logger(), "Closest goal selected: (%.2f, %.2f)", closest_goal.x_goal, closest_goal.y_goal);
    return closest_goal;
}

void GoalPlanner::execute_plan(Goal closest_goal){


    
}