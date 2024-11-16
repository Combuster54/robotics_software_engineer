#include "robot_class.hpp"

namespace RobotNamespace {

RobotClass::RobotClass(const std::string& robot_name, float velocity, const PhysicalAttributes& robotSpecs): robot_name(robot_name), velocity(velocity), specs(robotSpecs) {}

void RobotClass::moveForward(float vel){

    if(vel <0){
        vel = -vel;
    }
    velocity = vel;
    std::cout << "The Robot " << robot_name << " is moving forward" << std::endl;
}

void RobotClass::moveBackward(float vel){
    if(vel > 0){
        vel = -vel;
    }
    velocity = vel;
    std::cout << "The Robot " << robot_name << " is moving backward" << std::endl;
}

void RobotClass::stop(){

    velocity = 0.0;
    std::cout << "The Robot " << robot_name << " has been stopped" << std::endl;

}

}