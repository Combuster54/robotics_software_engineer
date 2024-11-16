#ifndef ROBOTCLASS_HPP
#define ROBOTCLASS_HPP

#include <iostream>
#include <vector>
#include <string>

namespace RobotNamespace {

struct PhysicalAttributes{
    float weight;
    float size;
    int num_sensors;
};

class RobotClass{

private: 
    std::string robot_name;
    float velocity;
    PhysicalAttributes specs;

public:
    RobotClass(const std::string& robot_name, float velocity, const PhysicalAttributes& robotSpecs);

    void moveForward(float vel);
    void moveBackward(float vel);
    void stop();
}; //  RobotClass

} // namespace RobotNamespace

#endif // ROBOTCLASS_HPP
