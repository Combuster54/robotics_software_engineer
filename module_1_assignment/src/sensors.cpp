#include "sensors.hpp"
#include <iostream>

namespace SensorLibrary {

    // Constructor de TemperatureSensor
    TemperatureSensor::TemperatureSensor(double temp) : temperature(temp) {}

    void TemperatureSensor::readTemperature() const {
        std::cout << "Temperature Sensor Reading: ";
        temperature.displayData();
    }

    void TemperatureSensor::setTemperature(double temp) {
        temperature.setData(temp);
    }

    // Constructor de DistanceSensor
    DistanceSensor::DistanceSensor(int dist) : distance(dist) {}

    void DistanceSensor::readDistance() const {
        std::cout << "Distance Sensor Reading: ";
        distance.displayData();
    }

    void DistanceSensor::setDistance(int dist) {
        distance.setData(dist);
    }

} // namespace SensorLibrary
