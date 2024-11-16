#ifndef SENSORS_HPP
#define SENSORS_HPP

#include "sensor_library_design.hpp"
#include <iostream>

namespace SensorLibrary {

    class TemperatureSensor {
    private:
        SensorData<double> temperature;

    public:
        TemperatureSensor(double temp);

        void readTemperature() const;
        void setTemperature(double temp);
    };

    class DistanceSensor {
    private:
        SensorData<int> distance;

    public:
        DistanceSensor(int dist);

        void readDistance() const;
        void setDistance(int dist);
    };

} // namespace SensorLibrary

#endif // SENSORS_HPP
