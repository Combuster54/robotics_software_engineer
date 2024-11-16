#ifndef SENSOR_DATA_HPP
#define SENSOR_DATA_HPP

#include <iostream>
#include <string>

namespace SensorLibrary {

    template<typename T>
    class SensorData {
    private:
        T data;

    public:
        SensorData(const T& value) : data(value) {}

        void setData(const T& value) {
            data = value;
        }

        T getData() const {
            return data;
        }

        void displayData() const {
            std::cout << "Sensor Data: " << data << std::endl;
        }
    };

} // namespace SensorLibrary

#endif // SENSOR_DATA_HPP
