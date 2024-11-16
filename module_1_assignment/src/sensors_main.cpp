#include "sensor_library_design.hpp"
#include "sensors.hpp"

int main() {
    using namespace SensorLibrary;

    // Crear sensores
    TemperatureSensor tempSensor(25.5);  // 25.5 °C
    DistanceSensor distSensor(150);      // 150 cm

    // Leer valores
    tempSensor.readTemperature();
    distSensor.readDistance();

    return 0;
}
