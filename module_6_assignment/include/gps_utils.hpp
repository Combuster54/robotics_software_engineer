#ifndef GPS_UTILS_HPP
#define GPS_UTILS_HPP

#include <cmath>

namespace GPSUtils {

    struct GPSData {
        double latitude;
        double longitude;
        double altitude;
    };

    struct ECEF {
        double x;
        double y;
        double z;
    };

    struct ENU {
        double east;
        double north;
        double up;
    };

    // Conversión de grados a radianes
    inline double degreesToRadians(double degrees) {
        return degrees * M_PI / 180.0;
    }

    // Convertir GPS a ECEF
    ECEF gpsToECEF(const GPSData &gps);

    // Convertir ECEF a ENU
    ENU ecefToENU(const ECEF &ecef, const ECEF &ref_ecef, const GPSData &ref_gps);

}  // namespace GPSUtils

#endif  // GPS_UTILS_HPP
