#include "gps_utils.hpp"

namespace GPSUtils {

    // Constantes geodésicas
    constexpr double a = 6378137.0;                // Radio ecuatorial de la Tierra (m)
    constexpr double f = 1 / 298.257223563;        // Achatamiento
    constexpr double e2 = f * (2 - f);             // Excentricidad cuadrada

    // Implementación de GPS a ECEF
    ECEF gpsToECEF(const GPSData &gps) {
        double lat_rad = degreesToRadians(gps.latitude);
        double lon_rad = degreesToRadians(gps.longitude);
        double N = a / std::sqrt(1 - e2 * std::sin(lat_rad) * std::sin(lat_rad));

        ECEF ecef;
        ecef.x = (N + gps.altitude) * std::cos(lat_rad) * std::cos(lon_rad);
        ecef.y = (N + gps.altitude) * std::cos(lat_rad) * std::sin(lon_rad);
        ecef.z = (N * (1 - e2) + gps.altitude) * std::sin(lat_rad);

        return ecef;
    }

    // Implementación de ECEF a ENU
    ENU ecefToENU(const ECEF &ecef, const ECEF &ref_ecef, const GPSData &ref_gps) {
        double lat0_rad = degreesToRadians(ref_gps.latitude);
        double lon0_rad = degreesToRadians(ref_gps.longitude);

        double dx = ecef.x - ref_ecef.x;
        double dy = ecef.y - ref_ecef.y;
        double dz = ecef.z - ref_ecef.z;

        ENU enu;
        enu.east = -std::sin(lon0_rad) * dx + std::cos(lon0_rad) * dy;
        enu.north = -std::sin(lat0_rad) * std::cos(lon0_rad) * dx -
                    std::sin(lat0_rad) * std::sin(lon0_rad) * dy +
                    std::cos(lat0_rad) * dz;
        enu.up = std::cos(lat0_rad) * std::cos(lon0_rad) * dx +
                 std::cos(lat0_rad) * std::sin(lon0_rad) * dy +
                 std::sin(lat0_rad) * dz;

        return enu;
    }

}  // namespace GPSUtils
