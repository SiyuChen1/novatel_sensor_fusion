//
// Created by siyuchen on 14.08.23.
//

#ifndef BUILD_LLA2ENU_H
#define BUILD_LLA2ENU_H

#include <cmath>

const double radius = 6378137;
const double f = 1.0/298.257223563;

void lla2ned(const double & lat, const double & lon,
             const double & alt, const double & lat_ref,
             const double & lon_ref, const double & alt_ref,
             double & north, double & east, double & down,
             const bool& degrees=true
);

void lla2enu(const double & lat, const double & lon,
             const double & alt, const double & lat_ref,
             const double & lon_ref, const double & alt_ref,
             double & north, double & east, double & up,
             const bool& degrees=true
);
#endif //BUILD_LLA2ENU_H
