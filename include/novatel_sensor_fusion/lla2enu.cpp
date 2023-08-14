//
// Created by siyuchen on 14.08.23.
//

#include "lla2enu.h"

void lla2ned(const double& lat, const double& lon,
             const double& alt, const double& lat_ref,
             const double& lon_ref, const double& alt_ref,
             double & north, double & east, double& down,
             const bool& degrees){
    const double dlat = degrees ? (lat - lat_ref) / 180 * M_PI: lat - lat_ref;
    const double dlon = degrees ? (lon - lon_ref) / 180 * M_PI: lon - lon_ref;
    const double sin_lat_ref = degrees ? (sin(lat_ref / 180 * M_PI)) : sin(lat_ref);
    const double cos_lat_ref = degrees ? (cos(lat_ref / 180 * M_PI)) : cos(lat_ref);
    const double e2 = f * ( 2 - f);
    const double tmp = 1 - e2 * sin_lat_ref * sin_lat_ref;
    const double rn = radius / sqrt(tmp);
    const double rm = rn * (1 - e2) / tmp;
    north = dlat / atan2(1.0, rm);
    east = dlon / atan2(1, rn * cos_lat_ref);
    down = - alt + alt_ref;
}

void lla2enu(const double& lat, const double& lon,
             const double& alt, const double& lat_ref,
             const double& lon_ref, const double& alt_ref,
             double & east, double & north, double& up,
             const bool& degrees){
    double down;
    lla2ned(lat, lon, alt, lat_ref, lon_ref, alt_ref, north, east, down, degrees);
    up = - down;
}