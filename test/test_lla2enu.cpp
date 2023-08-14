//
// Created by siyuchen on 14.08.23.
//

#include <gtest/gtest.h>

#include "novatel_sensor_fusion/lla2enu.h"

TEST(LLA2ENU, lla2ned){
    double lat_ref = 45.9132;
    double lon_ref = 36.7484;
    double alt_ref = 1877.7532;

    double lat = 45.9142;
    double lon = 36.7494;
    double alt = 1687.7532;

    double east_real = 77.5843,
            north_real = 111.1496,
            down_real = 190.0000;
    double east, north, down;
    lla2ned(lat, lon, alt, lat_ref, lon_ref, alt_ref,
            north, east, down, true);
    EXPECT_NEAR(east, east_real, 0.001);
    EXPECT_NEAR(north, north_real, 0.001);
    EXPECT_NEAR(down, down_real, 0.001);
}

TEST(LLA2ENU, lla2enu){
    double lat_ref = 45.9132;
    double lon_ref = 36.7484;
    double alt_ref = 1877.7532;

    double lat = 45.9142;
    double lon = 36.7494;
    double alt = 1687.7532;

    double east_real = 77.5843,
        north_real = 111.1496,
        up_real = -190.0000;
    double east, north, up;
    lla2enu(lat, lon, alt, lat_ref, lon_ref, alt_ref,
            east,north, up, true);
    EXPECT_NEAR(east, east_real, 0.001);
    EXPECT_NEAR(north, north_real, 0.001);
    EXPECT_NEAR(up, up_real, 0.001);
}


int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}