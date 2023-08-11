#!/bin/bash

PACKAGE_SRC_PATH=~/catkin_ws/log/latest_build/novatel_sensor_fusion
PACKAGE_INSTALL_PATH=~/catkin_ws/install/novatel_sensor_fusion
PACKAGE_BUILD_PATH=~/catkin_ws/build/novatel_sensor_fusion

CMAKE_PREFIX_PATH=/opt/ros/iron /usr/bin/cmake $PACKAGE_SRC_PATH -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_INSTALL_PREFIX=$PACKAGE_INSTALL_PATH
CMAKE_PREFIX_PATH=/opt/ros/iron /usr/bin/cmake --build $PACKAGE_BUILD_PATH -- -j24 -l24
CMAKE_PREFIX_PATH=/opt/ros/iron /usr/bin/cmake --install $PACKAGE_BUILD_PATH