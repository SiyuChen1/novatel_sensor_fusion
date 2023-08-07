cmake_minimum_required(VERSION 3.8)
project(novatel_sensor_fusion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/QuaternionStampedExtended.msg"
  "msg/NavSatStatusExtended.msg"
  "msg/NavRMC.msg"
  "msg/NavSat.msg"
  "msg/NavSatExtended.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
  ADD_LINTER_TESTS
 )

# Install Python modules
ament_python_install_package(novatel_sensor_fusion_py)

# Install Python executables
install(PROGRAMS
  scripts/read_raw_data_to_bag.py
  scripts/tf_broadcaster_lla2neu.py
  scripts/sub_gnss_imu.py
  scripts/read_raw_data_to_bag_group_by_timestamp.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install C++ executables
add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
