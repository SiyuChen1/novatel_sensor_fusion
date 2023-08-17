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
find_package(message_filters REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# custom defined messages type
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/QuaternionStampedExtended.msg"
  "msg/NavSatStatusExtended.msg"
  "msg/NavRMC.msg"
  "msg/NavSat.msg"
  "msg/NavSatExtended.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
  ADD_LINTER_TESTS
 )

include_directories(include)
add_library(lla2enu include/novatel_sensor_fusion/lla2enu.cpp)

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

add_executable(gnss_messages_sync src/gnss_messages_sync.cpp)

# start: illustration of using msg defined in this project
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(gnss_messages_sync ${cpp_typesupport_target})
# end

target_include_directories(gnss_messages_sync PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

# link custom libraries
target_link_libraries(gnss_messages_sync lla2enu)

ament_target_dependencies(gnss_messages_sync rclcpp std_msgs geometry_msgs message_filters)

add_executable(demo_message_filter_header_pub example/demo_message_filter_header_pub.cpp)
ament_target_dependencies(demo_message_filter_header_pub rclcpp std_msgs)

add_executable(demo_message_filter_header example/demo_message_filter_header.cpp)
ament_target_dependencies(demo_message_filter_header rclcpp message_filters std_msgs)

add_executable(demo_message_filter_temperature example/demo_message_filter_temperature.cpp)
ament_target_dependencies(demo_message_filter_temperature rclcpp sensor_msgs message_filters)

add_executable(demo_message_filter_imu example/demo_message_filter_imu.cpp)
ament_target_dependencies(demo_message_filter_imu rclcpp sensor_msgs message_filters)

install(TARGETS
        demo_message_filter_header
        demo_message_filter_header_pub
        demo_message_filter_temperature
        demo_message_filter_imu
        gnss_messages_sync
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME})

install(
        DIRECTORY "include/"
        DESTINATION include/${PROJECT_NAME})

install(TARGETS
        lla2enu
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

if(BUILD_TESTING)
#  find_package(ament_lint_auto REQUIRED)
#  # the following line skips the linter which checks for copyrights
#  # comment the line when a copyright and license is added to all source files
#  set(ament_cmake_copyright_FOUND TRUE)
#  # the following line skips cpplint (only works in a git repo)
#  # comment the line when this package is in a git repo and when
#  # a copyright and license is added to all source files
#  set(ament_cmake_cpplint_FOUND TRUE)
#  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest)

  ament_add_gtest(${PROJECT_NAME}_lla2enu_test test/test_lla2enu.cpp)

  target_include_directories(${PROJECT_NAME}_lla2enu_test PUBLIC
          $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
          $<INSTALL_INTERFACE:include>
  )

  # link custom libraries
  target_link_libraries(${PROJECT_NAME}_lla2enu_test lla2enu)

endif()

ament_package()
