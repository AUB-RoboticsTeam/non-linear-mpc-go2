cmake_minimum_required(VERSION 3.8)
project(legged_unitree_sim)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz_ros2_control REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(legged_common REQUIRED)

add_library(${PROJECT_NAME} SHARED
        src/FootContactPublisher.cpp
        src/FootGroundSupport.cpp
        src/FootContactSensor.cpp
        src/GroundTruthOdometryPublisher.cpp
        src/InitialJointPositionSetter.cpp
        src/ImuStateSensor.cpp
        )

target_include_directories(${PROJECT_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        )

ament_target_dependencies(${PROJECT_NAME}
        PUBLIC
        rclcpp
        hardware_interface
        gz-transport13
        gz_ros2_control
        pluginlib
        sensor_msgs
        ros_gz_interfaces
        std_msgs
        legged_common
        )

pluginlib_export_plugin_description_file(gz_ros2_control legged_unitree_sim_plugins.xml)

install(DIRECTORY config launch
        DESTINATION share/${PROJECT_NAME}
        )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME}
        FILES_MATCHING PATTERN "*.h"
        )

install(TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        )

install(PROGRAMS scripts/publish_robot_description.py
        DESTINATION lib/${PROJECT_NAME}
        )

install(EXPORT export_${PROJECT_NAME}
        DESTINATION share/${PROJECT_NAME}/cmake
        )

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
        rclcpp
        hardware_interface
        gz-transport13
        gz_ros2_control
        pluginlib
        sensor_msgs
        ros_gz_interfaces
        std_msgs
        legged_common
        )

ament_package()
