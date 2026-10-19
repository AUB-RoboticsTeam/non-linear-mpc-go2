cmake_minimum_required(VERSION 3.8)
project(legged_estimation)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ocs2_legged_robot REQUIRED)
find_package(ocs2_centroidal_model REQUIRED)
find_package(ocs2_pinocchio_interface REQUIRED)
find_package(ocs2_robotic_tools REQUIRED)
find_package(legged_common REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pinocchio REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Boost REQUIRED COMPONENTS system filesystem log_setup log)

add_library(${PROJECT_NAME}
        src/StateEstimateBase.cpp
        src/FromTopicEstimate.cpp
        src/LinearKalmanFilter.cpp
        )

target_include_directories(${PROJECT_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${EIGEN3_INCLUDE_DIRS}
        ${pinocchio_INCLUDE_DIRS}
        )

target_link_libraries(${PROJECT_NAME}
        PUBLIC
        pinocchio::pinocchio
        )

ament_target_dependencies(${PROJECT_NAME}
        PUBLIC
        rclcpp
        geometry_msgs
        nav_msgs
        ocs2_legged_robot
        ocs2_centroidal_model
        ocs2_pinocchio_interface
        ocs2_robotic_tools
        legged_common
        realtime_tools
        tf2
        tf2_ros
        tf2_geometry_msgs
        )

target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS} ${OCS2_PINOCCHIO_FLAGS})

install(TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME}
        FILES_MATCHING PATTERN "*.h"
        )

install(EXPORT export_${PROJECT_NAME}
        DESTINATION share/${PROJECT_NAME}/cmake
        )

ament_export_include_directories(
        include
        ${pinocchio_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        )
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
        rclcpp
        geometry_msgs
        nav_msgs
        ocs2_legged_robot
        ocs2_centroidal_model
        ocs2_pinocchio_interface
        ocs2_robotic_tools
        legged_common
        realtime_tools
        tf2
        tf2_ros
        tf2_geometry_msgs
        pinocchio
        Eigen3
        Boost
        )

ament_package()
