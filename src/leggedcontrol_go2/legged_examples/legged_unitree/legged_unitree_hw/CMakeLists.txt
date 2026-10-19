cmake_minimum_required(VERSION 3.8)
project(legged_unitree_hw)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(legged_common REQUIRED)
find_package(legged_hw REQUIRED)
find_package(unitree_sdk2 REQUIRED)

add_library(${PROJECT_NAME} SHARED
        src/UnitreeHW.cpp
        )

target_include_directories(${PROJECT_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        )

ament_target_dependencies(${PROJECT_NAME}
        PUBLIC
        rclcpp
        geometry_msgs
        sensor_msgs
        std_msgs
        std_srvs
        hardware_interface
        pluginlib
        legged_common
        legged_hw
        )

target_link_libraries(${PROJECT_NAME}
        PUBLIC
        unitree_sdk2
        )

# Read-only transport diagnostic.  This executable deliberately creates no
# low-command publisher, so it can be used before enabling hardware motion.
add_executable(unitree_lowstate_monitor
        src/unitree_lowstate_monitor.cpp
        )
target_link_libraries(unitree_lowstate_monitor
        PRIVATE
        unitree_sdk2
        )

pluginlib_export_plugin_description_file(hardware_interface legged_unitree_hw_plugins.xml)

install(TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        )

install(TARGETS unitree_lowstate_monitor
        RUNTIME DESTINATION lib/${PROJECT_NAME}
        )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME}
        FILES_MATCHING PATTERN "*.h"
        )

install(DIRECTORY config launch
        DESTINATION share/${PROJECT_NAME}
        )

install(FILES README_REAL_GO2.md
        DESTINATION share/${PROJECT_NAME}
        )

install(EXPORT export_${PROJECT_NAME}
        DESTINATION share/${PROJECT_NAME}/cmake
        )

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
        rclcpp
        geometry_msgs
        sensor_msgs
        std_msgs
        std_srvs
        hardware_interface
        pluginlib
        legged_common
        legged_hw
        )

ament_package()
