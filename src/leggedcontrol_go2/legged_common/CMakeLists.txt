cmake_minimum_required(VERSION 3.8)
project(legged_common)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME}
        INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        )

ament_target_dependencies(${PROJECT_NAME}
        INTERFACE
        hardware_interface
        )

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

install(PROGRAMS scripts/generate_urdf.sh
        DESTINATION lib/${PROJECT_NAME}
        )

install(EXPORT export_${PROJECT_NAME}
        DESTINATION share/${PROJECT_NAME}/cmake
        )

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(hardware_interface)

ament_package()
