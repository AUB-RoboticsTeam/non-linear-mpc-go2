cmake_minimum_required(VERSION 3.8)
project(legged_wbc)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(ocs2_centroidal_model REQUIRED)
find_package(ocs2_legged_robot REQUIRED)
find_package(ocs2_pinocchio_interface REQUIRED)
find_package(qpoases_catkin REQUIRED)
find_package(pinocchio REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Boost REQUIRED COMPONENTS system filesystem log_setup log)

add_library(${PROJECT_NAME}
        src/HoQp.cpp
        src/WbcBase.cpp
        src/HierarchicalWbc.cpp
        src/WeightedWbc.cpp
        )

target_include_directories(${PROJECT_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${EIGEN3_INCLUDE_DIRS}
        ${pinocchio_INCLUDE_DIRS}
        ${qpoases_catkin_INCLUDE_DIRS}
        )

target_link_libraries(${PROJECT_NAME}
        PUBLIC
        pinocchio::pinocchio
        qpoases_catkin::qpOASES
        )

ament_target_dependencies(${PROJECT_NAME}
        PUBLIC
        ocs2_core
        ocs2_centroidal_model
        ocs2_legged_robot
        ocs2_pinocchio_interface
        qpoases_catkin
        )

target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS} ${OCS2_PINOCCHIO_FLAGS})

#############
## Install ##
#############

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
        ocs2_core
        ocs2_centroidal_model
        ocs2_legged_robot
        ocs2_pinocchio_interface
        qpoases_catkin
        pinocchio
        Eigen3
        )

#############
## Testing ##
#############

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)

    ament_add_gtest(${PROJECT_NAME}_test
            test/HoQp_test.cpp
            )

    target_link_libraries(${PROJECT_NAME}_test
            ${PROJECT_NAME}
            gtest_main
            )

    target_compile_options(${PROJECT_NAME}_test PRIVATE ${OCS2_CXX_FLAGS} ${OCS2_PINOCCHIO_FLAGS})
endif()

ament_package()
