cmake_minimum_required(VERSION 3.8)
project(mtracker)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(LIGHT_INTENSITY_BH1750 "Decode and publish BH1750 light telemetry" OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(mtracker_core
    src/communication/protocol.cpp
    src/communication/serial_port.cpp
)
if(LIGHT_INTENSITY_BH1750)
    target_compile_definitions(mtracker_core PUBLIC LIGHT_INTENSITY_BH1750)
endif()
target_include_directories(mtracker_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

add_executable(mtracker_drv src/nodes/mtracker.cpp)
target_link_libraries(mtracker_drv mtracker_core)
ament_target_dependencies(mtracker_drv
    diagnostic_msgs
    geometry_msgs
    nav_msgs
    rclcpp
    std_msgs
    tf2
    tf2_geometry_msgs
    tf2_ros
)

add_executable(controller_simple src/control/controller_simple.cpp)
ament_target_dependencies(controller_simple
    geometry_msgs
    rclcpp
)

add_executable(trajectory_demo src/nodes/trajectory_demo.cpp)
target_include_directories(trajectory_demo PRIVATE include)
ament_target_dependencies(trajectory_demo geometry_msgs nav_msgs rclcpp)

add_executable(controller_tracking src/control/controller_tracking.cpp)
target_include_directories(controller_tracking PRIVATE include)
ament_target_dependencies(controller_tracking geometry_msgs nav_msgs rclcpp tf2 tf2_ros)

add_executable(set_odometry_demo src/nodes/set_odometry_demo.cpp)
ament_target_dependencies(set_odometry_demo
    geometry_msgs
    rclcpp
)

install(
    TARGETS mtracker_core mtracker_drv controller_simple set_odometry_demo
        trajectory_demo controller_tracking
    EXPORT export_mtracker
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(
    DIRECTORY config launch doc
    DESTINATION share/${PROJECT_NAME}
)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_cmake_pytest REQUIRED)
    find_package(ament_cmake_gtest REQUIRED)
    find_package(ament_cmake_cppcheck REQUIRED)
    find_package(ament_cmake_cpplint REQUIRED)
    find_package(ament_cmake_flake8 REQUIRED)
    find_package(ament_cmake_lint_cmake REQUIRED)
    find_package(ament_cmake_pep257 REQUIRED)
    find_package(ament_cmake_uncrustify REQUIRED)
    find_package(ament_cmake_xmllint REQUIRED)

    ament_cppcheck()
    ament_cpplint(FILTERS "-legal/copyright")
    ament_flake8()
    ament_lint_cmake()
    ament_pep257()
    ament_uncrustify()
    ament_xmllint()

    ament_add_gtest(test_protocol test/test_protocol.cpp)
    target_link_libraries(test_protocol mtracker_core)
    ament_add_pytest_test(test_circle_demo test/test_circle_demo.py)
    ament_add_gtest(test_demo_control test/test_demo_control.cpp)
    target_include_directories(test_demo_control PRIVATE include)
    ament_target_dependencies(test_demo_control geometry_msgs nav_msgs)
endif()

ament_export_targets(export_mtracker HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_package()
