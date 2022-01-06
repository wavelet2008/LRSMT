cmake_minimum_required(VERSION 3.5)
project(agiletaur_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(glog REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    /usr/include/eigen3
    /usr/local/include/qpOASES
    /usr/local/include/
    ${EIGEN3_INCLUDE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
)
add_executable( agiletaur_controller
    src/main.cpp

    src/controller/controller_monitor.cpp
    src/controller/pid_controller.cpp

    src/proxy/lowerproxy.cpp
    src/proxy/upperproxy.cpp
)

ament_target_dependencies(agiletaur_controller rclcpp std_msgs)

install(TARGETS
  agiletaur_controller
  DESTINATION lib/${PROJECT_NAME})

ament_package()