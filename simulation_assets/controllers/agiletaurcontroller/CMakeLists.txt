# set cmake version
cmake_minimum_required(VERSION 3.10)  
# set the project name
project(agiletaur_controller) 

# set the c++ 11 as standard
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 11)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
 
message("Hello Guys!!! Start compile!")

find_package(glog REQUIRED)
find_package(Eigen3 REQUIRED)

link_directories(
    ${PROJECT_SOURCE_DIR}/lib
    $ENV{WEBOTS_HOME}/lib/controller
)

set (LIBRARIES ${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX} ${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX})

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    $ENV{WEBOTS_HOME}/include/controller/c $ENV{WEBOTS_HOME}/include/controller/cpp
    /usr/include/eigen3
    /usr/local/include/qpOASES
    /usr/local/include/glog
    /usr/local/include/google
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

    src/config/config_parser.cpp
)

target_link_libraries(agiletaur_controller 
                      ${LIBRARIES}
                      ${GLOG_LIBRARIES}
                      glog::glog
                      qpOASES
                      config
                      )

install(
  TARGETS 
  agiletaur_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)