#CMakeLists.txt

cmake_minimum_required( VERSION 2.8 )

Project(usqp)

set(CMAKE_BUILD_TYPE "Release" )

set(CMAKE_CXX_FLAGS "-O3" )

# 添加Eigen头文件
link_directories("usr/local/lib")
include_directories( "/usr/local/include/qpOASES" )

add_executable(test_qp test_qp.cpp)
target_link_libraries(test_qp libqpOASES.so)
