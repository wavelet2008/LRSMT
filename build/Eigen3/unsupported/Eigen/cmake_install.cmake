# Install script for directory: /home/shipengl/LRSMT/env_build/eigen-3.3.2/unsupported/Eigen

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/shipengl/LRSMT/install/Eigen3")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xDevelx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/unsupported/Eigen" TYPE FILE FILES
    "/home/shipengl/LRSMT/env_build/eigen-3.3.2/unsupported/Eigen/AdolcForward"
    "/home/shipengl/LRSMT/env_build/eigen-3.3.2/unsupported/Eigen/AlignedVector3"
    "/home/shipengl/LRSMT/env_build/eigen-3.3.2/unsupported/Eigen/ArpackSupport"
    "/home/shipengl/LRSMT/env_build/eigen-3.3.2/unsupported/Eigen/AutoDiff"
    "/home/shipengl/LRSMT/env_build/eigen-3.3.2/unsupported/Eigen/BVH"
    "/home/shipengl/LRSMT/env_build/eigen-3.3.2/unsupported/Eigen/EulerAngles"
    "/home/shipengl/LRSMT/env_build/eigen-3.3.2/unsupported/Eigen/FFT"
    "/home/shipengl/LRSMT/env_build/eigen-3.3.2/unsupported/Eigen/IterativeSolvers"
    "/home/shipengl/LRSMT/env_build/eigen-3.3.2/unsupported/Eigen/KroneckerProduct"
    "/home/shipengl/LRSMT/env_build/eigen-3.3.2/unsupported/Eigen/LevenbergMarquardt"
    "/home/shipengl/LRSMT/env_build/eigen-3.3.2/unsupported/Eigen/MatrixFunctions"
    "/home/shipengl/LRSMT/env_build/eigen-3.3.2/unsupported/Eigen/MoreVectorization"
    "/home/shipengl/LRSMT/env_build/eigen-3.3.2/unsupported/Eigen/MPRealSupport"
    "/home/shipengl/LRSMT/env_build/eigen-3.3.2/unsupported/Eigen/NonLinearOptimization"
    "/home/shipengl/LRSMT/env_build/eigen-3.3.2/unsupported/Eigen/NumericalDiff"
    "/home/shipengl/LRSMT/env_build/eigen-3.3.2/unsupported/Eigen/OpenGLSupport"
    "/home/shipengl/LRSMT/env_build/eigen-3.3.2/unsupported/Eigen/Polynomials"
    "/home/shipengl/LRSMT/env_build/eigen-3.3.2/unsupported/Eigen/Skyline"
    "/home/shipengl/LRSMT/env_build/eigen-3.3.2/unsupported/Eigen/SparseExtra"
    "/home/shipengl/LRSMT/env_build/eigen-3.3.2/unsupported/Eigen/SpecialFunctions"
    "/home/shipengl/LRSMT/env_build/eigen-3.3.2/unsupported/Eigen/Splines"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xDevelx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/unsupported/Eigen" TYPE DIRECTORY FILES "/home/shipengl/LRSMT/env_build/eigen-3.3.2/unsupported/Eigen/src" FILES_MATCHING REGEX "/[^/]*\\.h$")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/shipengl/LRSMT/build/Eigen3/unsupported/Eigen/CXX11/cmake_install.cmake")

endif()

