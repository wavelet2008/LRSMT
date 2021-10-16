# Install script for directory: /home/shipengl/LRSMT/env_build/eigen-3.3.2/Eigen

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/Eigen" TYPE FILE FILES
    "/home/shipengl/LRSMT/env_build/eigen-3.3.2/Eigen/Cholesky"
    "/home/shipengl/LRSMT/env_build/eigen-3.3.2/Eigen/CholmodSupport"
    "/home/shipengl/LRSMT/env_build/eigen-3.3.2/Eigen/Core"
    "/home/shipengl/LRSMT/env_build/eigen-3.3.2/Eigen/Dense"
    "/home/shipengl/LRSMT/env_build/eigen-3.3.2/Eigen/Eigen"
    "/home/shipengl/LRSMT/env_build/eigen-3.3.2/Eigen/Eigenvalues"
    "/home/shipengl/LRSMT/env_build/eigen-3.3.2/Eigen/Geometry"
    "/home/shipengl/LRSMT/env_build/eigen-3.3.2/Eigen/Householder"
    "/home/shipengl/LRSMT/env_build/eigen-3.3.2/Eigen/IterativeLinearSolvers"
    "/home/shipengl/LRSMT/env_build/eigen-3.3.2/Eigen/Jacobi"
    "/home/shipengl/LRSMT/env_build/eigen-3.3.2/Eigen/LU"
    "/home/shipengl/LRSMT/env_build/eigen-3.3.2/Eigen/MetisSupport"
    "/home/shipengl/LRSMT/env_build/eigen-3.3.2/Eigen/OrderingMethods"
    "/home/shipengl/LRSMT/env_build/eigen-3.3.2/Eigen/PaStiXSupport"
    "/home/shipengl/LRSMT/env_build/eigen-3.3.2/Eigen/PardisoSupport"
    "/home/shipengl/LRSMT/env_build/eigen-3.3.2/Eigen/QR"
    "/home/shipengl/LRSMT/env_build/eigen-3.3.2/Eigen/QtAlignedMalloc"
    "/home/shipengl/LRSMT/env_build/eigen-3.3.2/Eigen/SPQRSupport"
    "/home/shipengl/LRSMT/env_build/eigen-3.3.2/Eigen/SVD"
    "/home/shipengl/LRSMT/env_build/eigen-3.3.2/Eigen/Sparse"
    "/home/shipengl/LRSMT/env_build/eigen-3.3.2/Eigen/SparseCholesky"
    "/home/shipengl/LRSMT/env_build/eigen-3.3.2/Eigen/SparseCore"
    "/home/shipengl/LRSMT/env_build/eigen-3.3.2/Eigen/SparseLU"
    "/home/shipengl/LRSMT/env_build/eigen-3.3.2/Eigen/SparseQR"
    "/home/shipengl/LRSMT/env_build/eigen-3.3.2/Eigen/StdDeque"
    "/home/shipengl/LRSMT/env_build/eigen-3.3.2/Eigen/StdList"
    "/home/shipengl/LRSMT/env_build/eigen-3.3.2/Eigen/StdVector"
    "/home/shipengl/LRSMT/env_build/eigen-3.3.2/Eigen/SuperLUSupport"
    "/home/shipengl/LRSMT/env_build/eigen-3.3.2/Eigen/UmfPackSupport"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xDevelx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/Eigen" TYPE DIRECTORY FILES "/home/shipengl/LRSMT/env_build/eigen-3.3.2/Eigen/src" FILES_MATCHING REGEX "/[^/]*\\.h$")
endif()

