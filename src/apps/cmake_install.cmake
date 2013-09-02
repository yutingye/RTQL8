# Install script for directory: /Users/Tingting/Research/RTQL8/src/apps

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/usr/local")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "Release")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

IF(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  INCLUDE("/Users/Tingting/Research/RTQL8/src/apps/viewer/cmake_install.cmake")
  INCLUDE("/Users/Tingting/Research/RTQL8/src/apps/forwardSim/cmake_install.cmake")
  INCLUDE("/Users/Tingting/Research/RTQL8/src/apps/cubes/cmake_install.cmake")
  INCLUDE("/Users/Tingting/Research/RTQL8/src/apps/motionAnalysis/cmake_install.cmake")
  INCLUDE("/Users/Tingting/Research/RTQL8/src/apps/pdController/cmake_install.cmake")
  INCLUDE("/Users/Tingting/Research/RTQL8/src/apps/balance/cmake_install.cmake")
  INCLUDE("/Users/Tingting/Research/RTQL8/src/apps/ik/cmake_install.cmake")
  INCLUDE("/Users/Tingting/Research/RTQL8/src/apps/hybrid/cmake_install.cmake")
  INCLUDE("/Users/Tingting/Research/RTQL8/src/apps/closedLoop/cmake_install.cmake")
  INCLUDE("/Users/Tingting/Research/RTQL8/src/apps/hanging/cmake_install.cmake")
  INCLUDE("/Users/Tingting/Research/RTQL8/src/apps/holding/cmake_install.cmake")
  INCLUDE("/Users/Tingting/Research/RTQL8/src/apps/interacting/cmake_install.cmake")
  INCLUDE("/Users/Tingting/Research/RTQL8/src/apps/loadModel/cmake_install.cmake")
  INCLUDE("/Users/Tingting/Research/RTQL8/src/apps/forwardSim_world/cmake_install.cmake")
  INCLUDE("/Users/Tingting/Research/RTQL8/src/apps/cubes_world/cmake_install.cmake")
  INCLUDE("/Users/Tingting/Research/RTQL8/src/apps/balance_world/cmake_install.cmake")

ENDIF(NOT CMAKE_INSTALL_LOCAL_ONLY)

