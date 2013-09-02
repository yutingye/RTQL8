# Install script for directory: /Users/Tingting/Research/RTQL8/src/kinematics

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

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/rtql8/kinematics" TYPE FILE FILES
    "/Users/Tingting/Research/RTQL8/src/kinematics/BodyNode.h"
    "/Users/Tingting/Research/RTQL8/src/kinematics/C3D.h"
    "/Users/Tingting/Research/RTQL8/src/kinematics/Dof.h"
    "/Users/Tingting/Research/RTQL8/src/kinematics/FileInfoC3D.h"
    "/Users/Tingting/Research/RTQL8/src/kinematics/FileInfoDof.h"
    "/Users/Tingting/Research/RTQL8/src/kinematics/Joint.h"
    "/Users/Tingting/Research/RTQL8/src/kinematics/Marker.h"
    "/Users/Tingting/Research/RTQL8/src/kinematics/ParserSkel.h"
    "/Users/Tingting/Research/RTQL8/src/kinematics/ParserVsk.h"
    "/Users/Tingting/Research/RTQL8/src/kinematics/Shape.h"
    "/Users/Tingting/Research/RTQL8/src/kinematics/ShapeCube.h"
    "/Users/Tingting/Research/RTQL8/src/kinematics/ShapeEllipsoid.h"
    "/Users/Tingting/Research/RTQL8/src/kinematics/ShapeMesh.h"
    "/Users/Tingting/Research/RTQL8/src/kinematics/Skeleton.h"
    "/Users/Tingting/Research/RTQL8/src/kinematics/Transformation.h"
    "/Users/Tingting/Research/RTQL8/src/kinematics/TrfmRotateEuler.h"
    "/Users/Tingting/Research/RTQL8/src/kinematics/TrfmRotateExpmap.h"
    "/Users/Tingting/Research/RTQL8/src/kinematics/TrfmRotateQuat.h"
    "/Users/Tingting/Research/RTQL8/src/kinematics/TrfmTranslate.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/rtql8/kinematics" TYPE FILE FILES "/Users/Tingting/Research/RTQL8/src/kinematics/FileInfoSkel.hpp")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/Users/Tingting/Research/RTQL8/lib/libkinematics.a")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libkinematics.a" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libkinematics.a")
    EXECUTE_PROCESS(COMMAND "/usr/bin/ranlib" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libkinematics.a")
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

