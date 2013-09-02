# Install script for directory: /Users/Tingting/Research/RTQL8/src/collision

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
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/rtql8/collision" TYPE FILE FILES
    "/Users/Tingting/Research/RTQL8/src/collision/aabb.h"
    "/Users/Tingting/Research/RTQL8/src/collision/BV.h"
    "/Users/Tingting/Research/RTQL8/src/collision/BV_fitter.h"
    "/Users/Tingting/Research/RTQL8/src/collision/BVH_defs.h"
    "/Users/Tingting/Research/RTQL8/src/collision/BVH_front.h"
    "/Users/Tingting/Research/RTQL8/src/collision/BVH_model.h"
    "/Users/Tingting/Research/RTQL8/src/collision/BVH_split_rule.h"
    "/Users/Tingting/Research/RTQL8/src/collision/collision.h"
    "/Users/Tingting/Research/RTQL8/src/collision/collision_geom.h"
    "/Users/Tingting/Research/RTQL8/src/collision/collision_primitive.h"
    "/Users/Tingting/Research/RTQL8/src/collision/CollisionShapes.h"
    "/Users/Tingting/Research/RTQL8/src/collision/CollisionSkeleton.h"
    "/Users/Tingting/Research/RTQL8/src/collision/conservative_advancement.h"
    "/Users/Tingting/Research/RTQL8/src/collision/continuous_collision.h"
    "/Users/Tingting/Research/RTQL8/src/collision/distance_primitive.h"
    "/Users/Tingting/Research/RTQL8/src/collision/intersect.h"
    "/Users/Tingting/Research/RTQL8/src/collision/kDOP.h"
    "/Users/Tingting/Research/RTQL8/src/collision/obb.h"
    "/Users/Tingting/Research/RTQL8/src/collision/primitive.h"
    "/Users/Tingting/Research/RTQL8/src/collision/proximity.h"
    "/Users/Tingting/Research/RTQL8/src/collision/roughly_comp.h"
    "/Users/Tingting/Research/RTQL8/src/collision/rss.h"
    "/Users/Tingting/Research/RTQL8/src/collision/tri_tri_intersection_test.h"
    "/Users/Tingting/Research/RTQL8/src/collision/vec_3f.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/Users/Tingting/Research/RTQL8/lib/libcollision.a")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcollision.a" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcollision.a")
    EXECUTE_PROCESS(COMMAND "/usr/bin/ranlib" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcollision.a")
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

