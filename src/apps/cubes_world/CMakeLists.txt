###############################################
# apps/cubes_world
PROJECT(cubes_world)
FILE(GLOB cubes_world_srcs "*.cpp")
FILE(GLOB cubes_world_hdrs "*.h")
ADD_EXECUTABLE(cubes_world ${cubes_world_srcs} ${cubes_world_hdrs})
TARGET_LINK_LIBRARIES(cubes_world lcpsolver collision kinematics dynamics yui renderer integration simulation)
LINK_THIRDPARTY_GLUT(cubes_world)
LINK_THIRDPARTY_TICPP(cubes_world)

SET_TARGET_PROPERTIES(cubes_world PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
