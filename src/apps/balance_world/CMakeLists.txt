###############################################
# apps/balance_world
PROJECT(balance_world)
FILE(GLOB collision_srcs "*.cpp")
FILE(GLOB collision_hdrs "*.h")
ADD_EXECUTABLE(balance_world ${collision_srcs} ${collision_hdrs})
TARGET_LINK_LIBRARIES(balance_world lcpsolver collision kinematics dynamics yui renderer integration simulation)
LINK_THIRDPARTY_GLUT(balance_world)
LINK_THIRDPARTY_TICPP(balance_world)

SET_TARGET_PROPERTIES(balance_world PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
