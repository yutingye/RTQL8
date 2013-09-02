###############################################
# apps/forwardSim_world
PROJECT(forwardSim_world)
FILE(GLOB forwardSim_world_srcs "*.cpp")
FILE(GLOB forwardSim_world_hdrs "*.h")
ADD_EXECUTABLE(forwardSim_world ${forwardSim_world_srcs} ${forwardSim_world_hdrs})
TARGET_LINK_LIBRARIES(forwardSim_world kinematics dynamics yui renderer integration simulation)
LINK_THIRDPARTY_GLUT(forwardSim_world)
LINK_THIRDPARTY_TICPP(forwardSim_world)

SET_TARGET_PROPERTIES(forwardSim_world PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
