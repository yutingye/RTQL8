###############################################
# apps/interacting
PROJECT(interacting)
FILE(GLOB interacting_srcs "*.cpp")
FILE(GLOB interacting_hdrs "*.h")
ADD_EXECUTABLE(interacting ${interacting_srcs} ${interacting_hdrs})
TARGET_LINK_LIBRARIES(interacting lcpsolver collision kinematics dynamics yui renderer integration)
LINK_THIRDPARTY_GLUT(interacting)
LINK_THIRDPARTY_TICPP(interacting)

SET_TARGET_PROPERTIES(interacting PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
