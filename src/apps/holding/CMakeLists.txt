###############################################
# apps/holding
PROJECT(holding)
FILE(GLOB holding_srcs "*.cpp")
FILE(GLOB holding_hdrs "*.h")
ADD_EXECUTABLE(holding ${holding_srcs} ${holding_hdrs})
TARGET_LINK_LIBRARIES(holding lcpsolver collision kinematics dynamics yui renderer integration)
LINK_THIRDPARTY_GLUT(holding)
LINK_THIRDPARTY_TICPP(holding)

SET_TARGET_PROPERTIES(holding PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
