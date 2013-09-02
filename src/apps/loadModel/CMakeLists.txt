###############################################
# apps/loadModel
PROJECT(loadModel)
FILE(GLOB loadModel_srcs "*.cpp")
FILE(GLOB loadModel_hdrs "*.h")
ADD_EXECUTABLE(loadModel ${loadModel_srcs} ${loadModel_hdrs})
TARGET_LINK_LIBRARIES(loadModel kinematics dynamics yui renderer integration)
LINK_THIRDPARTY_GLUT(loadModel)
LINK_THIRDPARTY_TICPP(loadModel)

SET_TARGET_PROPERTIES(loadModel PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
