###############################################
# Dynamics
PROJECT(dynamics)

# Search all header and source files
FILE(GLOB dynamics_srcs "*.cpp")
FILE(GLOB dynamics_hdrs "*.h")

# Library 
ADD_LIBRARY(dynamics ${dynamics_srcs} ${dynamics_hdrs})
TARGET_LINK_LIBRARIES(dynamics lcpsolver collision)
SET_TARGET_PROPERTIES(dynamics PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
SET_TARGET_PROPERTIES(dynamics PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

INSTALL(FILES ${dynamics_hdrs} DESTINATION include/rtql8/dynamics)
INSTALL(TARGETS dynamics DESTINATION lib)
