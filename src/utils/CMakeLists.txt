###############################################
# Utils
PROJECT(utils)

# Search all header and source files
FILE(GLOB utils_srcs "*.cpp")
FILE(GLOB utils_hdrs "*.h")

# Library 
ADD_LIBRARY(utils ${utils_srcs} ${utils_hdrs})
SET_TARGET_PROPERTIES(utils PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
SET_TARGET_PROPERTIES(utils PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

INSTALL(FILES ${utils_hdrs} DESTINATION include/rtql8/utils)
INSTALL(TARGETS utils DESTINATION lib)
