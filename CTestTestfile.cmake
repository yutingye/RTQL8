# CMake generated Testfile for 
# Source directory: /Users/Tingting/Research/RTQL8
# Build directory: /Users/Tingting/Research/RTQL8
# 
# This file includes the relevent testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
ADD_TEST(unittests "/Users/Tingting/Research/RTQL8/bin/unittests")
ADD_TEST(test_core_collision "/Users/Tingting/Research/RTQL8/bin/test_core_collision")
SUBDIRS(thirdparty/ticpp)
SUBDIRS(src)
