#*******************************************************************************
#
#  madivaru-lib project CMake file
#  Copyright (c) Tuomas Terho. All rights reserved.
#
#*******************************************************************************

cmake_minimum_required(VERSION 3.10)
project(madivaru-lib-project C CXX)
include(GoogleTest)
add_subdirectory(googletest)
link_directories(${googletest_BINARY_DIR})
enable_testing()
add_subdirectory(c/test/timer_system)

# EOF