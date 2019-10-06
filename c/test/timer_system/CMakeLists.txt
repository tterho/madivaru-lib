#*******************************************************************************
#
#  madivaru-lib project CMake file
#  Copyright (c) Tuomas Terho. All rights reserved.
#
#*******************************************************************************

add_executable(
        madivaru_lib_unit_tests__timer_system
        test_timer_system.cpp
        ${PROJECT_SOURCE_DIR}/c/src/utils/mdv_timer_system.c
)

target_include_directories(
        madivaru_lib_unit_tests__timer_system
        PUBLIC
                ${PROJECT_SOURCE_DIR}/c/src/include
)

target_link_libraries(
        madivaru_lib_unit_tests__timer_system
        gtest
        gmock
        gtest_main
)

gtest_discover_tests(
        madivaru_lib_unit_tests__timer_system
        EXTRA_ARGS --gtest_output=xml:../../../test_results/
)

# EOF