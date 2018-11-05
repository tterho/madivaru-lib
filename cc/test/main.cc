#include "gtest/gtest.h"
#include "test_timer.cc"
#include "test_sensordrv.cc"

int main(int argc,char **argv){
        ::testing::InitGoogleTest(&argc,argv);
        return RUN_ALL_TESTS();
}