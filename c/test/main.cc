#include "gtest/gtest.h"
#include "../test/utils/test_fifo.cc"

#include <conio.h>

int main(
        int argc,
        char **argv
)
{
        int i;

        ::testing::InitGoogleTest(&argc,argv);
        i=RUN_ALL_TESTS();

        _getch();

        return i;
}