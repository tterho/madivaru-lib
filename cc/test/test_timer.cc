#include "gtest/gtest.h"
#include "utils/timer.h"

namespace Timer{

/// @brief Time base = 1000 us / counter tick.
#define TIME_BASE 1000U

/// @brief Invocation limit = 200 rounds
#define INVOCATION_LIMIT 200U

/// @brief Test delay time = 1000 ms (1000 counter ticks).
#define TEST_DELAY_TIME 1000U

/**
 * @brief Test the constructor of the CTimerSys class with null values.
 */
TEST(CTimerSys_class,Constructor_nullparams){
        CTimerSys tsys(0,0);

        // Test that the counter is resetted to zero.
        EXPECT_EQ(0U,tsys.GetCount());
        // The default value for the time base is 1.
        EXPECT_EQ(1,tsys.GetTimeBase());
        // The default value for the invocation limit is 1.000.000.
        EXPECT_EQ(1000000,tsys.GetInvocationLimit());
}

/**
 * @brief Test the constructor of the CTimerSys class.
 */
TEST(CTimerSys_class,Constructor){
        CTimerSys tsys(TIME_BASE,INVOCATION_LIMIT);

        // Test that the counter is resetted to zero.
        EXPECT_EQ(0U,tsys.GetCount());
        // Test that the time base equals to the constructor value.
        EXPECT_EQ(TIME_BASE,tsys.GetTimeBase());
        // Test that the invocation limit equals to the constructor value.
        EXPECT_EQ(INVOCATION_LIMIT,tsys.GetInvocationLimit());
}

/**
 * @brief Test the default constructor and SetTimerSys method of the CTimer
 *      class.
 */
TEST(CTimer_class,Constructor_and_SetTimerSys){
        CTimerSys tsys(TIME_BASE,INVOCATION_LIMIT);
        CTimer t;

        // Try to start the timer, expect an exception.
        EXPECT_THROW(t.Start(),int);
        // Try to check the timeout, expect an exception.
        EXPECT_THROW(t.IsTimeout(TIMER_UNITS_MS,TEST_DELAY_TIME),int);
        // Try to make a delay, expect an exception.
        EXPECT_THROW(t.Delay(TIMER_UNITS_MS,TEST_DELAY_TIME),int);

        t.SetTimerSys(&tsys);

        // Try to start the timer, do not expect an exception.
        EXPECT_NO_THROW(t.Start());
        // Try to check the timeout, do not expect an exception.
        EXPECT_NO_THROW(t.IsTimeout(TIMER_UNITS_MS,TEST_DELAY_TIME));
        // Try to make a delay, do not expect an exception.
        EXPECT_NO_THROW(t.Delay(TIMER_UNITS_MS,TEST_DELAY_TIME));
}

/**
 * @brief Test the Count method of the CTimerSys class.
 */
TEST(CTimerSys_class,Count){
        CTimerSys tsys(TIME_BASE,INVOCATION_LIMIT);

        // The initial counter value must be zero.
        EXPECT_EQ(0U,tsys.GetCount());
        // Advance the counter by 1. The return value must equal to 1.
        tsys.Count(1);
        EXPECT_EQ(1U,tsys.GetCount());
        // Advance the counter by 2. The return value must equal to 1+2=3.
        tsys.Count(2);
        EXPECT_EQ(3U,tsys.GetCount());
}

/**
 * @brief Test the basic operation of the Start and IsTimeout methods of the 
 *      CTimer class.
 */
TEST(CTimer_class,Start_and_IsTimeout_basic_operation){
        CTimerSys tsys(TIME_BASE,INVOCATION_LIMIT);
        CTimer t(&tsys);
        Result_t result;

        // Start the timer and get the initial result. The result must be 
        // RESULT_OK.
        t.Start();
        result=t.IsTimeout(TIMER_UNITS_MS,TEST_DELAY_TIME);
        EXPECT_EQ(RESULT_OK,result);

        // Advance the timer counter by TEST_DELAY_TIME-1 and check the timeout. 
        // The result must be RESULT_OK (not timeout yet).
        tsys.Count(TEST_DELAY_TIME-1);
        result=t.IsTimeout(TIMER_UNITS_MS,TEST_DELAY_TIME);
        EXPECT_EQ(RESULT_OK,result);

        // Advance the timer counter by 1 and check the timeout. The result must
        // be now TIMER_RESULT_TIMEOUT for timeout.
        tsys.Count(1);
        result=t.IsTimeout(TIMER_UNITS_MS,TEST_DELAY_TIME);
        EXPECT_EQ(TIMER_RESULT_TIMEOUT,result);

        // Start the timer again and test the result. The result must be 
        // RESULT_OK (no timeout).
        t.Start();
        result=t.IsTimeout(TIMER_UNITS_MS,TEST_DELAY_TIME);
}

/**
 * @brief Test the invocation limit of the Start and IsTimeout methods of the 
 *      CTimer class.
 */
TEST(CTimer_class,Start_and_IsTimeout_invocation_limit){
        CTimerSys tsys(TIME_BASE,INVOCATION_LIMIT);
        CTimer t(&tsys);
        Result_t result;
        uint32_t i;

        // Start the timer and get the initial result. The result must be 
        // RESULT_OK (timer not running).
        t.Start();
        result=t.IsTimeout(TIMER_UNITS_MS,TEST_DELAY_TIME);
        EXPECT_EQ(RESULT_OK,result);

        // Run the timeout check INVOCATION_LIMIT-1 times (timer not running). 
        // The result must be RESULT_OK.
        for(i=0;i<INVOCATION_LIMIT-1;i++){
                result=t.IsTimeout(TIMER_UNITS_MS,TEST_DELAY_TIME);
                EXPECT_EQ(RESULT_OK,result);
        }

        // Check the timeout once. The result must be now
        // TIMER_ERROR_TIMER_NOT_RUNNING as the invocation limit has been 
        // reached.
        result=t.IsTimeout(TIMER_UNITS_MS,TEST_DELAY_TIME);
        EXPECT_EQ(TIMER_ERROR_TIMER_NOT_RUNNING,result);

        // Advance the timer counter by 1 and check the timeout. The result
        // must be RESULT_OK (timer is running).
        tsys.Count(1);
        result=t.IsTimeout(TIMER_UNITS_MS,TEST_DELAY_TIME);
        EXPECT_EQ(RESULT_OK,result);
}

/**
 * @brief Test different time units with the Start and IsTimeout methods of the 
 *      CTimer class.
 */
TEST(CTimer_class,Start_and_IsTimeout_time_units){
        // Set the time base to 1 us.
        CTimerSys tsys(1,INVOCATION_LIMIT);
        CTimer t(&tsys);
        Result_t result;
        uint32_t i;

        // Start the timer.
        t.Start();

        // Advance the counter by one and check the timeout for microseconds,
        // milliseconds and seconds.
        tsys.Count(1);
        result=t.IsTimeout(TIMER_UNITS_US,1);
        EXPECT_EQ(TIMER_RESULT_TIMEOUT,result);
        result=t.IsTimeout(TIMER_UNITS_MS,1);
        EXPECT_EQ(RESULT_OK,result);
        result=t.IsTimeout(TIMER_UNITS_S,1);
        EXPECT_EQ(RESULT_OK,result);

        // Advance the counter by 999 (to value 1000) and check the timeout for 
        // microseconds, milliseconds and seconds.
        tsys.Count(999);
        result=t.IsTimeout(TIMER_UNITS_US,1);
        EXPECT_EQ(TIMER_RESULT_TIMEOUT,result);
        result=t.IsTimeout(TIMER_UNITS_MS,1);
        EXPECT_EQ(TIMER_RESULT_TIMEOUT,result);
        result=t.IsTimeout(TIMER_UNITS_S,1);
        EXPECT_EQ(RESULT_OK,result);

        // Advance the counter by 999.000 (to value 1.000.000) and check the 
        // timeout for microseconds, milliseconds and seconds.
        tsys.Count(999000);
        result=t.IsTimeout(TIMER_UNITS_US,1);
        EXPECT_EQ(TIMER_RESULT_TIMEOUT,result);
        result=t.IsTimeout(TIMER_UNITS_MS,1);
        EXPECT_EQ(TIMER_RESULT_TIMEOUT,result);
        result=t.IsTimeout(TIMER_UNITS_S,1);
        EXPECT_EQ(TIMER_RESULT_TIMEOUT,result);
}


/**
 * @brief Test the invocation limit on the Delay method of the CTimer class.
 */
TEST(CTimer_class,Delay_invocation_limit){
        CTimerSys tsys(TIME_BASE,INVOCATION_LIMIT);
        CTimer t(&tsys);
        Result_t result;

        result=t.Delay(TIMER_UNITS_MS,1000);
        EXPECT_EQ(TIMER_ERROR_TIMER_NOT_RUNNING,result);
}

} // namespace
