/***************************************************************************//**
**
**  @file       test_timer_system.cpp
**  @ingroup    madivaru-lib
**  @brief      Unit tests for mdv_timer_system module.
**  @copyright  Copyright (c) Tuomas Terho. All rights reserved.
**
*******************************************************************************/
/*
**  BSD 3-Clause License
**
**  Copyright (c) Tuomas Terho
**  All rights reserved.
**
**  Redistribution and use in source and binary forms, with or without
**  modification, are permitted provided that the following conditions are met:
**
**  * Redistributions of source code must retain the above copyright notice,
**    this list of conditions and the following disclaimer.
**
**  * Redistributions in binary form must reproduce the above copyright notice,
**    this list of conditions and the following disclaimer in the documentation
**    and/or other materials provided with the distribution.
**
**  * Neither the name of the copyright holder nor the names of its
**    contributors may be used to endorse or promote products derived from
**    this software without specific prior written permission.
**
**  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
**  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
**  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
**  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
**  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
**  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
**  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
**  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
**  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
**  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
**  POSSIBILITY OF SUCH DAMAGE.
**
\******************************************************************************/

#include "gtest/gtest.h"
#include "mdv_timer_system.h"

// Test value for timer tick duration.
#define TEST_TIMER_SYSTEM_TTD 100
// Test value for invocation limit.
#define TEST_TIMER_SYSTEM_ILIM 10000

namespace{

class mdv_timer_system : public ::testing::Test
{
        protected:

        void SetUp() override {
                result=MDV_RESULT_OK;
                tsys=(MdvTimerSystem_t){0};
        }

        void TearDown() override {
        }

        MdvTimerSystem_t tsys;
        MdvResult_t result;
};

TEST_F(mdv_timer_system,_init__invalid_tsys_pointer_causes_error)
{
        result=mdv_timer_system_init(
                0,
                TEST_TIMER_SYSTEM_TTD,
                TEST_TIMER_SYSTEM_ILIM
        );
        EXPECT_EQ(MDV_ERROR_INVALID_POINTER,result)
                << "An error must be returned on an invalid pointer.";
}

TEST_F(mdv_timer_system,_init__invalid_ttd_value_causes_error)
{
        result=mdv_timer_system_init(&tsys,0,TEST_TIMER_SYSTEM_ILIM);
        EXPECT_EQ(MDV_ERROR_INVALID_PARAMETER,result)
                << "An error must be returned on invalid timer tick duration "\
                   "value.";
}

TEST_F(mdv_timer_system,_init__timer_system_initialized)
{
        memset(&tsys,0xff,sizeof(MdvTimerSystem_t));
        result=mdv_timer_system_init(
                &tsys,
                TEST_TIMER_SYSTEM_TTD,
                TEST_TIMER_SYSTEM_ILIM
        );
        EXPECT_EQ(0,tsys.tck)
                << "The tick counter must be resetted to zero.";
        EXPECT_EQ(0,tsys.ctck)
                << "The temporary current tick count value must be resetted "\
                   "to zero.";
        EXPECT_EQ(0,tsys.icnt)
                << "The invocation counter must be resetted to zero.";
        EXPECT_EQ(TEST_TIMER_SYSTEM_TTD,tsys.ttd)
                << "The time tick duration must be initialized.";
        EXPECT_EQ(TEST_TIMER_SYSTEM_ILIM,tsys.ilim)
                << "The invocation limit value must be initialized.";
        EXPECT_EQ(MDV_RESULT_OK,result)
                << "Must return OK when succeeded.";
}

TEST_F(mdv_timer_system,_tick__invalid_tsys_pointer_causes_error)
{
        result=mdv_timer_system_tick(
                0,
                1
        );
        EXPECT_EQ(MDV_ERROR_INVALID_POINTER,result)
                << "An error must be returned on an invalid pointer.";
}

TEST_F(mdv_timer_system,_tick__increase_tick_counter)
{
        tsys.tck=0;
        // Increase ticks by one.
        result=mdv_timer_system_tick(&tsys,1);
        EXPECT_EQ(1,tsys.tck)
                << "Tick counter must be increased by the given value.";
        EXPECT_EQ(MDV_RESULT_OK,result)
                << "Must return OK when succeeded.";
        result=mdv_timer_system_tick(
                &tsys,
                123 // Another value.
        );
        EXPECT_EQ(1+123,tsys.tck)
                << "Tick counter must be increased by the given value.";
        EXPECT_EQ(MDV_RESULT_OK,result)
                << "Must return OK when succeeded.";
}

TEST_F(mdv_timer_system,_get_tick_count__invalid_tsys_pointer_causes_error)
{
        uint32_t ticks;
        result=mdv_timer_system_get_tick_count(0,&ticks);
        EXPECT_EQ(MDV_ERROR_INVALID_POINTER,result)
                << "An error must be returned on an invalid pointer.";
}

TEST_F(mdv_timer_system,_get_tick_count__invalid_ticks_pointer_causes_error)
{
        result=mdv_timer_system_get_tick_count(&tsys,0);
        EXPECT_EQ(MDV_ERROR_INVALID_POINTER,result)
                << "An error must be returned on an invalid pointer.";
}

TEST_F(mdv_timer_system,_get_tick_count__invocation_counter_zero_if_not_enabled)
{
        uint32_t ticks;

        tsys.tck=123; // Some value.
        tsys.ctck=tsys.tck; // Current tick count equals to the tick counter.
        tsys.ilim=0; // Invocation limit not enabled (is zero).
        tsys.icnt=234; // Some value.
        result=mdv_timer_system_get_tick_count(&tsys,&ticks);
        EXPECT_EQ(0,tsys.icnt)
                << "Invocation counter must be reset to zero.";
        EXPECT_EQ(MDV_RESULT_OK,result)
                << "Must return OK when succeeded.";
}

TEST_F(mdv_timer_system,_get_tick_count__invocation_counter_increased_if_ticks_not_increased)
{
        uint32_t ticks;

        tsys.tck=123; // Some value.
        tsys.ctck=tsys.tck; // Current tick count equals to the tick counter.
        tsys.ilim=TEST_TIMER_SYSTEM_ILIM;
        tsys.icnt=0;
        result=mdv_timer_system_get_tick_count(&tsys,&ticks);
        EXPECT_EQ(1,tsys.icnt)
                << "Invocation counter must be increased.";
        EXPECT_EQ(MDV_RESULT_OK,result)
                << "Must return OK when succeeded.";
}

TEST_F(mdv_timer_system,_get_tick_count__error_when_invocation_limit_exceeded)
{
        uint32_t ticks;

        tsys.tck=123; // Some value.
        tsys.ctck=tsys.tck; // Current tick count equals to the tick counter.
        tsys.ilim=TEST_TIMER_SYSTEM_ILIM;
        tsys.icnt=TEST_TIMER_SYSTEM_ILIM; // Limit previously reached.
        result=mdv_timer_system_get_tick_count(&tsys,&ticks);
        EXPECT_EQ(MDV_TIMER_SYSTEM_ERROR_TIMER_NOT_RUNNING,result)
                << "Error must be returned if the timer is not running.";
}

TEST_F(mdv_timer_system,_get_tick_count__ticks_successfully_increased)
{
        uint32_t ticks;

        tsys.tck=123; // Some value.
        tsys.ctck=tsys.tck; // Current tick count equals to the tick counter.
        tsys.ilim=TEST_TIMER_SYSTEM_ILIM;
        tsys.icnt=234; // Some value.
        mdv_timer_system_tick(&tsys,1);
        result=mdv_timer_system_get_tick_count(&tsys,&ticks);
        EXPECT_EQ(0,tsys.icnt)
                << "Invocation counter must be reset to zero.";
        EXPECT_EQ(123+1,tsys.ctck)
                << "Current tick count must be set to the tick counter value.";
        EXPECT_EQ(123+1,ticks)
                << "The output value must be set.";
        EXPECT_EQ(MDV_RESULT_OK,result)
                << "Must return OK when succeeded.";
}

} // namespace