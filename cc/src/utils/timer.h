/***************************************************************************//**
**
**  @file       timer.h
**  @ingroup    utils
**  @brief      A general purpose timer.
**  @copyright  Copyright (C) 2012-2018 Tuomas Terho. All rights reserved.
**
**  A universal hardware independent timer.
**
*******************************************************************************/
/*
**  BSD 3-Clause License
**
**  Copyright (c) 2018, Tuomas Terho
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

#ifndef timer_H
#define timer_H

#include "types.h"

/******************************************************************************\
**
**  RESULT AND ERROR CODES
**
\******************************************************************************/

/// @brief Timer timeout.
#define TIMER_RESULT_TIMEOUT 1

/// @brief Timer tick counter is not running.
#define TIMER_ERROR_TIMER_NOT_RUNNING -1

/******************************************************************************\
**
**  TYPES
**
\******************************************************************************/

/*-------------------------------------------------------------------------*//**
**  @brief Time units.
*/
typedef enum
Timer_Units_t{
        // Time unit equals to one timer tick.
        TIMER_UNITS_TIMERTICK=0,
        // Time is in microseconds.
        TIMER_UNITS_US,
        // Time is in milliseconds.
        TIMER_UNITS_MS,
        // Time is in seconds.
        TIMER_UNITS_S
} Timer_Units_t;

/******************************************************************************\
**
**  CLASSES
**
\******************************************************************************/

/*-------------------------------------------------------------------------*//**
**  @brief Timer system class.
*/
class 
CTimerSys{
        public:
        /**
        **  @brief Constructor of the timer system class.
        **
        **  @param[in] timeBase Time base in microseconds. This value specifies
        **      how many microseconds one timer tick takes time. This timer
        **      system doesn't support timers faster than one microsecond.
        **  @param[in] timerInvocationLimit This limit specifies how many times
        **      the @ref Get function can be invoked without increase in the
        **      timer tick value before the system stops waiting. Set this value
        **      depending on the ratio between the system clock and the timer
        **      tick frequency. Too small limit value causes the timer to
        **      accidentally stop waiting. However, keeping this as small as
        **      possible enhances the system response on error situations.
        **
        **  @return No return value.
        */
        CTimerSys(
                uint32_t timeBase,
                uint32_t timerInvocationLimit
        );

        /**
        **  @brief Advances the tick counter by the given value.
        **
        **  @param[in] ticks Value to advance the counter.
        **
        **  @return No return value.
        */
        void Count(
                uint32_t ticks
        );

        /**
        **  @brief Gets the current tick counter value.
        **
        **  @return The current counter value.
        */
        uint32_t GetCount();

        /**
        ** @brief Gets the invocation limit.
        **
        ** @return The invocation limit of the timer system.
        */
        uint32_t GetInvocationLimit();

        /**
        ** @brief Gets the time base.
        **
        ** @return The time base of the timer system.
        */
        uint32_t GetTimeBase();

        private:

        /// Tick counter.
        uint32_t _tcnt;
        /// Time base of the timer (in microseconds).
        uint32_t _tb;
        /// Invocation limit.
        uint32_t _ilim;
};

/*-------------------------------------------------------------------------*//**
**  @brief Timer class.
*/
class CTimer{
        public:

        /**
        **  @brief Constructor of the CTimer class.
        **
        **  @param[in] timerSys Timer system the timer uses.
        */
        CTimer(CTimerSys *timerSys);

        /**
        **  @brief Starts the timer.
        **
        **  @return No return value.
        */
        void Start();

        /**
        **  @brief Tests a time-out time value.
        **
        **  @param[in] timeUnit Time units to use.
        **  @param[in] timeoutTime Timeout time value.
        **
        **  @retval RESULT_OK No timeout (timer is running successfully).
        **  @retval TIMER_RESULT_TIMEOUT A timeout occurred.
        **  @retval TIMER_ERROR_TIMER_NOT_RUNNING The timer is not running. The
        **      timer system tick counter doesn't increase as expected.
        */
        Result_t IsTimeout(
                Timer_Units_t timeUnit,
                uint32_t timeoutTime
        );

        /**
        **  @brief Makes a delay in timer units.
        **
        **  @param[in] timeUnit Time units to use.
        **  @param[in] delay Delay time in timer units.
        **
        **  @retval RESULT_OK Successful.
        **  @retval TIMER_ERROR_TIMER_NOT_RUNNING The timer system is not 
        **      running.
        */
        Result_t Delay(
                Timer_Units_t timeUnit,
                uint32_t delay
        );

        private:

        /// A pointer to a timer system.
        CTimerSys *_tsys;
        /// Timer start value.
        uint32_t _t;
        /// Invocation counter.
        uint32_t _icnt;
        /// Current counter helper value.
        uint32_t _ctcnt;
};

#endif // timer_H

/* EOF */
