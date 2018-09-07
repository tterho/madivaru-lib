/***************************************************************************//**
**
**  @file       timer.h
**  @ingroup    utils
**  @brief      A general purpose timer API.
**  @copyright  Copyright (C) 2012-2018 Tuomas Terho. All rights reserved.
**
**  A universal hardware independent timer interface.
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
**  ERROR CODES
**
\******************************************************************************/

/// @brief Invalid pointer.
///
/// At least one of the pointer parameters is invalid (points to null).
#define TIMER_ERROR_INVALID_POINTER -1

/// @brief Timer is not running.
///
/// Timer system is not running. Timer tick counter don't increase as expected.
#define TIMER_ERROR_TIMER_NOT_RUNNING -2

/******************************************************************************\
**
**  TYPE DEFINITIONS
**
\******************************************************************************/

/*-------------------------------------------------------------------------*//** 
**  @brief Timer type.
*/
typedef uint32_t Timer_t;

/*-------------------------------------------------------------------------*//** 
**  @brief Timer system structure.
*/
typedef struct
TimerSys_t{
        /// Current time helper variable.
        Timer_t ctim;
        /// Timer tick counter.
        Timer_t tck;
        /// Invokation counter.
        uint32_t icnt;
        /// Invokation limit.
        uint32_t ilim;
        /// Time base of the timer (in microseconds).
        uint32_t tb;
} TimerSys_t;

/*-------------------------------------------------------------------------*//** 
**  @brief Time units.
*/
typedef enum
Timer_TimeUnit_t{
        // Time unit equals to one timer tick.
        TIMER_TU_TIMERTICK=0,
        // Time is in microseconds.
        TIMER_TU_US,
        // Time is in milliseconds.
        TIMER_TU_MS,
        // Time is in seconds.
        TIMER_TU_S        
} Timer_TimeUnit_t;

/******************************************************************************\
**
**  PUBLIC FUNCTION DEFINITIONS
**
\******************************************************************************/

/*-------------------------------------------------------------------------*//** 
**  @brief Initializes the timer system.
**
**  @param[in] timerSys Timer system to be initialized.
**  @param[in] timeBase Time base in microseconds. This value specifies how many
**      microseconds one timer tick takes time. This timer system doesn't 
**      support timers faster than one microsecond.
**  @param[in] timerInvokationLimit This limit specifies how many times the @ref
**      TimerAPI_GetTimeLapse function can be invoked without increase in the 
**      timer tick value before the system gets tired. Set this value depending 
**      on the ratio between the system clock and the timer tick frequency. Too 
**      small limit value causes the timer to  get tired accidentally. However, 
**      keeping this as small as possible enhances the system response on error
**      situations.
**
**  @retval RESULT_OK Initialization successful.
**  @retval TIMER_ERROR_INVALID_POINTER The timerSys points to null.
*/
Result_t 
TimerAPI_Init(
        TimerSys_t *timerSys,
        uint32_t timeBase,
        uint32_t timerInvokationLimit
);

/*-------------------------------------------------------------------------*//** 
**  @brief Starts a new timer.
**
**  @param[in] timerSys Timer system to use.
**  @param[out] timer Timer to start.
**
**  @retval RESULT_OK Timer started successfully.
**  @retval TIMER_ERROR_INVALID POINTER Either the timerSys or timer points to
**      null.
*/
Result_t
TimerAPI_StartTimer(
        TimerSys_t *timerSys,
        Timer_t *timer
);

/*-------------------------------------------------------------------------*//** 
**  @brief Returns the time lapse from the timer start.
**
**  @param[in] timerSys Timer system to use.
**  @param[in] timer Timer to get the time lapse.
**  @param[in] timeUnit Time units to use.
**  @param[out] timeLapse Time lapse from the timer start.
**
**  @retval RESULT_OK Got time lapse successfully.
**  @retval TIMER_ERROR_INVALID_POINTER Either the timerSys or the timeLapse
**      points to null.
*/
Result_t
TimerAPI_GetTimeLapse(
        TimerSys_t *timerSys,
        Timer_t timer,
        Timer_TimeUnit_t timeUnit,
        uint32_t *timeLapse
);

/*-------------------------------------------------------------------------*//** 
**  @brief Makes a delay in timer ticks.
**
**  @param[in] timerSys Timer system to use.
**  @param[in] timeUnit Time units to use.
**  @param[in] delay Delay time in timer ticks.
**
**  @retval RESULT_OK Successful.
**  @retval TIMER_ERROR_INVALID_POINTER The timerSys parameter points to null.
*/
Result_t
TimerAPI_Delay(
        TimerSys_t *timerSys,
        Timer_TimeUnit_t timeUnit,
        uint32_t delay
);

/*-------------------------------------------------------------------------*//** 
**  @brief Advances the timer system by the given timer tick value.
**
**  @param[in] timerSys Timer system to use.
**  @param[in] ticks Value to advance the timer.
**
**  @retval RESULT_OK Timer system advanced successfully.
**  @retval TIMER_ERROR_INVALID_POINTER The timerSys pointer points to null.
**
**  This function is used by the user application to run the timer system. The 
**  interval and accuracy of invokation of this function declares the timer 
**  resolution and quality.
*/
Result_t
TimerAPI_TimerTick(
        TimerSys_t *timerSys,
        uint32_t ticks
);

#endif // timer_H

/* EOF */
