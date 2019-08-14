/***************************************************************************//**
**
**  @file       mdv_timer.h
**  @ingroup    madivaru-lib
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

#ifndef mdv_timer_H
#define mdv_timer_H

#include "mdv_types.h"

#ifdef __cplusplus
extern "C"{
#endif // ifdef __cplusplus

/******************************************************************************\
**
**  ERROR CODES
**
\******************************************************************************/

/// @brief Timer is not running.
///
/// Timer system is not running. Timer tick counter don't increase as expected.
#define MDV_TIMER_ERROR_TIMER_NOT_RUNNING \
        MDV_MODULE_SPECIFIC_ERROR(0)

/******************************************************************************\
**
**  TYPE DEFINITIONS
**
\******************************************************************************/

/*-------------------------------------------------------------------------*//**
**  @brief Timer type.
*/
typedef uint32_t MdvTimer_t;

/*-------------------------------------------------------------------------*//**
**  @brief Timer system structure.
*/
typedef struct
MdvTimerSystem_t{
        /// Current time helper variable.
        MdvTimer_t ctim;
        /// Timer tick counter.
        MdvTimer_t tck;
        /// Invokation counter.
        uint32_t icnt;
        /// Invokation limit.
        uint32_t ilim;
        /// Time base of the timer (in microseconds).
        uint32_t tb;
} MdvTimerSystem_t;

/*-------------------------------------------------------------------------*//**
**  @brief Time units.
*/
typedef enum
MdvTimeUnit_t{
        // Time unit equals to one timer tick.
        MDV_TIME_UNIT_TIMERTICK=0,
        // Time is in microseconds.
        MDV_TIME_UNIT_US,
        // Time is in milliseconds.
        MDV_TIME_UNIT_MS,
        // Time is in seconds.
        MDV_TIME_UNIT_S
} MdvTimeUnit_t;

/******************************************************************************\
**
**  API FUNCTION DECLARATIONS
**
\******************************************************************************/

/*-------------------------------------------------------------------------*//**
**  @brief Initializes the timer system.
**
**  @param[in] timerSys Timer system to be initialized.
**  @param[in] timeBase Time base in microseconds. This value specifies how many
**      microseconds one timer tick takes time. This timer system doesn't
**      support timers faster than one microsecond.
**  @param[in] timerInvocationLimit This limit specifies how many times the @ref
**      mdv_timer_get_time function can be invoked without increase in the
**      timer tick value before the system stops waiting. Set this value
**      depending on the ratio between the system clock and the timer tick
**      frequency. Too small limit value causes the timer to  get tired
**      accidentally. However, keeping this as small as possible enhances the
**      system response on error situations.
**
**  @retval MDV_RESULT_OK Initialization successful.
**  @retval MDV_ERROR_INVALID_POINTER The timerSys points to null.
*/
MdvResult_t
mdv_timer_system_init(
        MdvTimerSystem_t *timerSys,
        uint32_t timeBase,
        uint32_t timerInvocationLimit
);

/*-------------------------------------------------------------------------*//**
**  @brief Advances the timer system by the given timer tick value.
**
**  @param[in] timerSys Timer system to use.
**  @param[in] ticks Value to advance the timer.
**
**  @retval MDV_RESULT_OK Timer system advanced successfully.
**  @retval MDV_ERROR_INVALID_POINTER The timerSys pointer points to null.
**
**  This function is used by the user application to run the timer system. The
**  interval and accuracy of invokation of this function declares the timer
**  resolution and quality.
*/
MdvResult_t
mdv_timer_system_tick(
        MdvTimerSystem_t *timerSys,
        uint32_t ticks
);

/*-------------------------------------------------------------------------*//**
**  @brief Starts a timer.
**
**  @param[in] timerSys Timer system to use.
**  @param[out] timer Timer to start.
**
**  @retval MDV_RESULT_OK Timer started successfully.
**  @retval MDV_ERROR_INVALID POINTER Either the timerSys or timer points to
**      null.
*/
MdvResult_t
mdv_timer_start(
        MdvTimerSystem_t *timerSys,
        MdvTimer_t *timer
);

/*-------------------------------------------------------------------------*//**
**  @brief Returns the time elapsed from the timer start.
**
**  @param[in] timerSys Timer system to use.
**  @param[in] timer Timer to get the time lapse.
**  @param[in] timeUnit Time units to use.
**  @param[out] timeElapsed Time elapsed from the timer start.
**
**  @retval MDV_RESULT_OK Got time lapse successfully.
**  @retval MDV_ERROR_INVALID_POINTER Either the timerSys or the timeLapse
**      points to null.
*/
MdvResult_t
mdv_timer_get_time(
        MdvTimerSystem_t *timerSys,
        MdvTimer_t timer,
        MdvTimeUnit_t timeUnit,
        uint32_t *timeElapsed
);

/*-------------------------------------------------------------------------*//**
**  @brief Makes a delay in timer ticks.
**
**  @param[in] timerSys Timer system to use.
**  @param[in] timeUnit Time units to use.
**  @param[in] delay Delay time in timer ticks.
**
**  @retval MDV_RESULT_OK Successful.
**  @retval MDV_ERROR_INVALID_POINTER The timerSys parameter points to null.
*/
MdvResult_t
mdv_timer_delay(
        MdvTimerSystem_t *timerSys,
        MdvTimeUnit_t timeUnit,
        uint32_t delay
);

#ifdef __cplusplus
}
#endif // ifdef __cplusplus

#endif // ifndef mdv_timer_H

/* EOF */
