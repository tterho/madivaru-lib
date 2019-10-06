/***************************************************************************//**
**
**  @file       mdv_timer_system.h
**  @ingroup    madivaru-lib
**  @brief      A timer system for general purpose timers.
**  @copyright  Copyright (c) Tuomas Terho. All rights reserved.
**  @version    1.0
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

#ifndef mdv_timer_system_H
#define mdv_timer_system_H

#include "mdv_types.h"

/******************************************************************************\
**
**  ERROR CODES
**
\******************************************************************************/

/// @brief Timer system is not running.
///
/// Timer system is not running. The tick counter is not increased as expected.
#define MDV_TIMER_SYSTEM_ERROR_TIMER_NOT_RUNNING \
        MDV_MODULE_SPECIFIC_ERROR(0)

/******************************************************************************\
**
**  TYPE DEFINITIONS
**
\******************************************************************************/

/*-------------------------------------------------------------------------*//**
**  @brief Timer system structure.
*/
typedef struct
MdvTimerSystem_t{
        /// Current tick count helper variable.
        uint32_t ctck;
        /// Timer tick counter.
        uint32_t tck;
        /// Invocation counter.
        uint32_t icnt;
        /// Invocation limit.
        uint32_t ilim;
        /// One time tick duration (in microseconds).
        uint32_t ttd;
} MdvTimerSystem_t;

/******************************************************************************\
**
**  API FUNCTION DECLARATIONS
**
\******************************************************************************/

#ifdef __cplusplus
extern "C"{
#endif // ifdef __cplusplus

/*-------------------------------------------------------------------------*//**
**  @brief Initializes the timer system.
**
**  @param[in] tsys Timer system to be initialized.
**  @param[in] ttd Time tick duration in microseconds. This value specifies how
**             many microseconds one timer tick takes time. This timer system
**             doesn't support timers faster than one microsecond.
**  @param[in] ilim This limit specifies how many times the @ref
**             mdv_timer_get_time function can be invoked without increase in
**             the timer tick value before the system stops waiting. Set this
**             value depending on the ratio between the system clock and the
**             timer tick frequency. Too small limit value causes the timer to
**             get tired accidentally. However, keeping this as small as
**             possible enhances the system response on error situations.
**
**  @retval MDV_RESULT_OK Initialization successful.
**  @retval MDV_ERROR_INVALID_POINTER The tsys points to null.
*/
MdvResult_t
mdv_timer_system_init(
        MdvTimerSystem_t *const tsys,
        uint32_t ttd,
        uint32_t ilim
);

/*-------------------------------------------------------------------------*//**
**  @brief Advances the timer system by the given timer tick value.
**
**  @param[in] tsys Timer system to use.
**  @param[in] ticks Value to advance the timer.
**
**  @retval MDV_RESULT_OK Timer system advanced successfully.
**  @retval MDV_ERROR_INVALID_POINTER The tsys pointer points to null.
**
**  This function is used by the user application to run the timer system. The
**  interval and accuracy of invokation of this function declares the timer
**  resolution and quality.
*/
MdvResult_t
mdv_timer_system_tick(
        MdvTimerSystem_t *const tsys,
        uint32_t ticks
);

/*-------------------------------------------------------------------------*//**
**  @brief Returns the current tick count.
**
**  @param[in] tsys Timer system to use.
**  @param[out] ticks A pointer to a variable where to store the tick count.
**
**  @retval MDV_RESULT_OK Successful.
**  @retval MDV_ERROR_INVALID_POINTER Either the tsys or ticks pointer points to
**          null.
**  @retval MDV_TIMER_SYSTEM_ERROR_TIMER_NOT_RUNNING The timer system is not
**          working properly. Check that the @ref mdv_timer_system_tick function
**          is invoked periodically.
**
**  This function is used by @ref MdvTimer_t type timers.
*/
MdvResult_t
mdv_timer_system_get_tick_count(
        MdvTimerSystem_t *const tsys,
        uint32_t *const ticks
);

#ifdef __cplusplus
}
#endif // ifdef __cplusplus

#endif // ifndef mdv_timer_H

/* EOF */
