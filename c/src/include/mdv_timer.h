/***************************************************************************//**
**
**  @file       mdv_timer.h
**  @ingroup    madivaru-lib
**  @brief      A general purpose timer API.
**  @copyright  Copyright (c) Tuomas Terho. All rights reserved.
**
**  A universal hardware independent timer interface.
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

#ifndef mdv_timer_H
#define mdv_timer_H

#include "mdv_timer_system.h"

/******************************************************************************\
**
**  TYPE DEFINITIONS
**
\******************************************************************************/

/*-------------------------------------------------------------------------*//**
**  @brief Order of magnitude (time).
*/
typedef enum
MdvTimerOrderOfMagnitude_t{
        // The order of magnitude equals to one timer tick.
        MDV_TIMER_OM_TIMERTICK=0,
        // The order of magnitude is microsecond.
        MDV_TIMER_OM_US,
        // The order of magnitude is millisecond.
        MDV_TIMER_OM_MS,
        // The order of magnitude is second.
        MDV_TIMER_OM_S
} MdvTimerOrderOfMagnitude_t;

/*-------------------------------------------------------------------------*//**
**  @brief Timer type.
*/
typedef struct MdvTimer_t{
        /// @brief Timer system that runs this timer.
        MdvTimerSystem_t *tsys;
        /// @brief Tick count in timer start.
        uint32_t tck;
        /// One time tick duration (in microseconds), inherited from the timer
        /// system.
        uint32_t ttd;
} MdvTimer_t;

/******************************************************************************\
**
**  API FUNCTION DECLARATIONS
**
\******************************************************************************/

#ifdef __cplusplus
extern "C"{
#endif // ifdef __cplusplus

/*-------------------------------------------------------------------------*//**
**  @brief Initializes a timer.
**
**  @param[in] timer Timer to create.
**  @param[in] tsys Timer system to use to run the timer.
**
**  @retval MDV_RESULT_OK Successful.
**  @retval MDV_ERROR_INVALID_POINTER Either the timer or tsys points to null.
*/
MdvResult_t
mdv_timer_init(
        MdvTimer_t *const timer,
        MdvTimerSystem_t *const tsys
);

/*-------------------------------------------------------------------------*//**
**  @brief Starts a timer.
**
**  @param[out] timer Timer to start.
**
**  @retval MDV_RESULT_OK Successful.
**  @retval MDV_ERROR_INVALID POINTER The timer points to null.
*/
MdvResult_t
mdv_timer_start(
        MdvTimer_t *const timer
);

/*-------------------------------------------------------------------------*//**
**  @brief Returns the time elapsed from the timer start.
**
**  @param[in] timer Timer in use.
**  @param[in] om The order of magnitude (time).
**  @param[out] time Time elapsed from the timer start.
**
**  @retval MDV_RESULT_OK Successful.
**  @retval MDV_ERROR_INVALID_POINTER The timer or time points to null.
*/
MdvResult_t
mdv_timer_get_time(
        MdvTimer_t *const timer,
        MdvTimerOrderOfMagnitude_t om,
        uint32_t *const time
);

#ifdef __cplusplus
}
#endif // ifdef __cplusplus

#endif // ifndef mdv_timer_H

/* EOF */
