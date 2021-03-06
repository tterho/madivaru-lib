/***************************************************************************//**
**
**  @file       mdv_timer.c
**  @ingroup    madivaru-lib
**  @brief      A general purpose timer API.
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

#include "mdv_timer.h"

/******************************************************************************\
**
**  API FUNCTION DEFINITIONS
**
\******************************************************************************/

MdvResult_t
mdv_timer_init(
        MdvTimer_t *const timer,
        MdvTimerSystem_t *const tsys
)
{
        if(!tsys||!timer){
                return MDV_ERROR_INVALID_POINTER;
        }
        timer->tsys=tsys;
        timer->ttd=tsys->ttd;
        return MDV_RESULT_OK;
}

MdvResult_t
mdv_timer_start(
        MdvTimer_t *const timer
)
{
        if(!timer){
                return MDV_ERROR_INVALID_POINTER;
        }
        return mdv_timer_system_get_tick_count(timer->tsys,&(timer->tck));
}

MdvResult_t
mdv_timer_get_time(
        MdvTimer_t *const timer,
        MdvTimerOrderOfMagnitude_t om,
        uint32_t *const time
)
{
        MdvResult_t result;
        uint32_t tck;
        uint32_t tl;

        if(!timer||!time){
                return MDV_ERROR_INVALID_POINTER;
        }
        // Get the current tick count.
        result=mdv_timer_system_get_tick_count(timer->tsys,&tck);
        if(!MDV_SUCCESSFUL(result)){
                return result;
        }
        // Compare the tick count to the given timer. Handle a possible wrap-
        // around of the tick counter. Store the difference to the output
        // parameter.
        tl=(timer->tck<=tck)
           ?(tck-timer->tck)
           :(0xffffffff-timer->tck)+tck+1;
        // Calculate the time lapse based on the time base and the requested
        // order of magnitude.
        switch(om){
        default:
        case MDV_TIMER_OM_TIMERTICK:
                // The time value is in correct units. Nothing to do.
                break;
        case MDV_TIMER_OM_US:
                tl=(tl*timer->ttd);
        case MDV_TIMER_OM_MS:
                tl=(tl*timer->ttd)/1000;
                break;
        case MDV_TIMER_OM_S:
                tl=(tl*timer->ttd)/1000000;
                break;
        }
        *time=tl;
        return MDV_RESULT_OK;
}

/* EOF */