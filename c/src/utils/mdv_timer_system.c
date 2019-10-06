/***************************************************************************//**
**
**  @file       mdv_timer_system.c
**  @ingroup    madivaru-lib
**  @brief      A general purpose timer API.
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

#include "mdv_timer_system.h"

/******************************************************************************\
**
**  API FUNCTION DEFINITIONS
**
\******************************************************************************/

MdvResult_t
mdv_timer_system_init(
        MdvTimerSystem_t *const tsys,
        uint32_t ttd,
        uint32_t ilim
)
{
        if(!tsys){
                return MDV_ERROR_INVALID_POINTER;
        }
        if(!ttd){
                return MDV_ERROR_INVALID_PARAMETER;
        }
        tsys->tck=0;
        tsys->ctck=0;
        tsys->icnt=0;
        tsys->ttd=ttd;
        tsys->ilim=ilim;
        return MDV_RESULT_OK;
}

MdvResult_t
mdv_timer_system_tick(
        MdvTimerSystem_t *const tsys,
        uint32_t ticks
)
{
        if(!tsys){
                return MDV_ERROR_INVALID_POINTER;
        }
        tsys->tck+=ticks;
        return MDV_RESULT_OK;
}

MdvResult_t
mdv_timer_system_get_tick_count(
        MdvTimerSystem_t *tsys,
        uint32_t *ticks
)
{
        if(!tsys||!ticks){
                return MDV_ERROR_INVALID_POINTER;
        }
        // If there is no change between the last tick count value and the
        // running tick counter, and the invocation counting is enabled (limit
        // is greater than zero), advance the invocation counter. Otherwise,
        // reset the invocation counter.
        if((tsys->ctck==tsys->tck)&&(tsys->ilim)){
                ++tsys->icnt;
        }else{
                tsys->icnt=0;
        }
        // If the invokation counter has reached the timer invokation limit,
        // the timer is not running properly.
        if(tsys->icnt>tsys->ilim){
                return MDV_TIMER_SYSTEM_ERROR_TIMER_NOT_RUNNING;
        }
        // Store the running tick counter.
        tsys->ctck=tsys->tck;
        *ticks=tsys->tck;
        return MDV_RESULT_OK;
}

/* EOF */