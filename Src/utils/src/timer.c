/***************************************************************************//**
**
**  @file       timer.c
**  @ingroup    utils
**  @brief      A general purpose timer API.
**  @copyright  Copyright (C) 2012-2018 Tuomas Terho. All rights reserved.
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

#include "timer.h"

/******************************************************************************\
**
**  PUBLIC FUNCTION DECLARATIONS
**
\******************************************************************************/

/*------------------------------------------------------------------------------
**  Initializes the timer system.
*/
Result_t
TimerAPI_Init(
        TimerSys_t *timerSys
)
{
        if(!timerSys){
                return TIMER_ERROR_INVALID_POINTER;
        }  
        timerSys->tck=0;
        return RESULT_OK;
}

/*------------------------------------------------------------------------------
**  Starts a new timer.
*/
Result_t
TimerAPI_StartTimer(
        TimerSys_t *timerSys,
        Timer_t *timer
)
{
        if(!timerSys||!timer){
                return TIMER_ERROR_INVALID_POINTER;
        }
        *timer=timerSys->tck;
        return RESULT_OK;
}

/*------------------------------------------------------------------------------
**  Returns the time lapse from the timer start.
*/
Result_t
TimerAPI_GetTimeLapse(
        TimerSys_t *timerSys,
        Timer_t timer,
        uint32_t *timeLapse
)
{
        if(!timerSys||!timeLapse){
                return TIMER_ERROR_INVALID_POINTER;
        }
        timerSys->ctim=timerSys->tck;
        *timeLapse=(timer<=timerSys->ctim)
                   ?(timerSys->ctim-timer)
                   :(0xffffffff-timer)+timerSys->ctim+1;
        return RESULT_OK;
}

/*------------------------------------------------------------------------------
**  Advances the timer system by the given timer tick value.
*/
Result_t
TimerAPI_TimerTick(
        TimerSys_t *timerSys,
        uint32_t ticks
)
{
        if(!timerSys){
                return TIMER_ERROR_INVALID_POINTER;
        }
        timerSys->tck+=ticks;
        return RESULT_OK;
}

/* EOF */