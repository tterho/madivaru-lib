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

Result_t
TimerAPI_Init(
        TimerSys_t *timerSys,
        uint32_t timeBase,
        uint32_t timerInvokationLimit
)
{
        if(!timerSys){
                return TIMER_ERROR_INVALID_POINTER;
        }  
        timerSys->tck=0;
        timerSys->icnt=0;
        timerSys->tb=timeBase;
        timerSys->ilim=timerInvokationLimit;
        return RESULT_OK;
}

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
        timerSys->icnt=0;
        return RESULT_OK;
}

/*------------------------------------------------------------------------------
**  Returns the time lapse from the timer start.
*/
Result_t
TimerAPI_GetTimeLapse(
        TimerSys_t *timerSys,
        Timer_t timer,
        Timer_TimeUnit_t timeUnit,
        uint32_t *timeLapse
)
{
        uint32_t tl;
    
        if(!timerSys||!timeLapse){
                return TIMER_ERROR_INVALID_POINTER;
        }
        // If there is no change between the current timer value and the 
        // running tick counter, advance the invokation counter. Otherwise,
        // reset the invokation counter.
        if(timerSys->ctim==timerSys->tck){
                timerSys->icnt++;
        }else{
                timerSys->icnt=0;
        }
        // If the invokation counter has reached the timer invokation limit,
        // the timer is not running properly.
        if(timerSys->icnt>timerSys->ilim){
                return TIMER_ERROR_TIMER_NOT_RUNNING;
        }
        // Store the running tick counter and compare it to the given timer.
        // Handle a wrap-around of the tick counter. Store the difference to the 
        // output parameter.
        timerSys->ctim=timerSys->tck;
        tl=(timer<=timerSys->ctim)
           ?(timerSys->ctim-timer)
           :(0xffffffff-timer)+timerSys->ctim+1;
        // Calculate the time lapse based on the time base and the requested 
        // time units.
        switch(timeUnit){
        default:
        case TIMER_TU_TIMERTICK:
                // The timeLapse value is in correct units. Nothing to do.
                break;
        case TIMER_TU_US:
                tl=(tl*timerSys->tb);
        case TIMER_TU_MS:
                tl=(tl*timerSys->tb)/1000;
                break;
        case TIMER_TU_S:
                tl=(tl*timerSys->tb)/1000000;
                break;
        }
        *timeLapse=tl;
        return RESULT_OK;
}

/*------------------------------------------------------------------------------
**  Makes a delay in timer ticks.
*/
Result_t
TimerAPI_Delay(
        TimerSys_t *timerSys,
        Timer_TimeUnit_t timeUnit,
        uint32_t delay
)
{
        Timer_t t;
        uint32_t tl=0;
        Result_t result;
    
        if(!timerSys){
                return TIMER_ERROR_INVALID_POINTER;
        }
        // Start a timer.
        TimerAPI_StartTimer(timerSys,&t);
        // Wait as long as the time lapse is less than the delay.
        while(tl<delay){
                result=TimerAPI_GetTimeLapse(timerSys,t,timeUnit,&tl);
                if(!SUCCESSFUL(result)){
                        return result;
                }
        }
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