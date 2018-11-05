#include "timer.h"

CTimerSys::CTimerSys(
        uint32_t timeBase,
        uint32_t timerInvocationLimit
)
        : _tcnt(0),
        _tb(timeBase),
        _ilim(timerInvocationLimit)
{
        if(!_tb){
                _tb=1;
        }

        if(!_ilim){
                _ilim=1000000;
        }
}

void CTimerSys::Count(
        uint32_t ticks
)
{
        _tcnt+=ticks;
}

uint32_t CTimerSys::GetCount(
)
{
        return _tcnt;
}

uint32_t CTimerSys::GetInvocationLimit(
)
{
        return _ilim;
}

uint32_t CTimerSys::GetTimeBase(
)
{
        return _tb;
}

CTimer::CTimer()
        :
        _tsys(0),
        _t(0),
        _icnt(0),
        _ctcnt(0)
{
}

CTimer::CTimer(
        CTimerSys *timerSys
):
        _tsys(timerSys),
        _t(0),
        _icnt(0),
        _ctcnt(0)
{
}

void CTimer::SetTimerSys(
        CTimerSys *timerSys
)
{
        _tsys=timerSys;
}

void CTimer::Start(
)
{
        if(!_tsys){
                throw(TIMER_ERROR_NO_TIMER_SYSTEM);
        }

        // Get the current tick counter value and reset the invocation counter
        // and the current tick counter helper value.
        _t=_tsys->GetCount();
        _icnt=0;
        _ctcnt=_t;
}

Result_t CTimer::IsTimeout(
        Timer_Units_t timerUnits,
        uint32_t timeoutTime
)
{
        uint32_t t;
        uint32_t tl;

        if(!_tsys){
                throw(TIMER_ERROR_NO_TIMER_SYSTEM);
        }

        t=_tsys->GetCount();
        if(_ctcnt==t){
                _icnt++;
        }else{
                _icnt=0;
        }
        // If the invokation counter has reached the timer invokation limit,
        // the timer is not running properly.
        if(_icnt>_tsys->GetInvocationLimit()){
                return TIMER_ERROR_TIMER_NOT_RUNNING;
        }
        // Store the running tick counter and compare it to the given timer.
        // Handle a wrap-around of the tick counter. Store the difference to the 
        // output parameter.
        _ctcnt=t;
        tl=(_t<=_ctcnt)?(_ctcnt-_t):(0xffffffff-_t)+_ctcnt+1;
        // Calculate the time lapse based on the timer system time base and the 
        // requested time units.
        switch(timerUnits){
        default:
        case TIMER_UNITS_TIMERTICK:
                // The timeLapse value is in correct units. Nothing to do.
                break;
        case TIMER_UNITS_US:
                tl=(tl*_tsys->GetTimeBase());
                break;
        case TIMER_UNITS_MS:
                tl=(tl*_tsys->GetTimeBase())/1000;
                break;
        case TIMER_UNITS_S:
                tl=(tl*_tsys->GetTimeBase())/1000000;
                break;
        }
        if(tl<timeoutTime){
                return RESULT_OK;
        }
        return TIMER_RESULT_TIMEOUT;
}

Result_t CTimer::Delay(
        Timer_Units_t timerUnits,
        uint32_t delay
)
{
        Result_t result=0;

        // Start the timer.
        Start();
        // Wait for the timeout.
        while(result!=TIMER_RESULT_TIMEOUT){
                result=IsTimeout(timerUnits,delay);
                if(!SUCCESSFUL(result)){
                        return result;
                }
        }
        return RESULT_OK;
}

/* EOF */