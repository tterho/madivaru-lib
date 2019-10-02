/***************************************************************************//**
**
**  @file       mdv_xtc.c
**  @ingroup    madivaru-lib
**  @brief      An API for NTC and PTC temperature sensors.
**  @copyright  Copyright (c) Tuomas Terho
**
*******************************************************************************/
/*
**  COPYRIGHT (c) Tuomas Terho
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
*******************************************************************************/

#include "mdv_xtc.h"

/******************************************************************************\
**
**  LOCAL FUNCTION DEFINITIONS
**
\******************************************************************************/

/*-------------------------------------------------------------------------*//**
**  @brief Calculates linerized temperature T from resistance R.
**
**  @param[in] xtc Pointer to an NTC or a PTC.
**  @param[in] i The index of the nearest characteristic curve table element.
**  @param[in] R Resistance in ohms.
**
**  @return Temperature in °C.
*/
static float
xtc_get_temperature_by_resistance(
        MdvXtc_t *xtc,
        uint16_t i,
        float R
)
{
        float dRc;
        float dTc;
        float dR;

        // Calculate the difference between two consecutive curve values for
        // both resistance and temperature.
        dRc=xtc->crv[i].R-xtc->crv[i-1].R;
        dTc=xtc->crv[i].T-xtc->crv[i-1].T;
        // Calculate the difference between the resistance value and the nearest
        // lower curve value.
        dR=xtc->crv[i].R-R;
        // Calculate a linearization for the current temperature value.
        return xtc->crv[i].T-dR*(dTc/dRc);
}

/*-------------------------------------------------------------------------*//**
**  @brief Converts NTC resistance to the corresponding temperature value.
**
**  @param[in] ntc Pointer to an NTC.
**  @param[in] R Resistance in ohms.
**
**  @return Temperature in °C.
*/
static float
xtc_ntc_resistance_to_temperature(
        MdvXtc_t *ntc,
        float R
)
{
        uint16_t i;

        // Check the lowest temperature limit.
        if(R>ntc->crv[0].R){
                return ntc->crv[0].T;
        }
        // Check the highest temperature limit.
        if(R<ntc->crv[ntc->crvsz-1].R){
                return ntc->crv[ntc->crvsz-1].T;
        }
        // Get matching or nearest resistance from the curve.
        for(i=0;i<ntc->crvsz;i++){
                // Find a possible exact match.
                if(R==ntc->crv[i].R){
                        // Return the corresponding temperature.
                        return ntc->crv[i].T;
                }
                // No exact matches. Get the nearest lower value.
                if(R>ntc->crv[i].R){
                        break;
                }
        }
        return xtc_get_temperature_by_resistance(ntc,i,R);
}

/*-------------------------------------------------------------------------*//**
**  @brief Converts PTC resistance to the corresponding temperature value.
**
**  @param[in] xtc Pointer to a PTC.
**  @param[in] R Resistance in ohms.
**
**  @return Temperature in °C.
*/
static float
xtc_ptc_resistance_to_temperature(
        MdvXtc_t *ptc,
        float R
)
{
        uint16_t i;

        // Check the lowest temperature limit.
        if(R<ptc->crv[0].R){
                return ptc->crv[0].T;
        }
        // Check the highest temperature limit.
        if(R>ptc->crv[ptc->crvsz-1].R){
                return ptc->crv[ptc->crvsz-1].T;
        }
        // Get matching or nearest resistance from the curve.
        for(i=0;i<ptc->crvsz;i++){
                // Find a possible exact match.
                if(R==ptc->crv[i].R){
                        // Return the corresponding temperature.
                        return ptc->crv[i].T;
                }
                // No exact matches. Get the nearest higher value.
                if(R<ptc->crv[i].R){
                        break;
                }
        }
        return xtc_get_temperature_by_resistance(ptc,i,R);
}

/******************************************************************************\
**
**  API FUNCTION DEFINITIONS
**
\******************************************************************************/

float
mdv_xtc_get_temperature(
        MdvXtc_t *const xtc,
        float R
)
{
        switch(xtc->tp){
        case MDV_XTC_TYPE_NTC:return xtc_ntc_resistance_to_temperature(xtc,R);
        case MDV_XTC_TYPE_PTC:return xtc_ptc_resistance_to_temperature(xtc,R);
        default:return 0.0;
        }
}

float
mdv_xtc_get_resistance(
        MdvXtc_t *const xtc,
        float T
)
{
        uint16_t i;
        float dRc;
        float dTc;
        float dT;

        // Check the lowest temperature limit.
        if(T<xtc->crv[0].T){
                return xtc->crv[0].R;
        }
        // Check the highest temperature limit.
        if(T>xtc->crv[xtc->crvsz-1].T){
                return xtc->crv[xtc->crvsz-1].R;
        }
        // Get temperature from the curve.
        for(i=0;i<xtc->crvsz;i++){
                // Find a possible exact match.
                if(T==xtc->crv[i].T){
                        return xtc->crv[i].R;
                }
                // No exact matches. Get the nearest higher value.
                if(T<xtc->crv[i].T){
                        break;
                }
        }
        // Calculate the difference between two consecutive curve values for
        // both resistance and temperature.
        dRc=xtc->crv[i].R-xtc->crv[i-1].R;
        dTc=xtc->crv[i].T-xtc->crv[i-1].T;
        // Calculate the difference between the temperature and the nearest
        // higher curve value.
        dT=xtc->crv[i].T-T;
        // Calculate a linearization for the current resistance value.
        return xtc->crv[i].R-dT*(dRc/dTc);
}

/* EOF */