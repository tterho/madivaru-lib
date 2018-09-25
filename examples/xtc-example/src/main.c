/***************************************************************************//**
**
**  @defgroup   xtc-example XTC example application
**
**  A Windows Console application for Visual Studio Community. This application
**  demonstrates how to create an NTC or a PTC temperature sensor by using the 
**  madivaru-lib XTC API.
**
**  @file       main.c
**  @ingroup    xtc-example
**  @brief      The application main.
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

#include <stdio.h>
#include "xtc_api.h"

/// @brief The NTC / PTC curve size used in this example.
#define XTC_CURVE_SIZE 7

/*-------------------------------------------------------------------------*//**
**  @brief A PTC curve.
**
**  This is an example curve for a PTC (positive temperature coefficient) 
**  resistor. Normally the curve is defined by using the values given in the PTC
**  datasheet.
*/
static const XTC_Curve_t 
PTC_Curve[XTC_CURVE_SIZE]={
        {-10,100},
        {-5,200},
        {0,300},
        {5,400},
        {10,500},
        {15,600},
        {20,700}
};

/*-------------------------------------------------------------------------*//**
**  @brief A PTC declaration.
*/
static const XTC_t 
PTC={
        XTC_TYPE_PTC,
        XTC_CURVE_SIZE,
        PTC_Curve
};

/*-------------------------------------------------------------------------*//**
**  @brief An NTC curve.
**
**  This is an example curve for an NTC (negative temperature coefficient)
**  resistor. Normally the curve is defined by using the values given in the NTC
**  datasheet.
*/
static const XTC_Curve_t 
NTC_Curve[XTC_CURVE_SIZE]={
        {-10,700},
        {-5,600},
        {0,500},
        {5,400},
        {10,300},
        {15,200},
        {20,100}
};

/*-------------------------------------------------------------------------*//**
**  @brief An NTC declaration.
*/
static const XTC_t 
NTC={
        XTC_TYPE_NTC,
        XTC_CURVE_SIZE,
        NTC_Curve
};

int main(
        void
)
{
        float t=-5;
        float r=500;

        // In this loop we print 200 consecutive values of temperature and 
        // resistance and convert them to NTC and PTC resistances and 
        // temperatures, accordingly.
        for(int i=0;i<200;i++){
                printf("T=%f, NTC R=%f, PTC R=%f, R=%f, NTC T=%f, PTC T=%f\n",
                       t,
                       XTC_Convert_T_to_R(&NTC,t),
                       XTC_Convert_T_to_R(&PTC,t),
                       r,
                       XTC_Convert_R_to_T(&NTC,r),
                       XTC_Convert_R_to_T(&PTC,r)
                );
                t+=0.01;
                r+=1;
        }

        while(1);
}