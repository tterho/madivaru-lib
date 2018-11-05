/***************************************************************************//**
**
**  @file       xtc_api.h
**  @ingroup    utils
**  @brief      An API for NTC and PTC temperature sensors.
**  @copyright  Copyright (C) 2012-2018 Tuomas Terho
**
*******************************************************************************/
/*
**  COPYRIGHT (c) 2012-2018, Tuomas Terho
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

#ifndef xtc_api_H
#define xtc_api_H

#include "types.h"

/******************************************************************************\
**
**  MACROS AND TYPES
**
\******************************************************************************/

/*-------------------------------------------------------------------------*//**
**  @brief Type of the sensor.
*/
typedef enum
XTC_Type_t{
    /// Sensor type not set.
    XTC_TYPE_UNKNOWN=0,
    /// Negative temperature coefficient (NTC) type.
    XTC_TYPE_NTC,
    /// Positive temperature coefficient (PTC) type.
    XTC_TYPE_PTC
} XTC_Type_t;

/*-------------------------------------------------------------------------*//**
**  @brief NTC/PTC characteristic curve table element
**
**  The characteristic curve table contains temperatures in dT steps with their
**  corresponding resistance values. The dT is NTC or PTC specific. Temperatures
**  are floating point values in °C.
*/
typedef struct
XTC_Curve_t{
        /// Temperature in °C.
        float T;
        /// Resistance in ohms.
        float R;
} XTC_Curve_t;

/*-------------------------------------------------------------------------*//**
**  @brief NTC/PTC characteristic curve table
*/
typedef struct
XTC_t{
        /// The type of the sensor.
        XTC_Type_t tp;
        /// The size of the table.
        uint16_t crvsz;
        /// A pointer to the curve elements.
        const XTC_Curve_t *crv;
} XTC_t;

/******************************************************************************\
**
**  API FUNCTIONS
**
\******************************************************************************/

/*-------------------------------------------------------------------------*//**
**  @brief Converts resistance to the corresponding temperature value.
**
**  @param[in] xtc Pointer to an NTC or a PTC.
**  @param[in] R Resistance in ohms.
**
**  @return Temperature in °C.
*/
float
XTC_Convert_R_to_T(
        XTC_t *xtc,
        float R
);

/*-------------------------------------------------------------------------*//**
**  @brief Converts temperature to the corresponding resistance value.
**
**  @param[in] xtc Pointer to an NTC or a PTC
**  @param[in] T Temperature in °C.
**
**  @return Resistance in ohms.
*/
float
XTC_Convert_T_to_R(
        XTC_t *xtc,
        float T
);

#endif // xtc_api_H

/* EOF */