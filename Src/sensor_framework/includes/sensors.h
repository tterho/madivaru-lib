/***************************************************************************//**
**
**  @defgroup   sensor_framework Sensor framework
**
**  A general purpose sensor framework for applications utilizing any kind of
**  sensor equipment.
**
**  @file       sensors.h
**  @ingroup    sensor_framework
**  @brief      List of sensors supported by the application.
**  @copyright  Copyright (C) 2012-2018 Tuomas Terho. All rights reserved.
**
**  A user-configurable list of sensors supported by the target application.
**
*******************************************************************************/
/*
**  BSD 3-Clause License
**
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

#ifndef sensors_H
#define sensors_H

/******************************************************************************\
**
**  SENSOR ENUMERATION
**
\******************************************************************************/

/// Use this enumeration to specify the list of sensors supported by this 
/// application.
typedef enum
Sensor_t{
        /// Board temperature sensor (an example sensor).
        SENSOR_BTS=0, 
        /// Input supply voltage sensor (an example sensor).
        SENSOR_VCC, 
        /// The number of sensors supported by this system.
        /// @warning Do not modify/delete this item! It must always appear at 
        /// the end of this enumeration.
        SENSOR_COUNT
} Sensor_t;

#endif // sensors_H

/* EOF */