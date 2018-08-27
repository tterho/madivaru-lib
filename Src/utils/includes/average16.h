/***************************************************************************//**
**
**  @file       average16.h
**  @ingroup    utils
**  @brief      Calculates an average value from 16-bit sample stream.
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

#ifndef average16_H
#define average16_H

#include "types.h"

/******************************************************************************\
**
**  ERROR CODES
**
\******************************************************************************/

#define AVERAGE_ERROR_INVALID_POINTER -1

/******************************************************************************\
**
**  PUBLIC MACRO AND TYPE DEFINITIONS
**
\******************************************************************************/

/*-------------------------------------------------------------------------*//**
**  @brief Average filter mode
**
**  In contiguous mode all samples are summed and divided by the total sample
**  count. This mode saves memory but have one limitation. The summary value is 
**  32-bit, so the calculation will be stopped for long sample streams to 
**  prevent an overflow.
**
**  In floating mode the sample count is unlimited. Samples are stored in a
**  FIFO-type buffer. Only samples in the buffer are included into the average.
*/
typedef enum
Average16_Mode_t{
        AVERAGE16_MODE_CONTIGUOUS,
        AVERAGE16_MODE_FLOATING
} Average16_Mode_t;

/*-------------------------------------------------------------------------*//** 
**  @brief Average structure
*/
typedef struct 
Average16_t
{
        /// Average calculation mode.
        Average16_Mode_t mod;
        /// Pointer to a sample buffer (floating mode only).
        uint16_t *bfr;
        /// Write index (floating mode only).
        uint16_t wr;
        /// Buffer size in 16-bit words (floating mode only).
        uint16_t sz;
        /// Count of samples participating.
        uint16_t cnt;
        /// Sum of samples.
        uint32_t sum;
        /// Current average value.
        uint16_t val;
} Average16_t;

/******************************************************************************\
**
**  PUBLIC FUNCTION DEFINITIONS
**
\******************************************************************************/

/*-------------------------------------------------------------------------*//** 
**  @brief Initializes an average calculator.
**
**  @param[in] average A pointer to an average structure being initialized.
**  @param[in] mode Average calculation mode.
**  @param[in] size Size of the sample buffer in floating mode. For contiguous 
**      mode set this parameter to zero.
**  @param[in] buffer Buffer to store samples in floating mode. For contiguous 
**      mode set this parameter to null.
**
**  @retval RESULT_OK Initialization successful.
**  @retval AVERAGE_ERROR_INVALID_POINTER Either the average or the buffer 
**      pointer points to null.
*/
Result_t
Average16_Init(
        Average16_t *average,
        Average16_Mode_t mode,
        uint16_t size,
        uint16_t *buffer
);

/*-------------------------------------------------------------------------*//** 
**  @brief Resets an average calculator.
**
**  @param[in] average A pointer to an average structure being reset.
**
**  @retval RESULT_OK Reset successful.
**  @retval AVERAGE_ERROR_INVALID_POINTER The average pointer points to null.
*/
Result_t
Average16_Reset(
        Average16_t *average
);

/*-------------------------------------------------------------------------*//** 
**  @brief Puts a sample to an average calculator and returns the current 
**  average value. If there is not enough samples for average calculation, 
**  returns 0.
**
**  @param[in] average A pointer to an average structure where to put a sample.
**
**  @return The current average value.
*/
uint16_t
Average16_Put(
        Average16_t *average,
        uint16_t sample
);

#endif // average16_H

/* EOF */