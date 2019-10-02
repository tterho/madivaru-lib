/***************************************************************************//**
**
**  @file       mdv_average16.h
**  @ingroup    madivaru-lib
**  @brief      Calculates an average value from 16-bit sample stream.
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

#ifndef mdv_average16_H
#define mdv_average16_H

#include "mdv_types.h"

/******************************************************************************\
**
**  TYPE DEFINITIONS
**
\******************************************************************************/

/*-------------------------------------------------------------------------*//**
**  @brief Average filter mode
*/
typedef enum
MdvAverage16FilterMode_t{
        /// @brief Contiguous filter mode.
        ///
        /// In contiguous mode all samples are summed and divided by the total
        /// sample count.This mode saves memory but have one limitation.The
        /// summary value is 32-bit,so the calculation will be stopped for long
        /// sample streams to prevent an overflow.
        MDV_AVERAGE16_FILTER_MODE_CONTIGUOUS=0,
        /// @brief Floating filter mode.
        ///
        /// In floating mode the sample count is unlimited.Samples are stored in
        /// a FIFO-type buffer. Only samples in the buffer are included into the
        /// average.
        MDV_AVERAGE16_FILTER_MODE_FLOATING
} MdvAverage16FilterMode_t;

/*-------------------------------------------------------------------------*//**
**  @brief Average calculator structure
*/
typedef struct
MdvAverage16_t
{
        /// @brief Average calculation mode.
        MdvAverage16FilterMode_t mod;
        /// @brief Pointer to a sample buffer (floating mode only).
        uint16_t *bfr;
        /// @brief Write index (floating mode only).
        uint16_t wr;
        /// @brief Buffer size in 16-bit words (floating mode only).
        uint16_t sz;
        /// @brief Count of samples participating.
        uint16_t cnt;
        /// @brief Sum of samples.
        uint32_t sum;
        /// @brief Current average value.
        uint16_t val;
} MdvAverage16_t;

/******************************************************************************\
**
**  API FUNCTION DECLARATIONS
**
\******************************************************************************/

#ifdef __cplusplus
extern "C"{
#endif // ifdef __cplusplus

/*-------------------------------------------------------------------------*//**
**  @brief Initializes an average calculator.
**
**  @param[in] average A pointer to an average structure being initialized.
**  @param[in] mode Average calculation mode.
**  @param[in] size Size of the sample buffer in floating mode. For contiguous
**             mode set this parameter to zero.
**  @param[in] buffer Buffer to store samples in floating mode. For contiguous
**             mode set this parameter to null.
**
**  @retval MDV_RESULT_OK Initialization successful.
**  @retval MDV_ERROR_INVALID_POINTER Either the average or the buffer pointer
**          points to null.
*/
MdvResult_t
mdv_average16_init(
        MdvAverage16_t *const average,
        MdvAverage16FilterMode_t mode,
        uint16_t size,
        uint16_t *const buffer
);

/*-------------------------------------------------------------------------*//**
**  @brief Resets an average calculator.
**
**  @param[in] average A pointer to an average structure being reset.
**
**  @retval MDV_RESULT_OK Reset successful.
**  @retval MDV_ERROR_INVALID_POINTER The average pointer points to null.
*/
MdvResult_t
mdv_average16_reset(
        MdvAverage16_t *const average
);

/*-------------------------------------------------------------------------*//**
**  @brief Puts a sample to an average calculator and returns the current
**         average value. If there is not enough samples for average
**         calculation, returns 0.
**
**  @param[in] average A pointer to an average structure where to put a sample.
**  @param[in] sample The sample value to put.
**
**  @return The current average value.
*/
uint16_t
mdv_average16_put(
        MdvAverage16_t *const average,
        uint16_t sample
);

#ifdef __cplusplus
}
#endif // ifdef __cplusplus

#endif // ifndef mdv_average16_H

/* EOF */