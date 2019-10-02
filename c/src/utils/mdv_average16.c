/**************************************************************************-----
**
**  @file       mdv_average16.c
**  @ingroup    madivaru-lib
**  @brief      Calculates an average value from a 16-bit sample stream.
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

#include "mdv_average16.h"

/******************************************************************************\
**
**  API FUNCTION DEFINITIONS
**
\******************************************************************************/

/*------------------------------------------------------------------------------
**  Initializes an average calculator.
*/
MdvResult_t
mdv_average16_init(
        MdvAverage16_t *const average,
        MdvAverage16FilterMode_t mode,
        uint16_t size,
        uint16_t *const buffer
)
{
        if(!average||!buffer){
                return MDV_ERROR_INVALID_POINTER;
        }        
        average->mod=mode;
        average->sz=size;
        average->bfr=buffer;
        mdv_average16_reset(average);
        return MDV_RESULT_OK;
}

/*------------------------------------------------------------------------------
**  Resets an average calculator.
*/
MdvResult_t
mdv_average16_reset(
        MdvAverage16_t *const average
)
{
        if(!average){
                return MDV_ERROR_INVALID_POINTER;
        }
        average->wr=0;
        average->cnt=0;
        average->sum=0;
        average->val=0;
        return MDV_RESULT_OK;
}

/*------------------------------------------------------------------------------
**  Puts a sample to an average calculator and returns the current average 
**  value. If there is not enough samples for average calculation, returns 0.
*/
uint16_t
mdv_average16_put(
        MdvAverage16_t *const average,
        uint16_t sample
)
{
        uint16_t fs; // The first sample.

        switch(average->mod){
        case MDV_AVERAGE16_FILTER_MODE_FLOATING:
                // The first sample in the buffer will be overwritten
                // (if buffer is full of samples), so keep it temporarily in
                // memory for further use.
                fs=average->bfr[average->wr];
                // Write new sample to the buffer.
                average->bfr[average->wr]=sample;
                // Advance the buffer pointer and wrap around if end of buffer.
                ++average->wr;
                if(average->wr==average->sz){
                        average->wr=0;
                }
                // Increase sample count if smaller than the buffer size.
                if(average->cnt<average->sz){
                        ++average->cnt;
                }
                // Add the sample to the sum of the buffered samples.
                average->sum+=sample;
                // Drop the first sample away from the sum if needed.
                if(average->cnt==average->sz){
                        average->sum-=fs;
                }
                break;
        default:
        case MDV_AVERAGE16_FILTER_MODE_CONTIGUOUS:
                // Wrap-around check (stops calculation before an overflow).
                if(average->sum+sample<average->sum){
                        return average->val;
                }
                average->sum+=sample;
                ++average->cnt;
                break;
        }
        // Calculate average.
        average->val=average->sum/average->cnt;
        return average->val;
}

/* EOF */