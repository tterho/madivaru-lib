/***************************************************************************//**
**
**  @file       mdv_crc32.h
**  @ingroup    madivaru-lib
**  @brief      CRC-32 checksum calculator
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

#ifndef mdv_crc32_H
#define mdv_crc32_H

#include "mdv_types.h"

/******************************************************************************\
**
**  API FUNCTION DECLARATIONS
**
\******************************************************************************/

#ifdef __cplusplus
extern "C"{
#endif // ifdef __cplusplus

/*-------------------------------------------------------------------------*//**
**  @brief Calculates a CRC-32 checksum for the input data.
**
**  @param[in] crc CRC from a previous calculation. Set to zero for initial
**             calculation.
**  @param[in] size Data size in bytes.
**  @param[in] data Pointer to a buffer containing input data.
**
**  @return CRC-32 checksum
**
**  This function can be used for both single and continued CRC calculation.
**  In the initial calculation the crc parameter must be zero. For continued
**  calculation the crc value is the result of the previous calculation.
*/
uint32_t
mdv_crc32(
        uint32_t crc,
        uint32_t size,
        const uint8_t *const data
);

#ifdef __cplusplus
}
#endif // ifdef __cplusplus

#endif // ifndef mdv_crc32_H

/* EOF */