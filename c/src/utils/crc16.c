/***************************************************************************//**
**
**  @file       crc16.c
**  @ingroup    utils
**  @brief      CRC-16-IBM checksum calculator
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

#include "crc16.h"

/******************************************************************************\
**
**  MACRO DEFINITIONS
**
\******************************************************************************/

/*-------------------------------------------------------------------------*//**
**  @brief Polynomial representation for CRC-16-IBM (reversed calculation). 
*/
#define CRC16_POLYNOMIAL 0xA001

/******************************************************************************\
**
**  PUBLIC FUNCTION DECLARATIONS
**
\******************************************************************************/

/*------------------------------------------------------------------------------
**  Calculates a CRC-16 checksum for the input data. 
*/
uint16_t
CRC16(
        uint16_t crc,
        uint32_t size,
        const uint8_t *data
)
{
        // Calculate CRC for each byte.
        while(size--){
                crc=CRC16_Byte(crc,*(data++));
        }
        return crc;
}

/*------------------------------------------------------------------------------
**  Calculates a CRC-16 checksum for one byte. 
*/
uint16_t 
CRC16_Byte(
        uint16_t crc,
        uint8_t data
)
{
        uint8_t bit;
        
        // Generate CRC for data bits.
        crc=crc^((uint16_t)data&0x00ff);
        for(bit=0;bit<8;bit++){
                if(crc&1){
                        crc=(crc>>1)^CRC16_POLYNOMIAL;
                }else{
                        crc=(crc>>1);
                }
        }
        return crc;
}

/* EOF */