/***************************************************************************//**
**
**  @file       mdv_fifo.h
**  @ingroup    madivaru-lib
**  @brief      A general purpose first-in/first-out buffer (FIFO).
**  @copyright  Copyright (C) 2012-2018 Tuomas Terho. All rights reserved.
**
**  The maximum capacity of this FIFO is 65535 data items, 65535 bytes each.
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

#ifndef fifo_H
#define fifo_H

#include "mdv_types.h"

/******************************************************************************\
**
**  PROTECTION MACRO DECLARATIONS
**
\******************************************************************************/

/// @brief Enters into a critical section of code.
///
/// This macro is used in the Put and Get operations to prevent multiple 
/// simultaneous accesses to the FIFO. Add code here according to your system
/// needs, for example by disabling interrupts or by retrieving a mutex.
#ifndef MDV_FIFO_ENTER_CRITICAL
        #define MDV_FIFO_ENTER_CRITICAL()
#endif // ifndef MDV_FIFO_ENTER_CRITICAL

/// @brief Exits a critical section of code.
///
/// This macro is used in the Put and Get operations to prevent multiple
/// simultaneous accesses to the FIFO. Add code here according to your system
/// needs, for example by enabling interrupts or by freeing a mutex.
#ifndef MDV_FIFO_EXIT_CRITICAL
        #define MDV_FIFO_EXIT_CRITICAL()
#endif // ifndef MDV_FIFO_EXIT_CRITICAL

/******************************************************************************\
**
**  ERROR CODES
**
\******************************************************************************/

/// @brief Invalid pointer.
///
/// At least one of the pointer parameters is invalid (points to null).
#define MDV_FIFO_ERROR_INVALID_POINTER -1

/// @brief Invalid parameter.
///
/// At least one of the value parameters is invalid.
#define MDV_FIFO_ERROR_INVALID_PARAMETER -2

/// @brief FIFO is full.
///
/// The FIFO is full. Performing a Put operation loses data.
#define MDV_FIFO_ERROR_FULL -3

/// @brief FIFO is empty.
///
/// The FIFO is empty. Performing a Get operation doesn't return data.
#define MDV_FIFO_ERROR_EMPTY -4

/******************************************************************************\
**
**  TYPE DEFINITIONS
**
\******************************************************************************/

/*-------------------------------------------------------------------------*//** 
**  @brief FIFO structure
*/
typedef struct 
MdvFifo_t
{
        /// Pointer to the FIFO data buffer.
        void *d;
        /// Data item size in bytes.
        uint16_t dsz;
        /// Buffer size in bytes.
        uint32_t sz;
        /// Maximum items in the FIFO.
        uint16_t max;
        /// Current amount of data items in the FIFO.
        uint16_t cnt;
        /// Write index.
        uint16_t wr;
        /// Read index.
        uint16_t rd;
        /// Write pointer.
        void *wrp;
        /// Read pointer.
        void *rdp;
} MdvFifo_t;

/******************************************************************************\
**
**  API FUNCTION DECLARATIONS
**
\******************************************************************************/

/*-------------------------------------------------------------------------*//** 
**  @brief Initializes a FIFO.
**
**  @param[in,out] fifo A FIFO being initialized.
**  @param[in] data A pointer to a buffer for the FIFO data.
**  @param[in] size The size of the buffer in bytes.
**  @param[in] dsize The size of one data item in bytes.
**
**  @retval MDV_RESULT_OK Successful
**  @retval MDV_FIFO_ERROR_INVALID_POINTER Either the fifo or data parameter points 
**      to null.
**  @retval MDV_FIFO_ERROR_INVALID_PARAMETER The size or the data size is zero, or
**      the size is smaller than the data size.
**
**  Initializes a FIFO by associating it with an externally allocated data
**  buffer and resets the FIFO.
*/
MdvResult_t 
mdv_fifo_init(
        MdvFifo_t *fifo,
        void *data,        
        uint32_t size,
        uint16_t dsize
);

/*-------------------------------------------------------------------------*//** 
**  @brief Resets a FIFO.
**
**  @param[in] fifo FIFO to reset.
**
**  @retval MDV_RESULT_OK Successful
**  @retval MDV_FIFO_ERROR_INVALID_POINTER fifo parameter points to null.
**
**  Resets a FIFO by setting the read and write pointers to the beginning of the
**  buffer and setting the data count to zero.
*/
MdvResult_t
mdv_fifo_reset(
        MdvFifo_t *fifo
);

/*-------------------------------------------------------------------------*//** 
**  @brief Puts data to the FIFO.
**
**  @param[in] fifo FIFO to put data in.
**  @param[in] data Item to put to the FIFO.
**
**  @retval MDV_RESULT_OK Successful
**  @retval MDV_FIFO_ERROR_INVALID_POINTER Either the fifo or the data parameter 
**      points to null.
**  @retval MDV_FIFO_ERROR_FULL FIFO is full.
**
**  Puts data to a FIFO, advances its write pointer and increases data count.
**  The size of the data item must match with the size given in the 
**  FIFO initialization.
*/
MdvResult_t
mdv_fifo_put(
        MdvFifo_t *fifo,
        void *data
);

/*-------------------------------------------------------------------------*//** 
**  @brief Gets a byte from the FIFO
**
**  @param[in] fifo FIFO to get data from.
**  @param[in] data Pointer to a variable where to put data.
**
**  @retval MDV_RESULT_OK Successful
**  @retval MDV_FIFO_ERROR_INVALID_POINTER Either the fifo or the data parameter 
**      points to null.
**  @return MDV_FIFO_ERROR_EMPTY FIFO is empty.
**
**  Gets data from a FIFO, advances its read pointer and decreases data count.
*/
MdvResult_t
mdv_fifo_get(
        MdvFifo_t *fifo,
        void *data
);

#endif // ifndef mdv_fifo_H

/* EOF */