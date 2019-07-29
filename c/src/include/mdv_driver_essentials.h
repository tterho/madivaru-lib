/***************************************************************************//**
**
**  @file       mdv_driver_essentials.h
**  @ingroup    madivaru-lib
**  @brief      Essential driver definitions.
**  @copyright  Copyright (c) Tuomas Terho. All rights reserved.
**
**  Common driver definitions which are essential to any driver design.
**
*******************************************************************************/
/*
**  BSD 3-Clause License
**
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

#ifndef mdv_driver_essentials_H
#define mdv_driver_essentials_H

#include "mdv_types.h"

/******************************************************************************\
**
**  ERROR CODES
**
\******************************************************************************/

/// @brief Invalid pointer.
///
/// At least one of the pointer parameters is invalid (points to null).
#define MDV_DRIVER_ERROR_INVALID_POINTER -1

/// @brief Invalid parameter.
///
/// At least one of the parameter values is invalid (out of range).
#define MDV_DRIVER_ERROR_INVALID_PARAMETER -2

/// @brief Error in driver.
///
/// The driver has not been initialized correctly.
#define MDV_DRIVER_INTERNAL_ERROR -3

/******************************************************************************\
**
**  TYPE DEFINITIONS
**
\******************************************************************************/

/*-------------------------------------------------------------------------*//**
**  @brief Driver instance data pointer.
**
**  This type is used to pass driver implementation specific data to the driver.
*/
typedef void
MdvDriverInstance_t;

/******************************************************************************\
**
**  DRIVER INTERFACE FUNCTION TYPES
**
\******************************************************************************/

/*-------------------------------------------------------------------------*//**
**  @brief Initializes the driver.
**
**  @param[in] instance A pointer to driver instance data.
**
**  @return The result of the initialization.
**
**  Sets the driver instance data to its initial state but doesn't touch to the
**  driver hardware.
*/
typedef
MdvResult_t
(*MdvDriverInterface_Init_t)(
        MdvDriverInstance_t *instance
);

/*-------------------------------------------------------------------------*//**
**  @brief Opens the device for operation.
**
**  @param[in] instance A pointer to driver instance data.
**
**  @retval MDV_RESULT_OK Successful
**  @return On error returns an implementation specific negative error value.
**
**  Prepares the instance data and the hardware for operation.
*/
typedef
MdvResult_t
(*MdvDriverInterface_Open_t)(
        MdvDriverInstance_t *instance
);

/*-------------------------------------------------------------------------*//**
**  @brief Closes the device and returns it to its initial state.
**
**  @param[in] instance A pointer to driver instance data.
**
**  @retval MDV_RESULT_OK Successful
**  @return On error returns an implementation specific negative error value.
*/
typedef
MdvResult_t
(*MdvDriverInterface_Close_t)(
        MdvDriverInstance_t *instance
);

/*-------------------------------------------------------------------------*//**
**  @brief Prepares the driver to sleep mode.
**
**  @param[in] instance A pointer to driver instance data.
**
**  @retval MDV_RESULT_OK Successful.
**  @return On error returns an implementation specific negative error value.
**
**  Sets the hardware to its lowest power consuming state.
*/
typedef
MdvResult_t
(*MdvDriverInterface_Sleep_t)(
        MdvDriverInstance_t *instance
);

/*-------------------------------------------------------------------------*//**
**  @brief Performs post-sleep operations.
**
**  @param[in] instance A pointer to driver instance data.
**
**  @retval MDV_RESULT_OK Successful.
**  @return On error returns an implementation specific negative error value.
**
**  Sets the hardware to the operational state after a reset or sleep.
*/
typedef
MdvResult_t
(*MdvDriverInterface_Wakeup_t)(
        MdvDriverInstance_t *instance
);

/******************************************************************************\
**
**  DRIVER INTERFACE DEFINITION
**
\******************************************************************************/

/*-------------------------------------------------------------------------*//**
**  @brief Essential driver interface
*/
typedef struct
MdvDriverEssentials_t{
        /// @brief Driver instance data pointer.
        MdvDriverInstance_t *instance;
        /// @brief Initializes the driver.
        MdvDriverInterface_Init_t funcInit;
        /// @brief Opens the device.
        MdvDriverInterface_Open_t funcOpen;
        /// @brief Closes the device.
        MdvDriverInterface_Close_t funcClose;
        /// @brief Prepares the driver to sleep.
        MdvDriverInterface_Sleep_t funcSleep;
        /// @brief Performs post-sleep operations.
        MdvDriverInterface_Wakeup_t funcWakeup;
        /// @brief Interface initialization status.
        bool initialized;
} MdvDriverEssentials_t;

#endif // ifndef mdv_driver_essentials_H

/* EOF */
