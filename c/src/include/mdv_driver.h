/***************************************************************************//**
**
**  @file       mdv_driver.h
**  @ingroup    madivaru-lib
**  @brief      Common driver features.
**  @copyright  Copyright (c) Tuomas Terho. All rights reserved.
**
**  Driver common API to help designing a device driver.
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

#ifndef mdv_driver_H
#define mdv_driver_H

#include "mdv_types.h"

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
**  @brief Generic function type for the driver functions.
**
**  @param[in] instance A pointer to driver instance data.
**
**  @return The result of the operation.
*/
typedef MdvResult_t
(*MdvDriverFunction_t)(
        MdvDriverInstance_t *const instance
);

/******************************************************************************\
**
**  DRIVER INTERFACE DEFINITION
**
\******************************************************************************/

/*-------------------------------------------------------------------------*//**
**  @brief Common driver.
*/
typedef struct
MdvDriver_t{
        /// @brief Driver instance data pointer.
        MdvDriverInstance_t *instance;
        /// @brief Function interface.
        struct{
                /// @brief Initializes the driver.
                MdvDriverFunction_t init;
                /// @brief Opens the device.
                MdvDriverFunction_t open;
                /// @brief Closes the device.
                MdvDriverFunction_t close;
                /// @brief Performs post-sleep operations.
                MdvDriverFunction_t wakeup;
                /// @brief Prepares the driver to sleep.
                MdvDriverFunction_t sleep;
                /// @brief Runs the driver.
                MdvDriverFunction_t run;
        } func;
        /// @brief Interface initialization status.
        bool initialized;
} MdvDriver_t;

/******************************************************************************\
**
**  API FUNCTION DECLARATIONS
**
\******************************************************************************/

#ifdef __cplusplus
extern "C"{
#endif // ifdef __cplusplus

/*-------------------------------------------------------------------------*//**
**  @brief Sets up the driver.
**
**  @param[out] drv A pointer to a common driver to set up.
**  @param[in] instance A pointer to a driver instance.
**  @param[in] funcInit (Optional) A function to initialize the driver.
**  @param[in] funcOpen (Optional) A function to open the port.
**  @param[in] funcClose (Optional) A function to close the port.
**  @param[in] funcWakeup (Optional) A function to perform post-sleep operations.
**  @param[in] funcSleep (Optional) A function to prepare for sleep mode.
**  @param[in] funcRun (Optional) A function to run post-sleep operations.
**
**  @retval MDV_RESULT_OK Successful.
**  @retval MDV_ERROR_INVALID_POINTER The drv or the instance pointer
**          points to NULL.
*/
MdvResult_t
mdv_driver_setup(
        MdvDriver_t *const drv,
        MdvDriverInstance_t *const instance,
        MdvDriverFunction_t funcInit,
        MdvDriverFunction_t funcOpen,
        MdvDriverFunction_t funcClose,
        MdvDriverFunction_t funcWakeup,
        MdvDriverFunction_t funcSleep,
        MdvDriverFunction_t funcRun
);

/*-------------------------------------------------------------------------*//**
**  @brief Calls the driver Init function safely.
**
**  @param[in] drv A pointer to a common driver.
**
**  @retval MDV_RESULT_OK Successful.
**  @retval MDV_ERROR_INVALID_POINTER The drv pointer points to NULL.
**  @retval MDV_ERROR_INITIALIZATION_FAILED The driver have not been
**          initialized.
**  @return Any other negative value: A driver specific error code (see the
**          implementation of the driver).
*/
MdvResult_t
mdv_driver_safe_init(
        MdvDriver_t *const drv
);

/*-------------------------------------------------------------------------*//**
**  @brief Calls the driver Open function safely.
**
**  @param[in] drv A pointer to a common driver.
**
**  @retval MDV_RESULT_OK Successful.
**  @retval MDV_ERROR_INVALID_POINTER The drv pointer points to NULL.
**  @retval MDV_ERROR_INITIALIZATION_FAILED The driver have not been
**          initialized.
**  @return Any other negative value: A driver specific error code (see the
**          implementation of the driver).
*/
MdvResult_t
mdv_driver_safe_open(
        MdvDriver_t *const drv
);

/*-------------------------------------------------------------------------*//**
**  @brief Calls the driver Close function safely.
**
**  @param[in] drv A pointer to a common driver.
**
**  @retval MDV_RESULT_OK Successful.
**  @retval MDV_ERROR_INVALID_POINTER The drv pointer points to NULL.
**  @retval MDV_ERROR_INITIALIZATION_FAILED The driver have not been
**          initialized.
**  @return Any other negative value: A driver specific error code (see the
**          implementation of the driver).
*/
MdvResult_t
mdv_driver_safe_close(
        MdvDriver_t *const drv
);

/*-------------------------------------------------------------------------*//**
**  @brief Calls the driver Wakeup function safely.
**
**  @param[in] drv A pointer to a common driver.
**
**  @retval MDV_RESULT_OK Successful.
**  @retval MDV_ERROR_INVALID_POINTER The drv pointer points to NULL.
**  @retval MDV_ERROR_INITIALIZATION_FAILED The driver have not been
**          initialized.
**  @return Any other negative value: A driver specific error code (see the
**          implementation of the driver).
*/
MdvResult_t
mdv_driver_safe_wakeup(
        MdvDriver_t *const drv
);

/*-------------------------------------------------------------------------*//**
**  @brief Calls the driver Sleep function safely.
**
**  @param[in] drv A pointer to a common driver.
**
**  @retval MDV_RESULT_OK Successful.
**  @retval MDV_ERROR_INVALID_POINTER The drv pointer points to NULL.
**  @retval MDV_ERROR_INITIALIZATION_FAILED The driver have not been
**          initialized.
**  @return Any other negative value: A driver specific error code (see the
**          implementation of the driver).
*/
MdvResult_t
mdv_driver_safe_sleep(
        MdvDriver_t *const drv
);

/*-------------------------------------------------------------------------*//**
**  @brief Calls the driver Run function safely.
**
**  @param[in] drv A pointer to a common driver.
**
**  @retval MDV_RESULT_OK Successful.
**  @retval MDV_ERROR_INVALID_POINTER The drv pointer points to NULL.
**  @retval MDV_ERROR_INITIALIZATION_FAILED The driver have not been
**          initialized.
**  @return Any other negative value: A driver specific error code (see the
**          implementation of the driver).
*/
MdvResult_t
mdv_driver_safe_run(
        MdvDriver_t *const drv
);

#ifdef __cplusplus
}
#endif // ifdef __cplusplus

#endif // ifndef mdv_driver_H

/* EOF */
