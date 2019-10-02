/***************************************************************************//**
**
**  @file       mdv_driver.c
**  @ingroup    madivaru-lib
**  @brief      Common driver features.
**  @copyright  Copyright (c) Tuomas Terho. All rights reserved.
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

#include "mdv_driver.h"

/******************************************************************************\
**
**  LOCAL FUNCTION DEFINITIONS
**
\******************************************************************************/

/*-------------------------------------------------------------------------*//**
**  @brief Invokes a driver function.
**
**  @param[in] drv A pointer to a common driver.
**  @param[in] driverFunc The function to be invoked.
**
**  @retval MDV_RESULT_OK Successful.
**  @retval MDV_ERROR_INVALID_POINTER The drv pointer points to NULL.
**  @retval MDV_ERROR_INITIALIZATION_FAILED The driver have not been
**          initialized.
**  @return Any other negative value: A driver specific error code (see the
**          implementation of the driver).
*/
static MdvResult_t
invoke_safely(
        MdvDriver_t *const drv,
        MdvDriverFunction_t driverFunc
)
{
        // Check that the drv pointer is valid (not null).
        if(!drv){
                return MDV_ERROR_INVALID_POINTER;
        }
        // Check that the interface has been initialized.
        if(!drv->initialized){
                return MDV_ERROR_INITIALIZATION_FAILED;
        }
        // Check that the function is in use. If the function is not supported
        // (not set), bypass it silently (the API doesn't need to know that the
        // driver doesn't implement an optional function).
        if(!driverFunc){
                return MDV_RESULT_OK;
        }
        // Pass the driver instance to the  function and return the result.
        return driverFunc(drv->instance);
}

/******************************************************************************\
**
**  API FUNCTION DEFINITIONS
**
\******************************************************************************/

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
)
{
        if(!drv||!instance){
                return MDV_ERROR_INVALID_POINTER;
        }
        drv->instance=instance;
        drv->func.init=funcInit;
        drv->func.open=funcOpen;
        drv->func.close=funcClose;
        drv->func.wakeup=funcWakeup;
        drv->func.sleep=funcSleep;
        drv->func.run=funcRun;
        drv->initialized=true;
        return MDV_RESULT_OK;
}

MdvResult_t
mdv_driver_safe_init(
        MdvDriver_t *const drv
)
{
        return invoke_safely(drv,drv->func.init);
}

MdvResult_t
mdv_driver_safe_open(
        MdvDriver_t *const drv
)
{
        return invoke_safely(drv,drv->func.open);
}

MdvResult_t
mdv_driver_safe_close(
        MdvDriver_t *const drv
)
{
        return invoke_safely(drv,drv->func.close);
}

MdvResult_t
mdv_driver_safe_wakeup(
        MdvDriver_t *const drv
)
{
        return invoke_safely(drv,drv->func.wakeup);
}

MdvResult_t
mdv_driver_safe_sleep(
        MdvDriver_t *const drv
)
{
        return invoke_safely(drv,drv->func.sleep);
}

MdvResult_t
mdv_driver_safe_run(
        MdvDriver_t *const drv
)
{
        return invoke_safely(drv,drv->func.run);
}

/* EOF */