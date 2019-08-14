/***************************************************************************//**
**
**  @file       mdv_driver_essentials.c
**  @ingroup    madivaru-lib
**  @brief      Essential driver features.
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

#include "mdv_driver_essentials.h"

/******************************************************************************\
**
**  API FUNCTION DEFINITIONS
**
\******************************************************************************/

MdvResult_t
mdv_setup_driver_essentials(
        MdvDriverEssentials_t *essentials,
        MdvDriverInstance_t *instance,
        MdvDriverInterface_Init_t funcInit,
        MdvDriverInterface_Open_t funcOpen,
        MdvDriverInterface_Close_t funcClose,
        MdvDriverInterface_Sleep_t funcSleep,
        MdvDriverInterface_Wakeup_t funcWakeup
)
{
        if(!essentials||!instance){
                return MDV_ERROR_INVALID_POINTER;
        }
        if(!funcInit||!funcOpen||!funcClose||!funcSleep||!funcWakeup){
                return MDV_ERROR_INVALID_PARAMETER;
        }
        essentials->instance=instance;
        essentials->funcInit=funcInit;
        essentials->funcOpen=funcOpen;
        essentials->funcClose=funcClose;
        essentials->funcSleep=funcSleep;
        essentials->funcWakeup=funcWakeup;
        essentials->initialized=true;
        return MDV_RESULT_OK;
}

/* EOF */