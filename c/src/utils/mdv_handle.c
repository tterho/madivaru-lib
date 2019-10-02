/***************************************************************************//**
**
**  @file       handle.c
**  @ingroup    madivaru-lib
**  @brief      Handle management API.
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

#include "mdv_handle.h"

/******************************************************************************\
**
**  API FUNCTION DEFINITIONS
**
\******************************************************************************/

MdvResult_t
mdv_handle_create(
        MdvHandle_t *const handle,
        void *const object,
        uint32_t refsMax
)
{
        if(!handle||!object){
                return MDV_ERROR_INVALID_POINTER;
        }
        if(handle->obj){
                return MDV_ERROR_RESOURCE_IN_USE;
        }
        handle->obj=object;
        handle->refsMax=refsMax;
        return MDV_RESULT_OK;
}

MdvResult_t
mdv_handle_destroy(
        MdvHandle_t *const handle
)
{
        if(!handle){
                return MDV_ERROR_INVALID_POINTER;
        }
        if(handle->obj&&handle->refs){
                return MDV_ERROR_RESOURCE_IN_USE;
        }
        handle->obj=(void*)NULL;
        handle->refs=0;
        handle->refsMax=0;
        return MDV_RESULT_OK;
}

MdvResult_t
mdv_handle_request_object(
        MdvHandle_t *const handle,
        void **object
)
{
        if(!handle||!object){
                return MDV_ERROR_INVALID_POINTER;
        }
        if(!handle->obj){
                return MDV_ERROR_INVALID_HANDLE;
        }
        if(handle->refs>=handle->refsMax){
                return MDV_ERROR_OUT_OF_RESOURCES;
        }
        *object=handle->obj;
        ++handle->refs;
        return MDV_RESULT_OK;
}

MdvResult_t
mdv_handle_release_object(
        MdvHandle_t *const handle,
        void **object
)
{
        if(!handle||!object){
                return MDV_ERROR_INVALID_POINTER;
        }
        if(!handle->obj){
                return MDV_ERROR_INVALID_HANDLE;
        }
        if(*object!=handle->obj){
                return MDV_ERROR_INVALID_PARAMETER;
        }
        if(!handle->refs){
                return MDV_ERROR_INVALID_OPERATION;
        }
        --handle->refs;
        *object=(void*)NULL;
        return MDV_RESULT_OK;
}

MdvResult_t
mdv_handle_get_references(
        MdvHandle_t *const handle,
        uint32_t *const refs,
        uint32_t *const maxRefs
)
{
        if(!handle||!refs){
                return MDV_ERROR_INVALID_POINTER;
        }
        if(!handle->obj){
                return MDV_ERROR_INVALID_HANDLE;
        }
        *refs=handle->refs;
        if(maxRefs){
                *maxRefs=handle->refsMax;
        }
        return MDV_RESULT_OK;
}

bool
mdv_handle_is_valid(
        MdvHandle_t *const handle
)
{
        if(!handle){
                return false;
        }
        if(!handle->obj){
                return false;
        }
        return true;
}

/* EOF */