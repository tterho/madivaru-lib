/***************************************************************************//**
**
**  @file       mdv_handle.h
**  @ingroup    madivaru-lib
**  @brief      Handle management API.
**  @copyright  Copyright (c) Tuomas Terho. All rights reserved.
**
**  Function reference:
**
**      mdv_handle_create
**              Use to create (initialize) a handle for an object, and to
**              specify how many times the object can be referenced
**              simultaneuosly.
**
**      mdv_handle_destroy
**              Use to destroy (reset) an unreferenced handle.
**
**      mdv_handle_request
**              Get a pointer to the object associated with the handle, and
**              increase object's reference count.
**
**      mdv_handle_release
**              Release the object reference (decrease object's reference
**              count).
**
**      mdv_handle_get_references
**              Returns object's current reference count.
**
**      mdv_handle_is_valid
**              Tests whether the handle points to an object or not.
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

#ifndef mdv_handle_H
#define mdv_handle_H

#include "mdv_types.h"

/******************************************************************************\
**
**  HANDLE TYPE DEFINITION
**
\******************************************************************************/

/*-------------------------------------------------------------------------*//**
**  @brief Handle data type.
*/
typedef struct
MdvHandle_t{
        /// @brief A pointer to an object.
        void *obj;
        /// @brief Current references.
        int refs;
        /// @brief Maximum references.
        int refsMax;
} MdvHandle_t;

/******************************************************************************\
**
**  API FUNCTION DECLARATIONS
**
\******************************************************************************/

#ifdef __cplusplus
extern "C"{
#endif // ifdef __cplusplus

/*-------------------------------------------------------------------------*//**
**  @brief Creates a handle to an object.
**
**  @param[in] handle A pointer to a handle to create.
**  @param[in] object A pointer to an object.
**  @param[in] refsMax Maximum allowed references to the object (0 = unlimited).
**
**  @retval MDV_RESULT_OK Successful.
**  @retval MDV_ERROR_INVALID_POINTER The handle pointer or the object pointer
**          points to null.
**  @retval MDV_ERROR_RESOURCE_IN_USE The handle already points to an objects
**          (object pointer is not null).
*/
MdvResult_t
mdv_handle_create(
        MdvHandle_t *const handle,
        void *const object,
        uint32_t refsMax
);

/*-------------------------------------------------------------------------*//**
**  @brief Destroys a handle.
**
**  @param[in] handle A handle to destroy.
**
**  @retval MDV_RESULT_OK Successful.
**  @retval MDV_ERROR_INVALID_POINTER The handle pointer points to null.
**  @retval MDV_ERROR_RESOURCE_IN_USE The handle has been referenced and can't
**          be destroyed.
*/
MdvResult_t
mdv_handle_destroy(
        MdvHandle_t *const handle
);

/*-------------------------------------------------------------------------*//**
**  @brief Gets a reference to the object.
**
**  @param[in] handle A pointer to a handle.
**  @param[out] object A pointer to an object pointer to get.
**
**  @retval MDV_RESULT_OK Successful.
**  @retval MDV_ERROR_INVALID_POINTER The handle pointer or the object pointer
**          points to null.
**  @retval MDV_ERROR_INVALID_HANDLE The handle doesn't point to an object.
**  @retval MDV_ERROR_OUT_OF_RESOURCES The maximum amount of references exceeded.
*/
MdvResult_t
mdv_handle_request_object(
        MdvHandle_t *const handle,
        void **object
);

/*-------------------------------------------------------------------------*//**
**  @brief Releases a reference to the object.
**
**  @param[in] handle A pointer to a handle.
**  @param[in] object A pointer to an object pointer to release.
**
**  @retval MDV_RESULT_OK Successful.
**  @retval MDV_ERROR_INVALID_POINTER The handle pointer or the object pointer
**          points to null.
**  @retval MDV_ERROR_INVALID_HANDLE The handle doesn't point to an object.
**  @retval MDV_ERROR_INVALID_PARAMETER The object pointer doesn't match to the
**          object being released.
**  @retval MDV_ERROR_INVALID_OPERATION There are no references to the object
**          left.
*/
MdvResult_t
mdv_handle_release_object(
        MdvHandle_t *const handle,
        void **object
);

/*-------------------------------------------------------------------------*//**
**  @brief Gets the current amount of references to the object.
**
**  @param[in] handle A pointer to a handle.
**  @param[out] refs Pointer to a variable to hold the reference count.
**  @param[out] maxRefs Pointer to a variable to hold the maximum reference
**              count (optional, can be set to null).
**
**  @retval MDV_RESULT_OK Successful.
**  @retval MDV_ERROR_INVALID_POINTER The handle pointer or the refs pointer
**          points to null.
**  @retval MDV_ERROR_INVALID_HANDLE The handle is invalid (doesn't point to an
**          object).
*/
MdvResult_t
mdv_handle_get_references(
        MdvHandle_t *const handle,
        uint32_t *const refs,
        uint32_t *const maxRefs
);

/*-------------------------------------------------------------------------*//**
**  @brief Checks if the handle points to a valid object.
**
**  @param[in] handle A pointer to a handle.
**
**  @retval true The handle points to an object.
**  @retval false The handle doesn't point to an object, or the handle pointer
**          points to null.
*/
bool
mdv_handle_is_valid(
        MdvHandle_t *const handle
);

#ifdef __cplusplus
}
#endif // ifdef __cplusplus

#endif // ifndef mdv_handle_H

/* EOF */
