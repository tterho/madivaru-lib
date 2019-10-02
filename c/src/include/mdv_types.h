/***************************************************************************//**
**
**  @defgroup   madivaru-lib Madivaru library.
**
**  General purpose APIs, driver interfaces and utilities.
**
**  @file       mdv_types.h
**  @ingroup    madivaru-lib
**  @brief      Common types and macros
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

#ifndef mdv_types_H
#define mdv_types_H

#include <stdio.h>
#include <inttypes.h>
#include <stdbool.h>

/******************************************************************************\
**
**  GENERIC VARIABLE TYPE
**
\******************************************************************************/

/*-------------------------------------------------------------------------*//**
**  @brief Identifiers of the generic variable data types.
*/
typedef enum
MdvVarType_t{
        MDV_VARTYPE_NONE=0,
#ifdef USE_64BIT_VAR_T
        MDV_VARTYPE_F64,
        MDV_VARTYPE_F64PTR,
        MDV_VARTYPE_U64,
        MDV_VARTYPE_U64PTR,
        MDV_VARTYPE_S64,
        MDV_VARTYPE_S64PTR,
#endif // USE_64BIT_VAR_T
        MDV_VARTYPE_F32,
        MDV_VARTYPE_F32PTR,
        MDV_VARTYPE_U32,
        MDV_VARTYPE_U32PTR,
        MDV_VARTYPE_S32,
        MDV_VARTYPE_S32PTR,
        MDV_VARTYPE_U16,
        MDV_VARTYPE_U16PTR,
        MDV_VARTYPE_S16,
        MDV_VARTYPE_S16PTR,
        MDV_VARTYPE_U8,
        MDV_VARTYPE_U8PTR,
        MDV_VARTYPE_S8,
        MDV_VARTYPE_S8PTR,
        MDV_VARTYPE_BOOL,
        MDV_VARTYPE_BOOLPTR,
        MDV_VARTYPE_ENUM
} MdvVarType_t;

/*-------------------------------------------------------------------------*//**
**  @brief Generic variable data type.
**
**  Maximum of 32-bit variables are used by the default. To get 64-bit variables
**  in use, define a global macro USE_64BIT_VAR_T in the project options.
*/
typedef struct
MdvVar_t{
        /// Type of the variable.
        MdvVarType_t type;
        /// Size of a variable array (when a pointer-type variable points to an
        /// array).
        uint16_t sz;
        union{
#ifdef USE_64BIT_VAR_T
                /// 64-bit floating point value.
                double   F64,*F64Ptr;
                /// 64-bit unsigned integer.
                uint64_t U64,*U64Ptr;
                /// 64-bit signed integer.
                int64_t  S64,*S64Ptr;
                /// 64-bit value.
                uint64_t val;
#else // ifdef USE_64BIT_VAR_T
                /// 32-bit value.
                uint32_t val;
#endif // ifdef USE_64BIT_VAR_T
                /// 32-bit floating point value.
                float    F32,*F32Ptr;
                /// 32-bit unsigned integer.
                uint32_t U32,*U32Ptr;
                /// 32-bit signed integer.
                int32_t  S32,*S32Ptr;
                /// 16-bit unsigned integer.
                uint16_t U16,*U16Ptr;
                /// 16-bit signed integer.
                int16_t  S16,*S16Ptr;
                /// 8-bit unsigned integer.
                uint8_t  U8,*U8Ptr;
                /// 8-bit signed integer.
                int8_t   S8,*S8Ptr;
                /// Boolean.
                bool   Bool,*BoolPtr;
                /// Enumeration.
                int16_t  Enum;
        };
} MdvVar_t;

/******************************************************************************\
**
**  FUNCTION RESULT AND ERROR HANDLING
**
**  The error code range is divided into two sections:
**  - section from -1 to n + 1 is reserved for common (generic) error codes,
**  - section from n to m = INT16_MIN can be used as module specific error 
**    codes.
**
**  Constants and macros for the error code section:
**  - The constant MDV_MODULE_SPECIFIC_ERROR_RANGE specifies the n of the range.
**  - The macro MDV_MODULE_SPECIFIC_ERROR(INDEX) can be used to declare a module
**    specific error code by an index beginning from 0.
**
\******************************************************************************/

/*-------------------------------------------------------------------------*//**
**  @brief Common result data type for function result handling, and common
**      errors values.
*/
typedef int16_t MdvResult_t;

/*-------------------------------------------------------------------------*//**
**  @brief A macro to test a result.
*/
#define MDV_SUCCESSFUL(RESULT) ((RESULT)>=0)

/// @brief Generic definition for a successful result.
#define MDV_RESULT_OK 0

/******************************************************************************\
**
**  COMMON ERROR CODES
** 
\******************************************************************************/

/// @brief A generic (unspecified) error.
#define MDV_ERROR_GENERIC -1

/// @brief A pointer parameter is invalid (points to null).
#define MDV_ERROR_INVALID_POINTER -2

/// @brief A parameter value is invalid.
#define MDV_ERROR_INVALID_PARAMETER -3

/// @brief Trying to use an uninitialized handle.
#define MDV_ERROR_INVALID_HANDLE -4

/// @brief A configuration parameter value is invalid.
#define MDV_ERROR_INVALID_CONFIGURATION -5

/// @brief The operation is not allowed in the current operational state.
#define MDV_ERROR_INVALID_OPERATION -6

/// @brief Out of resources.
#define MDV_ERROR_OUT_OF_RESOURCES -7

/// @brief A resource is in use.
#define MDV_ERROR_RESOURCE_IN_USE -8

/// @brief A device initialization has failed.
#define MDV_ERROR_INITIALIZATION_FAILED -9

/// @brief A driver hardware error.
#define MDV_ERROR_HARDWARE -10

/// @brief A time-out occurred.
#define MDV_ERROR_TIMEOUT -11

/// @brief A driver interface has not been initialized.
#define MDV_ERROR_DRIVER_INTERFACE -12

/// @brief A driver internal failure.
#define MDV_ERROR_DRIVER_INTERNAL_FAILURE -13

/******************************************************************************\
**
**  MODULE SPECIFIC ERROR CODE RANGE
**
\******************************************************************************/

/// @brief Module specific error range begins from this value.
#define MDV_MODULE_SPECIFIC_ERROR_RANGE -100

/*-------------------------------------------------------------------------*//**
**  @brief A macro to specify module specific error codes by an index.
**
**  @param INDEX An index for an error code (beginning from 0).
*/
#define MDV_MODULE_SPECIFIC_ERROR(INDEX) \
        ((MDV_MODULE_SPECIFIC_ERROR_RANGE)-(INDEX))

/******************************************************************************\
**
**  GENERAL PURPOSE VALUE MANIPULATION MACROS
**
\******************************************************************************/

/*-------------------------------------------------------------------------*//**
**  @brief Bit macro.
**
**  @param INDEX Bit index.
**
**  This macro is used to generate a bit mask by the means of the bit index.
*/
#define MDV_BIT(INDEX) (1<<(INDEX))

/*-------------------------------------------------------------------------*//**
**  @brief Increases a value and wraps around.
**
**  @param[in,out] VALUE The value being increased.
**  @param[in] MIN The minimum possible value.
**  @param[in] MAX The maximum possible value.
**
**  The value is increased by one. If it reaches the maximum value, it is
**  wrapped around to the minimum value.
*/
#define MDV_INC_W(VALUE,MIN,MAX) (((VALUE)==(MAX))?(MIN):((VALUE)+1))

/*-------------------------------------------------------------------------*//**
**  @brief Decreases a value and wraps around.
**
**  @param[in,out] VALUE The value being decreased.
**  @param[in] MIN The minimum possible value.
**  @param[in] MAX The maximum possible value.
**
**  The value is decreased by one. If it reaches the minimum value, it is
**  wrapped around to the maximum value.
*/
#define MDV_DEC_W(VALUE,MIN,MAX) (((VALUE)==(MIN))?(MAX):((VALUE)-1))

/*-------------------------------------------------------------------------*//**
**  @brief Increases a value and stops at the maximum value.
**
**  @param[in,out] VALUE The value being increased.
**  @param[in] MAX The maximum possible value.
**
**  The value is increased by one until it reaches the maximum value.
*/
#define MDV_INC_L(VALUE,MAX) (((VALUE)==(MAX))?(MAX):((VALUE)+1))

/*-------------------------------------------------------------------------*//**
**  @brief Decreases a value and stops at the minimum value.
**
**  @param[in,out] VALUE The value being decreased.
**  @param[in] MIN The minimum possible value.
**
**  The value is decreased by one until it reaches the minimum value.
*/
#define MDV_DEC_L(VALUE,MIN) (((VALUE)==(MIN))?(MIN):((VALUE)-1))

/*-------------------------------------------------------------------------*//**
**  @brief Tests the maximum boundary of the value.
**
**  @param[in] VALUE The value being tested.
**  @param[in] MAX The maximum possible value.
**
**  This macro performs a boolean test for the value and the maximum value.
*/
#define MDV_MAX_L(VALUE,MAX) ((VALUE)<(MAX)?(VALUE):(MAX))

/*-------------------------------------------------------------------------*//**
**  @brief Tests the minimum boundary of the value.
**
**  @param[in] VALUE The value being tested.
**  @param[in] MIN The minimum possible value.
**
**  This macro performs a boolean test for the value and the minimum value.
*/
#define MDV_MIN_L(VALUE,MIN) ((VALUE)>(MIN)?(VALUE):(MIN))

/*-------------------------------------------------------------------------*//**
**  @brief Performs MSB alignment between two different size registers from
**      smaller to bigger register.
**
**  @param[in] VALUE Value to be aligned.
**  @param[in] SZ_SRC The binary size of the source register.
**  @param[in] SZ_TRG The binary size of the target register.
**
**  Aligns a smaller register to a bigger one by aligning the value to left.
**  This macro doesn't modify the VALUE parameter.
**
**  Example: A 12-bit A/D converter source value needs to be aligned to a 16-bit
**      target variable:
**      target=ALIGNL(source,12,16);
**
**  @attention The SZ_TRG must be always greater than or equal to the SZ_SRC.
*/
#define MDV_ALIGNL(VALUE,SZ_SRC,SZ_TRG) ((VALUE)<<(SZ_TRG-SZ_SRC))

/*-------------------------------------------------------------------------*//**
**  @brief Performs MSB alignment between two different size of registers from
**      smaller to bigger.
**
**  @param[in] VALUE Value to be aligned.
**  @param[in] SZ_SRC The binary size of the source register.
**  @param[in] SZ_TRG The binary size of the target register.
**
**  Aligns a smaller register to a bigger one by aligning the value to left.
**  This macro stores the result to the VALUE parameter.
**
**  Example: A 12-bit A/D converter value stored into a variable needs to be
**      aligned to a 16-bit value:
**      ALIGNLS(variable,12,16);
**
**  @attention The SZ_TRG must be always greater than or equal to the SZ_SRC.
*/
#define MDV_ALIGNLS(VALUE,SZ_SRC,SZ_TRG) ((VALUE)<<=(SZ_TRG-SZ_SRC))

/*-------------------------------------------------------------------------*//**
**  @brief Performs LSB alignment between to different size of registers from
**      bigger to smaller.
**
**  @param[in,out] VALUE Value to be aligned.
**  @param[in] SZ_SRC The binary size of the source register.
**  @param[in] SZ_TRG The binary size of the target register.
**
**  Aligns a bigger register to a smaller one by aligning the value to right.
**  This macro doesn't modify the VALUE parameter.
**
**  Example: A 16-bit source value must fit into a 10-bit PWM timer/counter
**      target register:
**      target=ALIGNR(source,16,10);
**
**  @attention The SZ_SRC must be always greater than or equal to the SZ_TRG.
*/
#define MDV_ALIGNR(VALUE,SZ_SRC,SZ_TRG) ((VALUE)>>(SZ_SRC-SZ_TRG))

/*-------------------------------------------------------------------------*//**
**  @brief Performs LSB alignment between to different size of registers from
**      bigger to smaller.
**
**  @param[in,out] VALUE Value to be aligned.
**  @param[in] SZ_SRC The binary size of the source register.
**  @param[in] SZ_TRG The binary size of the target register.
**
**  Aligns a bigger register to a smaller one by aligning the value to right.
**  This macro stores the result to the VALUE parameter.
**
**  Example: A 16-bit source value must fit into a 10-bit target value:
**      ALIGNRS(variable,16,10);
**
**  @attention The SZ_SRC must be always greater than or equal to the SZ_TRG.
*/
#define MDV_ALIGNRS(VALUE,SZ_SRC,SZ_TRG) ((VALUE)>>=(SZ_SRC-SZ_TRG))

/*-------------------------------------------------------------------------*//**
**  @brief Rounds a floating point value to an integer.
**
**  @param[in] FVAL A floating point value to round.
*/
#define MDV_FTOI(FVAL) ((uint32_t)((FVAL)+0.5))

#endif // ifndef mdv_types_H

/* EOF */
