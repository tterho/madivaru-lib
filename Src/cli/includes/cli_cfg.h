/***************************************************************************//**
**
**  @file       cli_cfg.h
**  @ingroup    utils
**  @brief      The user configuration of the command line input processor.
**  @copyright  Copyright (C) 2012-2018 Tuomas Terho
**
**  This file contains a user specified list of commands and their parameters
**  supported by the user application.
**
*******************************************************************************/
/*
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

#ifndef cli_cfg_H
#define cli_cfg_H

#include "types.h"

/*-------------------------------------------------------------------------*//**
**  @brief Declares the maximum size of the command. Keep the maximum size
**  as small as possible to save memory.
*/
//#define CLI_MAX_CMD_SIZE 10

/*-------------------------------------------------------------------------*//**
**  @brief Declares the maximum amount of parameters for one command. Keep this
**  as small as possible to save memory.
*/
#define CLI_MAX_PARAM_COUNT 2

/*-------------------------------------------------------------------------*//**
**  @brief Declares the maximum length of one console input line. Keep this as
**  small as possible to save memory.
*/
#define CLI_MAX_PARSER_INPUT_LENGTH 128

/*-------------------------------------------------------------------------*//**
**  @brief User defined list of command identifiers supported by the console.
**
**  @remarks The CLI_CMDID_Count must appear at the end of the list.
*/
typedef enum
CLI_CmdId_t{
        CLI_CMDID_GETVERSION=0, // An example command.
        CLI_CMDID_HELP, // An example command.
        CLI_CMDID_Count
} CLI_CmdId_t;

/*-------------------------------------------------------------------------*//**
**  @brief User defined list of commands supported by the console.
**
**  @remarks The size of this list is declared automatically by specifying the
**  list of command identifiers in the @ref CLI_CmdId_t enumeration.
**  The items in this list must appear in the same order than the command
**  identifiers in the enumeration.
**
**  @remarks This is a list of pointers to constant strings. The length of the
**  strings can vary.
**
**  @warning Do not use duplicated commands.
*/
const char *
CLI_Cmd[CLI_CMDID_Count]={
        "getv" // An example command.
        "help" // An example command.
};

/*-------------------------------------------------------------------------*//**
**  @brief Parameter types.
**
**  These types specify how the console treats a parameter.
**
**  @warning Do not modify this list.
*/
typedef enum
CLI_ParamType_t{
        /// The parameter is optional.
        CLI_PARAMTYPE_OPTIONAL=0,
        /// The parameter is required. A missing parameter causes an error.
        CLI_PARAMTYPE_REQUIRED,
        /// The parameter is an alternative. At least one alternative parameter
        /// must exist in the command line.
        CLI_PARAMTYPE_ALTERNATIVE
} CLI_ParamType_t;

/*-------------------------------------------------------------------------*//**
**  @brief A descriptor for a parameter.
**
**  This descriptor specifies a parameter.
**
**  @warning Do not modify this structure.
*/
typedef struct
CLI_ParamDescr_t{
        /// The type of the parameter (optional/required).
        CLI_ParamType_t Type;
        /// The name of the parameter.
        char *Param;
        /// Indicates whether the parameter should have a value or not.
        bool HasValue;
        /// Type of the parameter value.
        vartype_t VarType;
} CLI_ParamDescr_t;

/*-------------------------------------------------------------------------*//**
**  @brief User defined list of parameters supported by the commands.
**
**  @remarks The size of this list is declared automatically by specifying the
**  list of command identifiers in the @ref CLI_CmdId_t enumeration, and
**  by specifying the @ref CLI_MAX_PARAM_COUNT constant. The parameters
**  in this list must appear in the same order than the command identifiers in
**  the enumeration.
**
**  @remarks This is a list of pointers to constant strings. The length of the
**  strings can vary.
**
**  @attention The parameter amount per command can vary, but can't exceed the
**  maximum parameter count. If the command implements less than the maximum
**  amount of parameters, the remaining parameters must be set to null in this
**  list.
**
**  @attention The order of the parameters matters. The user application gets
**  the index of the parameter in this list as an identifier if the parameter
**  exists in the command line.
*/
const CLI_ParamDescr_t
CLI_Parameter[CLI_CMDID_Count][CLI_MAX_PARAM_COUNT]={
        // CLI_CMDID_GETVERSION
        {{CLI_PARAMTYPE_ALTERNATIVE,"hw",false,VARTYPE_NONE},
         {CLI_PARAMTYPE_ALTERNATIVE,"sw",false,VARTYPE_NONE},
         0,
         0,
         0},
        // CLI_CMDID_HELP
        {0,
         0,
         0,
         0,
         0}
};

#endif // cli_cfg_H

/* EOF */

