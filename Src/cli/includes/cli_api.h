/***************************************************************************//**
**
**  @file       cli_api.h
**  @ingroup    utils
**  @brief      A general purpose command line input processor.
**  @copyright  Copyright (C) 2012-2018 Tuomas Terho
**
**  This API can be used to build a command line input processor to enable
**  control of an application by user.
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

#ifndef cli_api_H
#define cli_api_H

#include "types.h"

/******************************************************************************\
**
**  ERROR CODES
**
\******************************************************************************/

/// @brief At least one of the pointer parameters is invalid.
#define CLI_ERROR_INVALID_POINTER -1

/// @brief At least one of the value parameters is invalid.
#define CLI_ERROR_INVALID_PARAMETER -2

/// @brief User sent an interrupt process command (Crtl+C).
#define CLI_ERROR_INTERRUPT_PROCESS -10

/// @brief A command handler is invalid.
#define CLI_ERROR_INVALID_COMMAND_HANDLER -11

/// @brief A CLI command is invalid.
///
/// The command must begin with a-z or A-Z (case sensitive).
#define CLI_ERROR_PARSER_INVALID_COMMAND -20

/// @brief A CLI command is not recognized.
///
/// The command is not supported by the application.
#define CLI_ERROR_PARSER_UNKNOWN_COMMAND -21

/// @brief A CLI parameter is invalid.
///
/// The parameter must begin with - or / and it must contain a single character 
/// from the range a-z or A-Z (case sensitive).
#define CLI_ERROR_PARSER_INVALID_PARAMETER -22

/// @brief A CLI parameter is not recognized.
///
/// The parameter is not supported by the command.
#define CLI_ERROR_PARSER_UNKNOWN_PARAMETER -23

/// @brief A CLI parameter value is missing.
///
/// The parameter requires a value, but no value specified on the command line.
#define CLI_ERROR_PARSER_MISSING_PARAMETER_VALUE -24

/// @brief A CLI parameter value is invalid.
///
/// The parameter value is in invalid format (e.g. expected an integer, got a 
/// string).
#define CLI_ERROR_PARSER_INVALID_PARAMETER_VALUE -25

/// @brief A CLI parameter value is out of range.
///
/// The parameter value is out of the expected range.
#define CLI_ERROR_PARSER_PARAMETER_VALUE_OUT_OF_RANGE -26

/******************************************************************************\
**
**  CONSTANTS
**
\******************************************************************************/

/*-------------------------------------------------------------------------*//**
**  @brief The amount of the commands supported by this application.
**
**  This value can be overridden by a user.
*/
#ifndef CLI_CMD_COUNT
        #define CLI_CMD_COUNT 2
#endif // CLI_CMD_COUNT

/*-------------------------------------------------------------------------*//**
**  @brief Declares the maximum amount of parameters for one command. Keep this
**  as small as possible to save memory.
**
**  This value can be overridden by a user.
*/
#ifndef CLI_MAX_PARAM_COUNT
        #define CLI_MAX_PARAM_COUNT 2
#endif // CLI_MAX_PARAM_COUNT

/*-------------------------------------------------------------------------*//**
**  @brief Declares the maximum length of one console input line. Keep this as
**  small as possible to save memory.
**
**  This value can be overridden by a user.
*/
#ifndef CLI_MAX_PARSER_INPUT_LENGTH
        #define CLI_MAX_PARSER_INPUT_LENGTH 128
#endif // CLI_MAX_PARSER_INPUT_LENGTH

/******************************************************************************\
**
**  TYPES
**
\******************************************************************************/

/*-------------------------------------------------------------------------*//**
**  @brief Console command callback type.
**
**  @param[in] cmdId Command identifier (application specific).
**  @param[in] paramCount The amount of parameters in the command line.
**  @param[in] paramIds The list of the parameter identifiers in the command
**      line.
**  @param[in] params The list of the parameter values in the command line.
**
**  @return No return value.
**
**  This callback is invoked by the command interpreter once for each command
**  line command. The callback passes information about the command and its
**  parameters to the user application.
*/
typedef void
(*CLI_CmdCbk_t)(
        uint8_t cmdId,
        uint8_t paramCount,
        uint8_t *paramIds,
        var_t *params
);

/*-------------------------------------------------------------------------*//**
**  @brief A descriptor for a command.
*/
typedef struct
struct_CLI_CmdDescr_t{
        /// The name of the command.
        char *Cmd;
        /// A callback that handles the command.
        CLI_CmdCbk_t Cbk;
} CLI_CmdDescr_t;

/*-------------------------------------------------------------------------*//**
**  @brief Parameter types.
**
**  These types specify how the console treats a parameter.
*/
typedef enum
enum_CLI_ParamType_t{
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
*/
typedef struct
struct_CLI_ParamDescr_t{
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
**  @brief Echo callback type.
**
**  @param[in] ch An echoed character.
**
**  @return No return value.
**
**  This callback is invoked by the command parser to echo the command input
**  line.
*/
typedef void
(*CLI_EchoCbk_t)(
        char ch
);

/*-------------------------------------------------------------------------*//**
**  @brief Parser structure.
*/
typedef struct
struct_CLI_Parser_t{
        /// User defined list of commands line commands.
        const CLI_CmdDescr_t *clicmd;
        /// User defined list of command line parameters.
        const CLI_ParamDescr_t **cliparam;
        /// The current command line input.
        char inp[CLI_MAX_PARSER_INPUT_LENGTH];
        /// Input write pointer.
        char *iptr;
        /// Input character count.
        uint16_t icnt;
        /// The current command id (parser intermediate output).
        uint8_t cmdId;
        /// The current parameter count (parser intermediate output).
        uint8_t pcnt;
        /// The current parameter identifiers (parser intermediate output).
        uint8_t pids[CLI_MAX_PARAM_COUNT];
        /// The current parameter values (parser intermediate output).
        var_t pval[CLI_MAX_PARAM_COUNT];
        /// A callback to echo the command line input.
        CLI_EchoCbk_t ecbk;
        /// Echo enable.
        bool eena;
        /// Parser enable.
        bool pena;
} CLI_Parser_t;

/******************************************************************************\
**
**  API FUNCTIONS
**
\******************************************************************************/

/*-------------------------------------------------------------------------*//**
**  @brief Initializes the console parser.
**
**  @param[out] parser A parser to initialize.
**  @param[in] clicmd User defined list of commands supported by the 
**      application.
**  @param[in] cliparam User defined list of parameters supported by the
**      commands.
**  @param[in] echo A callback for command line echo.
**
**  @return No return value.
*/
Result_t
CLI_Init(
        CLI_Parser_t *parser,
        const CLI_CmdDescr_t *clicmd,
        const CLI_ParamDescr_t **cliparam,
        CLI_EchoCbk_t echo
);

/*-------------------------------------------------------------------------*//**
**  @brief Enables and disables the command line parser.
**
**  @param[in] parser A parser to use.
**  @param[in] enable Enable or disable.
**
**  @retval RESULT_OK Successful.
**  @retval CLI_ERROR_INVALID_POINTER The parser parameter points to null.
**
*/
Result_t
CLI_EnableParser(
        CLI_Parser_t *parser,
        bool enable
);

/*-------------------------------------------------------------------------*//**
**  @brief Enables and disables the command line echo.
**
**  @param[in] parser A parser to use.
**  @param[in] enable Enable or disable.
**
**  @retval RESULT_OK Successful.
**  @retval CLI_ERROR_INVALID_POINTER The parser parameter points to null.
*/
Result_t
CLI_EnableEcho(
        CLI_Parser_t *parser,
        bool enable
);

/*-------------------------------------------------------------------------*//**
**  @brief Puts ASCII characters to the parser and executes actions on command
**      characters.
**
**  @param[in] parser A parser to use.
**  @param[in] input An input character.
**
**  @retval RESULT_OK Successful.
**  @retval CLI_ERROR_INVALID_POINTER The parser parameter points to null.
**  @return On parser errors returns negative error codes. See the error
**      descriptions for more information.
*/
Result_t
CLI_InputChar(
        CLI_Parser_t *parser,
        char input
);

#endif // cli_api_H

/* EOF */
