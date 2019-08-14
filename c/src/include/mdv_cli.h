/***************************************************************************//**
**
**  @file       mdv_cli.h
**  @ingroup    madivaru-lib
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

#ifndef mdv_cli_H
#define mdv_cli_H

#include "mdv_types.h"

/******************************************************************************\
**
**  MODULE SPECIFIC RESULTS AND ERROR CODES
**
\******************************************************************************/

/// @brief Parsing finished successfully.
#define MDV_CLI_RESULT_PARSING_FINISHED 1

/// @brief The help printed out and parsing stopped.
#define MDV_CLI_RESULT_HELP_PRINTED 2

/// @brief User sent an interrupt process command (Crtl+C).
#define MDV_CLI_ERROR_INTERRUPT_PROCESS \
        MDV_MODULE_SPECIFIC_ERROR(0)

/// @brief A command handler is invalid.
#define MDV_CLI_ERROR_INVALID_COMMAND_HANDLER \
        MDV_MODULE_SPECIFIC_ERROR(1)

/// @brief An unsupported parameter value type.
#define MDV_CLI_ERROR_UNSUPPORTED_PARAMETER_VALUE_TYPE \
        MDV_MODULE_SPECIFIC_ERROR(2)

/// @brief A CLI command is invalid.
///
/// The command must begin with a-z or A-Z (case sensitive).
#define MDV_CLI_ERROR_PARSER_INVALID_COMMAND \
        MDV_MODULE_SPECIFIC_ERROR(3)

/// @brief A CLI command is not recognized.
///
/// The command is not supported by the application.
#define MDV_CLI_ERROR_PARSER_UNKNOWN_COMMAND \
        MDV_MODULE_SPECIFIC_ERROR(4)

/// @brief A CLI parameter is missing.
///
/// Not all mandatory parameters or none of the alternative parameters found.
#define MDV_CLI_ERROR_PARSER_PARAMETER_MISSING \
        MDV_MODULE_SPECIFIC_ERROR(5)

/// @brief A CLI parameter is invalid.
///
/// The parameter must begin with - or / and it must contain a single character
/// from the range a-z or A-Z (case sensitive).
#define MDV_CLI_ERROR_PARSER_INVALID_PARAMETER \
        MDV_MODULE_SPECIFIC_ERROR(6)

/// @brief A CLI parameter is not recognized.
///
/// The parameter is not supported by the command.
#define MDV_CLI_ERROR_PARSER_UNKNOWN_PARAMETER \
        MDV_MODULE_SPECIFIC_ERROR(7)

/// @brief A CLI parameter value is missing.
///
/// The parameter requires a value, but no value specified on the command line.
#define MDV_CLI_ERROR_PARSER_MISSING_PARAMETER_VALUE \
        MDV_MODULE_SPECIFIC_ERROR(8)

/// @brief A CLI parameter value is invalid.
///
/// The parameter value is in invalid format (e.g. expected an integer, got a
/// string).
#define MDV_CLI_ERROR_PARSER_INVALID_PARAMETER_VALUE \
        MDV_MODULE_SPECIFIC_ERROR(9)

/// @brief A CLI parameter value is out of range.
///
/// The parameter value is out of the expected range.
#define MDV_CLI_ERROR_PARSER_PARAMETER_VALUE_OUT_OF_RANGE \
        MDV_MODULE_SPECIFIC_ERROR(10)

/// @brief An extra parameter in the command line.
///
/// There is at least one extra parameter in the command line.
#define MDV_CLI_ERROR_PARSER_EXTRA_PARAMETER \
        MDV_MODULE_SPECIFIC_ERROR(11)

/// @brief An unexpected end of line.
///
/// Found a parameter or a parameter value specifier, but not the parameter or
/// the value.
#define MDV_CLI_ERROR_PARSER_UNEXPECTED_EOL \
        MDV_MODULE_SPECIFIC_ERROR(12)

/// @brief An unexpected parameter value.
///
/// Found a parameter value which was not expected.
#define MDV_CLI_ERROR_PARSER_UNEXPECTED_PARAMETER_VALUE \
        MDV_MODULE_SPECIFIC_ERROR(13)

/// @brief A duplicated parameter.
///
/// Found a parameter which already exists in the list.
#define MDV_CLI_ERROR_PARSER_DUPLICATED_PARAMETER \
        MDV_MODULE_SPECIFIC_ERROR(14)

/******************************************************************************\
**
**  TYPE DEFINITIONS
**
\******************************************************************************/

/*-------------------------------------------------------------------------*//**
**  @brief Parameter types.
**
**  These types specify how the console treats a parameter.
*/
typedef enum
MdvCliParameterType_t{
        /// No parameter defined.
        MDV_CLI_PARAMETER_TYPE_NONE=0,
        /// The parameter is optional.
        MDV_CLI_PARAMETER_TYPE_OPTIONAL,
        /// The parameter is mandatory. A missing parameter causes an error.
        MDV_CLI_PARAMETER_TYPE_MANDATORY,
        /// The parameter is an alternative. At least one alternative parameter
        /// must exist in the command line.
        MDV_CLI_PARAMETER_TYPE_ALTERNATIVE
} MdvCliParameterType_t;

/*-------------------------------------------------------------------------*//**
**  @brief A container for the parsed parameter ID and value.
*/
typedef struct
MdvCliParameter_t{
        /// The ID of the parameter.
        uint8_t id;
        /// The value of the parameter.
        MdvVar_t val;
        /// The type of the parameter.
        MdvCliParameterType_t t;
} MdvCliParameter_t;

/*-------------------------------------------------------------------------*//**
**  @brief Console command callback type.
**
**  @param[in] cmdId Command identifier (application specific).
**  @param[in] params The list of the parsed parameters.
**  @param[in] paramCount The amount of parsed parameters in the list.
**  @param[in] userData A pointer to user specified data.
**
**  @retval MDV_RESULT_OK Successful.
**  @return On an error returns a negative error code that is passed to the
**      invoker.
**
**  This callback is invoked by the command interpreter once for each command
**  line command. The callback passes information about the command and its
**  parameters to the user application. The user application may check the
**  parameters and their values and return corresponding error codes.
*/
typedef MdvResult_t
(*MdvCliCommandCallback_t)(
        uint8_t cmdId,
        MdvCliParameter_t *params,
        uint8_t paramCount,
        void *userData
);

/*-------------------------------------------------------------------------*//**
**  @brief A descriptor for a command.
*/
typedef struct
MdvCliCommandDescriptor_t{
        /// The name of the command.
        char *cmd;
        /// The description of the options of the command.
        char *optDescr;
        /// The detailed description of the command.
        char *descr;
        /// A callback that handles the command.
        MdvCliCommandCallback_t Cbk;
} MdvCliCommandDescriptor_t;

/*-------------------------------------------------------------------------*//**
**  @brief A descriptor for a parameter.
*/
typedef struct
MdvCliParameterDescriptor_t{
        /// The type of the parameter (optional/required).
        MdvCliParameterType_t type;
        /// The name of the parameter.
        char *param;
        /// The description of the parameter.
        char *descr;
        /// Indicates whether the parameter should have a value or not.
        bool hasValue;
        /// Type of the parameter value.
        MdvVarType_t varType;
} MdvCliParameterDescriptor_t;

/*-------------------------------------------------------------------------*//**
**  @brief Echo callback type.
**
**  @param[in] str String to echo.
**  @param[in] length Length of the string.
**  @param[in] userData A pointer to user specified data.
**
**  @return No return value.
**
**  This callback is invoked by the command parser to echo the command input
**  line.
*/
typedef void
(*MdvCliEchoCallback_t)(
        char *str,
        uint16_t length,
        void *userData
);

/*-------------------------------------------------------------------------*//**
**  @brief Parser structure.
*/
typedef struct
MdvCliParser_t{
        /// A user defined list of command line commands.
        const MdvCliCommandDescriptor_t *clicmd;
        /// The amount of the commands in the list.
        uint8_t cliccnt;
        /// A user defined list of command line parameters.
        const MdvCliParameterDescriptor_t *cliprm;
        /// A pointer to an external buffer for parsed parameters.
        MdvCliParameter_t *pprm;
        /// The maximum amount of parameters per command.
        uint8_t clipcnt;
        /// A user defined list of enumerated value names for the parameter
        /// values.
        const char **clienu;
        /// The amount of enumerated value names in the list.
        uint16_t cliecnt;
        /// A pointer to an external buffer for the command line input.
        char *inp;
        /// The length of the input buffer.
        uint16_t inl;
        /// Input write pointer.
        char *iptr;
        /// Input character count.
        uint16_t icnt;
        /// The parsed command id.
        uint8_t pcmd;
        /// The parsed parameter count.
        uint8_t pcnt;
        /// A callback to echo the command line input.
        MdvCliEchoCallback_t ecbk;
        /// Echo enable.
        bool eena;
        /// Parser enable.
        bool pena;
        /// Help header.
        const char *hhdr;
        /// A pointer to user specified data.
        void *ud;
} MdvCliParser_t;

#ifdef __cplusplus
extern "C"{
#endif // ifdef __cplusplus

/******************************************************************************\
**
**  API FUNCTION DECLARATIONS
**
\******************************************************************************/

/*-------------------------------------------------------------------------*//**
**  @brief Initializes a parser for command line input.
**
**  @param[in] inputBuffer A pointer to a buffer where the input data is
**      received.
**  @param[in] inputBufferLength The length of the input buffer.
**  @param[in] cliCmds A user defined list of commands supported by the
**      application. The list must contain at least cmdCount items.
**  @param[in] cliCmdCount The amount of commands in supported by the
**      application.
**  @param[in] cliParams A user defined list of parameters supported by the
**      commands. The list must contain at least cmdCount * maxParams items.
**  @param[in] parsedParams A pointer to an external buffer for parsed
**      parameters. The list must contain at least maxParams items.
**  @param[in] cliMaxParams The maximum amount of parameters per command.
**  @param[in] cliEnums A user defined list of enumerated value names for the
**      parameter values.
**  @param[in] cliEnumCount The amount of enumerated value names in the list.
**  @param[in] echo A callback for command line echo.
**  @param[in] helpHeader A pointer to a help header text.
**  @param[in] userData A pointer to user specified data.
**  @param[out] parser A pointer to a parser to create.
**
**  @retval MDV_RESULT_OK Successful.
**  @retval MDV_ERROR_INVALID_POINTER One of the pointer parameters parser,
**      inputBuffer, clicmd or cliparam is invalid (points to null).
*/
MdvResult_t
mdv_cli_init_parser(
        char *inputBuffer,
        uint16_t inputBufferLength,
        const MdvCliCommandDescriptor_t *cliCmds,
        uint8_t cliCmdCount,
        const MdvCliParameterDescriptor_t *cliParams,
        MdvCliParameter_t *parsedParams,
        uint8_t cliMaxParams,
        const char **cliEnums,
        uint16_t cliEnumCount,
        MdvCliEchoCallback_t echo,
        const char *helpHeader,
        void *userData,
        MdvCliParser_t *parser
);

/*-------------------------------------------------------------------------*//**
**  @brief Enables the command line parser.
**
**  @param[in] parser A parser to use.
**
**  @retval MDV_RESULT_OK Successful.
**  @retval MDV_ERROR_INVALID_POINTER The parser parameter points to null.
*/
MdvResult_t
mdv_cli_enable_parser(
        MdvCliParser_t *parser
);

/*-------------------------------------------------------------------------*//**
**  @brief Disables the command line parser.
**
**  @param[in] parser A parser to use.
**
**  @retval MDV_RESULT_OK Successful.
**  @retval MDV_ERROR_INVALID_POINTER The parser parameter points to null.
*/
MdvResult_t
mdv_cli_disable_parser(
        MdvCliParser_t *parser
);

/*-------------------------------------------------------------------------*//**
**  @brief Enables the command line echo.
**
**  @param[in] parser A parser to use.
**
**  @retval MDV_RESULT_OK Successful.
**  @retval MDV_ERROR_INVALID_POINTER The parser parameter points to null.
*/
MdvResult_t
mdv_cli_enable_echo(
        MdvCliParser_t *parser
);

/*-------------------------------------------------------------------------*//**
**  @brief Disables the command line echo.
**
**  @param[in] parser A parser to use.
**
**  @retval MDV_RESULT_OK Successful.
**  @retval MDV_ERROR_INVALID_POINTER The parser parameter points to null.
*/
MdvResult_t
mdv_cli_disable_echo(
        MdvCliParser_t *parser
);

/*-------------------------------------------------------------------------*//**
**  @brief Begins a new input.
**
**  @param[in] parser A parser to use.
**  @param[in] newLine Add new line before the input (true) or not (false).
**
**  @retval MDV_RESULT_OK Successful.
**  @retval MDV_ERROR_INVALID_POINTER The parser parameter points to null.
*/
MdvResult_t
mdv_cli_begin_new_input(
        MdvCliParser_t *parser,
        bool newLine
);

/*-------------------------------------------------------------------------*//**
**  @brief Puts an ASCII character to the parser and executes actions on a
**      command characters.
**
**  @param[in] parser A parser to use.
**  @param[in] input An input character.
**
**  @retval MDV_RESULT_OK Successful.
**  @retval MDV_ERROR_INVALID_POINTER The parser parameter points to null.
**  @return On parser errors returns negative error codes. See the error
**      descriptions for more information.
*/
MdvResult_t
mdv_cli_putchar(
        MdvCliParser_t *parser,
        uint8_t input
);

#ifdef __cplusplus
}
#endif // ifdef __cplusplus

#endif // ifndef mdv_cli_H

/* EOF */
