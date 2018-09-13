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
**  ERROR CODES AND RESULTS
**
\******************************************************************************/

/// @brief Parsing finished successfully.
#define CLI_RESULT_PARSING_FINISHED 1

/// @brief The help printed out and parsing stopped.
#define CLI_RESULT_HELP_PRINTED 2

/// @brief At least one of the pointer parameters is invalid.
#define CLI_ERROR_INVALID_POINTER -1

/// @brief At least one of the value parameters is invalid.
#define CLI_ERROR_INVALID_PARAMETER -2

/// @brief User sent an interrupt process command (Crtl+C).
#define CLI_ERROR_INTERRUPT_PROCESS -10

/// @brief A command handler is invalid.
#define CLI_ERROR_INVALID_COMMAND_HANDLER -11

/// @brief An unsupported parameter value type.
#define CLI_ERROR_UNSUPPORTED_PARAMETER_VALUE_TYPE -12

/// @brief A CLI command is invalid.
///
/// The command must begin with a-z or A-Z (case sensitive).
#define CLI_ERROR_PARSER_INVALID_COMMAND -20

/// @brief A CLI command is not recognized.
///
/// The command is not supported by the application.
#define CLI_ERROR_PARSER_UNKNOWN_COMMAND -21

/// @brief A CLI parameter is missing.
///
/// Not all mandatory parameters or none of the alternative parameters found.
#define CLI_ERROR_PARSER_PARAMETER_MISSING -22

/// @brief A CLI parameter is invalid.
///
/// The parameter must begin with - or / and it must contain a single character 
/// from the range a-z or A-Z (case sensitive).
#define CLI_ERROR_PARSER_INVALID_PARAMETER -23

/// @brief A CLI parameter is not recognized.
///
/// The parameter is not supported by the command.
#define CLI_ERROR_PARSER_UNKNOWN_PARAMETER -24

/// @brief A CLI parameter value is missing.
///
/// The parameter requires a value, but no value specified on the command line.
#define CLI_ERROR_PARSER_MISSING_PARAMETER_VALUE -25

/// @brief A CLI parameter value is invalid.
///
/// The parameter value is in invalid format (e.g. expected an integer, got a 
/// string).
#define CLI_ERROR_PARSER_INVALID_PARAMETER_VALUE -26

/// @brief A CLI parameter value is out of range.
///
/// The parameter value is out of the expected range.
#define CLI_ERROR_PARSER_PARAMETER_VALUE_OUT_OF_RANGE -27

/// @brief An extra parameter in the command line.
///
/// There is at least one extra parameter in the command line.
#define CLI_ERROR_PARSER_EXTRA_PARAMETER -28

/// @brief An unexpected end of line.
///
/// Found a parameter or a parameter value specifier, but not the parameter or 
/// the value.
#define CLI_ERROR_PARSER_UNEXPECTED_EOL -29

/// @brief An unexpected parameter value.
///
/// Found a parameter value which was not expected.
#define CLI_ERROR_PARSER_UNEXPECTED_PARAMETER_VALUE -30

/// @brief A duplicated parameter.
///
/// Found a parameter which already exists in the list.
#define CLI_ERROR_PARSER_DUPLICATED_PARAMETER -31

/******************************************************************************\
**
**  TYPES
**
\******************************************************************************/

/*-------------------------------------------------------------------------*//**
**  @brief A container for the parsed parameter ID and value.
*/
typedef struct
CLI_Param_t{
        /// The ID of the parameter.
        uint8_t id;
        /// The value of the parameter.
        var_t val;
} CLI_Param_t;

/*-------------------------------------------------------------------------*//**
**  @brief Console command callback type.
**
**  @param[in] cmdId Command identifier (application specific).
**  @param[in] params The list of the parsed parameters.
**  @param[in] paramCount The amount of parsed parameters in the list.
**  @param[in] userData A pointer to user specified data.
**
**  @retval RESULT_OK Successful.
**  @return On an error returns a negative error code that is passed to the 
**      invoker.
**
**  This callback is invoked by the command interpreter once for each command
**  line command. The callback passes information about the command and its
**  parameters to the user application. The user application may check the
**  parameters and their values and return corresponding error codes.
*/
typedef Result_t
(*CLI_CmdCbk_t)(
        uint8_t cmdId,
        CLI_Param_t *params,
        uint8_t paramCount,
        void *userData
);

/*-------------------------------------------------------------------------*//**
**  @brief A descriptor for a command.
*/
typedef struct
CLI_CmdDescr_t{
        /// The name of the command.
        char *Cmd;
        /// The description of the options of the command.
        char *OptDescr;
        /// The detailed description of the command.
        char *Descr;
        /// A callback that handles the command.
        CLI_CmdCbk_t Cbk;
} CLI_CmdDescr_t;

/*-------------------------------------------------------------------------*//**
**  @brief Parameter types.
**
**  These types specify how the console treats a parameter.
*/
typedef enum
CLI_ParamType_t{
        /// No parameter defined.
        CLI_PARAMTYPE_NONE=0,
        /// The parameter is optional.
        CLI_PARAMTYPE_OPTIONAL,
        /// The parameter is mandatory. A missing parameter causes an error.
        CLI_PARAMTYPE_MANDATORY,
        /// The parameter is an alternative. At least one alternative parameter
        /// must exist in the command line.
        CLI_PARAMTYPE_ALTERNATIVE
} CLI_ParamType_t;

/*-------------------------------------------------------------------------*//**
**  @brief A descriptor for a parameter.
*/
typedef struct
CLI_ParamDescr_t{
        /// The type of the parameter (optional/required).
        CLI_ParamType_t Type;
        /// The name of the parameter.
        char *Param;
        /// The description of the parameter.
        char *Descr;
        /// Indicates whether the parameter should have a value or not.
        bool HasValue;
        /// Type of the parameter value.
        vartype_t VarType;
} CLI_ParamDescr_t;

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
(*CLI_EchoCbk_t)(
        char *str,
        uint16_t length,
        void *userData
);

/*-------------------------------------------------------------------------*//**
**  @brief Parser structure.
*/
typedef struct
CLI_Parser_t{
        /// A user defined list of command line commands.
        const CLI_CmdDescr_t *clicmd;
        /// The amount of the commands in the list.
        uint8_t cliccnt;
        /// A user defined list of command line parameters.
        const CLI_ParamDescr_t *cliprm;
        /// A pointer to an external buffer for parsed parameters.
        CLI_Param_t *pprm;
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
        CLI_EchoCbk_t ecbk;
        /// Echo enable.
        bool eena;
        /// Parser enable.
        bool pena;
        /// Help header.
        const char *hhdr;
        /// A pointer to user specified data.
        void *ud;
} CLI_Parser_t;

/******************************************************************************\
**
**  API FUNCTIONS
**
\******************************************************************************/

/*-------------------------------------------------------------------------*//**
**  @brief Creates a parser for command line input.
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
**  @retval RESULT_OK Successful.
**  @retval CLI_ERROR_INVALID_POINTER One of the pointer parameters parser,
**      inputBuffer, clicmd or cliparam is invalid (points to null).
*/
Result_t
CLI_CreateParser(
        char *inputBuffer,
        uint16_t inputBufferLength,
        const CLI_CmdDescr_t *cliCmds,
        uint8_t cliCmdCount,
        const CLI_ParamDescr_t *cliParams,
        CLI_Param_t *parsedParams,
        uint8_t cliMaxParams,
        const char **cliEnums,
        uint16_t cliEnumCount,
        CLI_EchoCbk_t echo,
        const char *helpHeader,
        void *userData,
        CLI_Parser_t *parser
);

/*-------------------------------------------------------------------------*//**
**  @brief Enables the command line parser.
**
**  @param[in] parser A parser to use.
**
**  @retval RESULT_OK Successful.
**  @retval CLI_ERROR_INVALID_POINTER The parser parameter points to null.
*/
Result_t
CLI_EnableParser(
        CLI_Parser_t *parser
);

/*-------------------------------------------------------------------------*//**
**  @brief Disables the command line parser.
**
**  @param[in] parser A parser to use.
**
**  @retval RESULT_OK Successful.
**  @retval CLI_ERROR_INVALID_POINTER The parser parameter points to null.
*/
Result_t
CLI_DisableParser(
        CLI_Parser_t *parser
);

/*-------------------------------------------------------------------------*//**
**  @brief Enables the command line echo.
**
**  @param[in] parser A parser to use.
**
**  @retval RESULT_OK Successful.
**  @retval CLI_ERROR_INVALID_POINTER The parser parameter points to null.
*/
Result_t
CLI_EnableEcho(
        CLI_Parser_t *parser
);

/*-------------------------------------------------------------------------*//**
**  @brief Disables the command line echo.
**
**  @param[in] parser A parser to use.
**
**  @retval RESULT_OK Successful.
**  @retval CLI_ERROR_INVALID_POINTER The parser parameter points to null.
*/
Result_t
CLI_DisableEcho(
        CLI_Parser_t *parser
);

/*-------------------------------------------------------------------------*//**
**  @brief Begins a new input.
**
**  @param[in] parser A parser to use.
**  @param[in] newLine Add new line before the input (true) or not (false).
**
**  @retval RESULT_OK Successful.
**  @retval CLI_ERROR_INVALID_POINTER The parser parameter points to null.
*/
Result_t
CLI_BeginNewInput(
        CLI_Parser_t *parser,
        bool newLine
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
        uint8_t input
);

#endif // cli_api_H

/* EOF */
