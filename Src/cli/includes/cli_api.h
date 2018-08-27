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

#include "cli_cfg.h"

/// @brief At least one of the pointer parameters is invalid.
#define CLI_ERROR_INVALID_POINTER -1

/// @brief At least one of the value parameters is invalid.
#define CLI_ERROR_INVALID_PARAMETER -2

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
        CLI_CmdId_t cmdId,
        uint8_t paramCount,
        uint8_t paramIds[CLI_MAX_PARAM_COUNT],
        var_t params[CLI_MAX_PARAM_COUNT]
);

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
CLI_Parser_t{
        /// The current command line input.
        char i[CLI_MAX_PARSER_INPUT_LENGTH];
        /// Input write pointer.
        char *iptr;
        /// Input count.
        uint16_t icnt;
        /// Command callback register.
        CLI_CmdCbk_t cmdCbk[CLI_CMDID_Count];
        /// The current command id (parser intermediate output).
        CLI_CmdId_t cmdId;
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

/*-------------------------------------------------------------------------*//**
**  @brief Initializes the console parser.
**
**  @param[in] parser A parser to initialize.
**  @param[in] echo A callback for command line echo.
**
**  @return No return value.
*/
Result_t
CLI_Init(
        CLI_Parser_t *parser,
        CLI_EchoCbk_t echo
);

/*-------------------------------------------------------------------------*//**
**  @brief Registers a command callback for a command identifier.
**
**  @param[in] parser The parser to use.
**  @param[in] cmdId The identifier of the command to register.
**  @param[in] cmdCbk The callback that serves the command.
**
**  @retval RESULT_OK Successful.
**  @retval CLI_ERROR_INVALID_POINTER The parser parameter points to null.
**  @retval CLI_ERROR_INVALID_PARAMETER The cmdId is out of range or the
**      cmdCbk points to null.
**
**  Each command has a separated command callback that is invoked when the
**  parser detects the particular command in the command line. However, the
**  commands can be served by a single callback implementation because all the
**  command information, including the command identifier, is passed to the
**  callback function parameters. If using a single callback function
**  implementation, use a switch-case inside the function to serve each command
**  separately.
*/
Result_t
CLI_RegisterCmdCbk(
        CLI_Parser_t *parser,
        CLI_CmdId_t cmdId,
        CLI_CmdCbk_t cmdCbk
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
*/
Result_t
CLI_InputChar(
        CLI_Parser_t *parser,
        char input
);

#endif // cli_api_H

/* EOF */
