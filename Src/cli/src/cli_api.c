/***************************************************************************//**
**
**  @file       cli_api.c
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

#include "cli_api.h"
#include <string.h>
#include <ctype.h>

/*-------------------------------------------------------------------------*//**
**  @brief Echoes an input.
**
**  @param[in] parser Parser to use.
**  @param[in] str Input string.
**  @param[in] sz Input string length.
**
**  @return No return value.
*/
void
cpapi_Echo(
        CLI_Parser_t *parser,
        char *str,
        uint16_t sz
)
{
        // If the echo callback has not been set or echo is disabled, do
        // nothing.
        if(!parser->ecbk||!parser->eena){
                return;
        }
        // Echo all characters.
        while(sz){
                parser->ecbk(*str);
                ++str;
                --sz;
        }
}

/*-------------------------------------------------------------------------*//**
**  @brief Parses an input.
**
**  @param[in] parser Parser to use.
**
**  @return Returns RESULT_OK when line parsed successfully.
**  @return On a parser error returns a negative error code. See the error code
**      descriptions for detailed information.
*/
Result_t
cpapi_ParseInput(
        CLI_Parser_t *parser
)
{
        uint16_t i;
        uint16_t cmdl;
        uint8_t j;

        // Check the first character (must be a-z or A-Z).
        if(!isalpha(parser->inp[0])){
                return CLI_ERROR_PARSER_INVALID_COMMAND;
        }
        // Find the first space from the command line.
        for(i=0;i<CLI_MAX_PARSER_INPUT_LENGTH;i++){
                if(parser->inp[i]==32){
                        cmdl=i;
                        break;
                }
        }
        // No spaces found. The command is invalid.
        if(i==CLI_MAX_PARSER_INPUT_LENGTH){
                return CLI_ERROR_PARSER_INVALID_COMMAND;
        }
        // Search for the similar length commands. On a match, compare the 
        // strings together to find an exact match.
        for(j=0;j<CLI_CMD_COUNT;j++){
                if(strlen(parser->clicmd[j].Cmd)==i){
                        if(strncmp(parser->clicmd[j].Cmd,parser->inp[0],i)){
                                // Command found.
                                parser->cmdId=j;
                                break;
                        }
                }
        }
        // Command not found.
        if(j==CLI_CMD_COUNT){
                return CLI_ERROR_PARSER_UNKNOWN_COMMAND;
        }


        // TODO: Finalize the parser.

        // The command handler must exist.
        if(!parser->clicmd[parser->cmdId].Cbk){
                return CLI_ERROR_INVALID_COMMAND_HANDLER;
        }
        // Send the parser result to the user application.
        parser->clicmd[parser->cmdId].Cbk(
                parser->cmdId,
                parser->pcnt,
                parser->pids,
                parser->pval
        );
        return RESULT_OK;
}

/*------------------------------------------------------------------------------
**  Initializes the console parser.
*/
Result_t
CLI_Init(
        CLI_Parser_t *parser,
        const CLI_CmdDescr_t *clicmd,
        const CLI_ParamDescr_t **cliparam,
        CLI_EchoCbk_t echo
)
{
        if(!parser){
                return CLI_ERROR_INVALID_POINTER;
        }
        memset(parser,0,sizeof(CLI_Parser_t));
        // Enable the parser by default.
        parser->pena=true;
        // Set the command line commands and their parameters.
        parser->clicmd=clicmd;
        parser->cliparam=cliparam;
        // Setup and enable the echo.
        if(echo){
                parser->ecbk=echo;
                parser->eena=true;
        }
        // Set the command line input pointer to the beginning of the input
        // buffer.
        parser->iptr=&parser->inp[0];
        return RESULT_OK;
}

/*------------------------------------------------------------------------------
**  Enables and disables the command line parser.
*/
Result_t
CLI_EnableParser(
        CLI_Parser_t *parser,
        bool enable
)
{
        if(!parser){
                return CLI_ERROR_INVALID_POINTER;
        }
        parser->pena=enable;
        return RESULT_OK;
}

/*------------------------------------------------------------------------------
**  Enables and disables the command line echo.
*/
Result_t
CLI_EnableEcho(
        CLI_Parser_t *parser,
        bool enable
)
{
        if(!parser){
                return CLI_ERROR_INVALID_POINTER;
        }
        parser->eena=enable;
        return RESULT_OK;
}

/*------------------------------------------------------------------------------
**  Puts an ASCII character to the parser, or executes an action on a command
**      character.
*/
Result_t
CLI_InputChar(
        CLI_Parser_t *parser,
        char input
)
{
        if(!parser){
                return CLI_ERROR_INVALID_POINTER;
        }
        // Check that the input is in the valid range.
        if(input<0||input>126){
                return CLI_ERROR_INVALID_PARAMETER;
        }
        // Handle character input.
        if(input>=32){
                // Check the input buffer capacity.
                if(parser->icnt>=CLI_MAX_PARSER_INPUT_LENGTH){
                        // Input buffer is full. Nothing to do.
                        return RESULT_OK;
                }
                // Echo the input.
                cpapi_Echo(parser,&input,1);
                // Set the input character to the buffer and advance the pointer
                // and the counter.
                *parser->iptr=input;
                ++parser->iptr;
                ++parser->icnt;
                return RESULT_OK;
        }
        // Handle special command input.
        switch(input){
        // For unsupported special commands do nothing.
        default:
                break;
        // Interrupt process command (Ctrl+C).
        case 3:
                // Clears the input and send an echo. Then return the value
                // CLI_ERROR_INTERRUPT_PROCESS to indicate that the user 
                // interrupted the operation.
                parser->iptr=&parser->inp[0];
                parser->icnt=0;
                cpapi_Echo(parser,"\r\n^C\r\n",2);
                return CLI_ERROR_INTERRUPT_PROCESS;
        // Backspace command.
        case 8:
                // Check the input buffer pointer.
                if(!parser->icnt){
                        // Input buffer is empty. Nothing to do.
                        return RESULT_OK;
                }
                // Move the input buffer pointer backwards and decrease the
                // counter.
                --parser->iptr;
                --parser->icnt;
                break;
        // Enter command.
        case 13:
                // The enter command starts parsing of the input.
                // If there is nothing to parse, echo a line feed.
                if(!parser->icnt){
                        // Input buffer is empty. Echo a line feed.
                        cpapi_Echo(parser,"\r\n",2);
                        break;
                }
                // Parse the input and return the result.
                return cpapi_ParseInput(parser);
        // Escape command.
        case 27:
                // Escape command clears the input.
                parser->iptr=&parser->inp[0];
                parser->icnt=0;
                cpapi_Echo(parser,"\r\n",2);
                break;

        }
        return RESULT_OK;
}

/* EOF */
