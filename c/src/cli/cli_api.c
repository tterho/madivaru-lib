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
#include <stdlib.h>

/******************************************************************************\
**
**  LOCAL FUNCTIONS
**
\******************************************************************************/

/*-------------------------------------------------------------------------*//**
**  @brief Echoes an input.
**
**  @param[in] parser Parser to use.
**  @param[in] str Input string.
**  @param[in] sz Input string length.
**
**  @return No return value.
*/
static void
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
        parser->ecbk(str,sz,parser->ud);
}

/*-------------------------------------------------------------------------*//**
**  @brief Begins a new line.
**
**  @param[in] parser Parser to use.
**
**  @return No return value.
*/
static void
cpapi_NewLine(
        CLI_Parser_t *parser
)
{
        cpapi_Echo(parser,"\r\n",2);
}

/*-------------------------------------------------------------------------*//**
**  @brief Begins a new input.
**
**  @param[in] parser Parser to use.
**  @param[in] newLine Add new line before input.
**
**  @return No return value.
*/
static void
cpapi_BeginNewInput(
        CLI_Parser_t *parser,
        bool newLine
)
{
        if(newLine){
                cpapi_NewLine(parser);
        }
        parser->iptr=&parser->inp[0];
        parser->icnt=0;
        cpapi_Echo(parser,"> ",2);
}

/*-------------------------------------------------------------------------*//**
**  @brief Prints help about all commands.
**
**  @param[in] parser Parser to use.
**
**  @return No return value.
*/
static void
cpapi_PrintAppHelp(
        CLI_Parser_t *parser
)
{
        uint8_t i;
        const CLI_CmdDescr_t *cmd;

        // Print the help header.
        if(parser->hhdr){
                cpapi_Echo(
                        parser,
                        (char*)parser->hhdr,
                        (uint16_t)strlen(parser->hhdr)
                );
                cpapi_Echo(parser,"\r\n\r\n",4);
        }
        // Print help for all commands.
        for(i=0;i<parser->cliccnt;i++){
                cmd=&parser->clicmd[i];
                cpapi_Echo(parser,"\x09",1);
                cpapi_Echo(parser,cmd->Cmd,(uint16_t)strlen(cmd->Cmd));
                cpapi_Echo(parser,"\x09\x09",2);
                cpapi_Echo(parser,cmd->OptDescr,(uint16_t)strlen(cmd->OptDescr));
                cpapi_Echo(parser,"\r\n\x09\x09\x09",5);
                cpapi_Echo(parser,cmd->Descr,(uint16_t)strlen(cmd->Descr));
                cpapi_Echo(parser,"\r\n\r\n",4);
        }
        cpapi_BeginNewInput(parser,false);
}

/*-------------------------------------------------------------------------*//**
**  @brief Prints help about the parameters of the specific command.
**
**  @param[in] parser Parser to use.
**  @param[in] cmdId A command ID.
**
**  @return No return value.
*/
static void
cpapi_PrintCmdHelp(
        CLI_Parser_t *parser,
        uint8_t cmdId
)
{
        uint8_t i;
        const CLI_CmdDescr_t *cmd;
        const CLI_ParamDescr_t *param;

        // Print help for the command.
        cmd=&parser->clicmd[cmdId];
        cpapi_Echo(parser,cmd->Cmd,(uint16_t)strlen(cmd->Cmd));
        cpapi_Echo(parser," ",1);
        cpapi_Echo(parser,cmd->OptDescr,(uint16_t)strlen(cmd->OptDescr));
        cpapi_Echo(parser,"\r\n",2);
        cpapi_Echo(parser,cmd->Descr,(uint16_t)strlen(cmd->Descr));
        cpapi_Echo(parser,"\r\n\r\n",4);
        // Print help for the parameters of the command.
        for(i=0;i<parser->clipcnt;i++){
                param=&parser->cliprm[cmdId*parser->clipcnt+i];
                if(param->Type==CLI_PARAMTYPE_NONE){
                        continue;
                }
                cpapi_Echo(parser,"\x09-",2);
                cpapi_Echo(parser,param->Param,(uint16_t)strlen(param->Param));
                cpapi_Echo(parser,"\x09\x09",2);
                cpapi_Echo(parser,param->Descr,(uint16_t)strlen(param->Descr));
                cpapi_Echo(parser,"\r\n",2);
        }
        cpapi_BeginNewInput(parser,false);
}

/*-------------------------------------------------------------------------*//**
**  @brief Parses a parameter from the command line.
**
**  @param[in] parser Parser to use.
**  @param[in] i Current index.
**
**  @retval RESULT_OK Successful.
**  @return On an error returns a negative error code.
*/
static Result_t
cpapi_ParseParam(
        CLI_Parser_t *parser,
        uint16_t *i
)
{
        uint16_t j;
        uint8_t k;
        uint8_t len=0;
        const CLI_ParamDescr_t *p=0;
        bool found=false;
        bool chars=false;
        var_t *val=0;

        // EOL found.
        if(*i>=parser->icnt){
                return CLI_RESULT_PARSING_FINISHED;
        }
        // Bypass preceding spaces.
        while(parser->inp[*i]==32){
                (*i)++;
                // EOL found.
                if(*i==parser->icnt){
                        return CLI_RESULT_PARSING_FINISHED;
                }
        }
        // Other than a '-' found: return an error.
        if(parser->inp[*i]!='-'){
                return CLI_ERROR_PARSER_INVALID_PARAMETER;
        }
        (*i)++;
        // EOL found, but not expected: return an error.
        if(*i==parser->icnt){
                return CLI_ERROR_PARSER_UNEXPECTED_EOL;
        }
        // The parameter doesn't begin with a letter: return an error.
        if(!((parser->inp[*i]>='a'&&parser->inp[*i]<='z')||
             (parser->inp[*i]>='A'&&parser->inp[*i]<='A'))){
                return CLI_ERROR_PARSER_INVALID_PARAMETER;
        }
        // Find the end of the parameter name.
        while(parser->inp[*i]!=32&&parser->inp[*i]!='='){
                len++;
                (*i)++;
                if(*i==parser->icnt){
                        break;
                }
        }
        // Go to the beginning of the parameter.
        (*i)-=len;
        // Check if the parameter is a help request.
        if(len==4){
                if(!strncmp(&parser->inp[*i],"help",4)){
                        cpapi_PrintCmdHelp(parser,parser->pcmd);
                        return CLI_RESULT_HELP_PRINTED;
                }
        }
        // Find the parameter from the list.
        for(j=0;j<parser->clipcnt;j++){
                p=&parser->cliprm[parser->clipcnt*parser->pcmd+j];
                if(strlen(p->Param)==len){
                        if(!strncmp(p->Param,&parser->inp[*i],len)){
                                // A parameter found, but the parameter count
                                // exceeds the maximum amount of parameters per
                                // command.
                                if (parser->pcnt == parser->clipcnt)
                                {
                                    return CLI_ERROR_PARSER_EXTRA_PARAMETER;
                                }
                                // A parameter found. Put it into the list and
                                // reset its value.
                                parser->pprm[parser->pcnt].id=(uint8_t)j;
                                parser->pprm[parser->pcnt].t=p->Type;
                                val=&parser->pprm[parser->pcnt].val;
                                val->type=VARTYPE_NONE;
                                val->U32=0;
                                val->sz=0;
                                parser->pcnt++;
                                found=true;
                                break;
                        }
                }
        }
        // Parameter not found in the list: return an error.
        if(!found){
                return CLI_ERROR_PARSER_UNKNOWN_PARAMETER;
        }
        // Check for duplicates.
        for(k=0;k<parser->pcnt-1;k++){
                if(parser->pprm[k].id==(uint8_t)j){
                        return CLI_ERROR_PARSER_DUPLICATED_PARAMETER;
                }
        }
        // These parameter value types are not supported by this parser.
        switch(p->VarType){
        case VARTYPE_F32PTR:
        case VARTYPE_U32PTR:
        case VARTYPE_S32PTR:
        case VARTYPE_U16PTR:
        case VARTYPE_S16PTR:
        case VARTYPE_U8PTR:
        case VARTYPE_BOOLPTR:
                return CLI_ERROR_UNSUPPORTED_PARAMETER_VALUE_TYPE;
        default:
                break;
        }
        // Continue the process from the end of the parameter name.
        (*i)+=len;
        // An EOL or a space found. Check whether the parameter should have had
        // a value or not. If yes, return an error. Otherwise, return RESULT_OK
        // to continue parsing.
        if(*i==parser->icnt||parser->inp[*i]==32){
                if(p->VarType!=VARTYPE_NONE){
                        return CLI_ERROR_PARSER_MISSING_PARAMETER_VALUE;
                }
                else{
                        return RESULT_OK;
                }
        }
        // A '=' found. Check whether the parameter should have had a value or
        // not. If not, return an error.
        if(p->VarType==VARTYPE_NONE){
                return CLI_ERROR_PARSER_UNEXPECTED_PARAMETER_VALUE;
        }
        (*i)++;
        // EOL found, but not expected: return an error.
        if(*i==parser->icnt){
                return CLI_ERROR_PARSER_UNEXPECTED_EOL;
        }
        // Get the value.
        len=0;
        while(parser->inp[*i]!=32){
                (*i)++;
                len++;
                // An EOL found.
                if(*i==parser->icnt){
                        break;
                }
        }
        // No value found. Return an error.
        if(!len){
                return CLI_ERROR_PARSER_MISSING_PARAMETER_VALUE;
        }
        // Set the character at i to null for a null-terminated string.
        parser->inp[*i]=0;
        // Check the value type.
        (*i)-=len;
        for(j=0;j<len;j++){
                if(!((parser->inp[(*i)+j]>='0'&&
                      parser->inp[(*i)+j]<='9')||
                     parser->inp[(*i)+j]=='.'||
                     parser->inp[(*i)+j]=='-')){
                        chars=true;
                        break;
                }
        }
        // The value contains characters when it shouldn't. Return an error.
        switch(p->VarType){
        case VARTYPE_F32:
        case VARTYPE_U32:
        case VARTYPE_S32:
        case VARTYPE_U16:
        case VARTYPE_S16:
        case VARTYPE_U8:
                if(chars){
                        return CLI_ERROR_PARSER_INVALID_PARAMETER_VALUE;
                }
                break;
        default:
                break;
        }
        // Convert the variable depending on its type.
        switch(p->VarType){
        default:
                // This type is beyond the VARTYPE enumeration.
                return CLI_ERROR_UNSUPPORTED_PARAMETER_VALUE_TYPE;
                break;
        case VARTYPE_F32:
                val->F32=(float)atof(&parser->inp[*i]);
                val->type=p->VarType;
                val->sz=1;
                break;
        case VARTYPE_U32:
        case VARTYPE_S32:
        case VARTYPE_U16:
        case VARTYPE_S16:
        case VARTYPE_U8:
                val->S32=atoi(&parser->inp[*i]);
                val->type=p->VarType;
                val->sz=1;
                break;
        case VARTYPE_S8:
                if(len>1){
                        return CLI_ERROR_PARSER_INVALID_PARAMETER_VALUE;
                }
                val->S8=parser->inp[*i];
                break;
        case VARTYPE_S8PTR:
                val->S8Ptr=(int8_t*)&parser->inp[*i];
                val->type=p->VarType;
                val->sz=len;
                break;
        case VARTYPE_BOOL:
                if(len==1&&parser->inp[*i]=='1'||
                   len==4&&!strncmp(&parser->inp[*i],"true",4)){
                        val->Bool=true;
                }else if((len==1&&parser->inp[*i]=='0')||
                        len==5&&!strncmp(&parser->inp[*i],"false",5)){
                        val->Bool=false;
                }else{
                        return CLI_ERROR_PARSER_INVALID_PARAMETER_VALUE;
                }
                break;
        case VARTYPE_ENUM:
                found=false;
                for(j=0;j<parser->cliecnt;j++){
                        if(strlen(parser->clienu[j])==len){
                                if(!strncmp(parser->clienu[j],&parser->inp[*i],len)){
                                        val->Enum=j;
                                        found=true;
                                        break;
                                }
                        }
                }
                if(!found){
                        return CLI_ERROR_PARSER_INVALID_PARAMETER_VALUE;
                }
                break;
        }
        (*i)+=len+1;
        return RESULT_OK;
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
        Result_t result;
        uint8_t m[2]={0};
        uint8_t a[2]={0};

        // Check the first character (must be a-z or A-Z).
        if(!(parser->inp[0]>='a'&&parser->inp[0]<='z'||
             parser->inp[0]>='A'&&parser->inp[0]<='A')){
                return CLI_ERROR_PARSER_INVALID_COMMAND;
        }
        // Find the first space from the command line.
        for(i=0;i<parser->icnt;i++){
                if(parser->inp[i]==32){
                        cmdl=i;
                        break;
                }
        }
        // No spaces found from the whole input buffer. The command is invalid.
        if(i==parser->inl){
                return CLI_ERROR_PARSER_INVALID_COMMAND;
        }
        // Check if the command is a help request.
        if(i==4){
                if(!strncmp(parser->inp,"help",4)){
                        cpapi_PrintAppHelp(parser);
                        return CLI_RESULT_HELP_PRINTED;
                }
        }
        // Search for the matching length commands. On a match, compare the 
        // strings together to find an exact match.
        for(j=0;j<parser->cliccnt;j++){
                if(strlen(parser->clicmd[j].Cmd)==i){
                        if(!strncmp(parser->clicmd[j].Cmd,parser->inp,i)){
                                // Command found.
                                parser->pcmd=j;
                                break;
                        }
                }
        }
        // Command not found.
        if(j==parser->cliccnt){
                return CLI_ERROR_PARSER_UNKNOWN_COMMAND;
        }
        // Command found. Reset the parameter count and parse parameters.
        parser->pcnt=0;
        do{
                result=cpapi_ParseParam(parser,&i);
        } while(result==RESULT_OK);
        // Parsing failed. Return an error.
        if(!SUCCESSFUL(result)){
                return result;
        }
        // The help printed out. Nothing more to do.
        if(result==CLI_RESULT_HELP_PRINTED){
                return RESULT_OK;
        }
        // Calculate the amount of mandatory and alternative parameters in the
        // user defined list. Optional parameters doesn't need to be calculated 
        // or checked, because... umm... they are optional.
        i=parser->pcmd*parser->clipcnt;
        for(j=0;j<parser->clipcnt;j++){
                switch(parser->cliprm[i+j].Type){
                default:break;
                case CLI_PARAMTYPE_MANDATORY:m[0]++;break;
                case CLI_PARAMTYPE_ALTERNATIVE:a[0]++;break;
                }
        }
        // Calculate the amount of mandatory and alternative parameters in the
        // parsed command line.
        for(j=0;j<parser->pcnt;j++){
                switch(parser->pprm[j].t){
                default:
                case CLI_PARAMTYPE_MANDATORY:m[1]++;break;
                case CLI_PARAMTYPE_ALTERNATIVE:a[1]++;break;
                }
        }
        // Check that all mandatory parameters have been found.
        if(m[0]!=m[1]){
                return CLI_ERROR_PARSER_PARAMETER_MISSING;
        }
        // Check that at least one alternative parameter has been found.
        if(a[0]&&!a[1]){
                return CLI_ERROR_PARSER_PARAMETER_MISSING;
        }
        // Check that there is no more than just one alternative parameter.
        if(a[0]&&a[1]>1){
                return CLI_ERROR_PARSER_EXTRA_PARAMETER;
        }
        // The command handler must exist.
        if(!parser->clicmd[parser->pcmd].Cbk){
                return CLI_ERROR_INVALID_COMMAND_HANDLER;
        }
        // Send the parser result to the user application.
        result=parser->clicmd[parser->pcmd].Cbk(
                parser->pcmd,
                parser->pprm,
                parser->pcnt,
                parser->ud
        );
        if(SUCCESSFUL(result)){
                cpapi_BeginNewInput(parser,true);
        }
        return result;
}

/******************************************************************************\
**
**  API FUNCTIONS
**
\******************************************************************************/

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
)
{
        if(!inputBuffer||
           !cliCmds||
           !cliParams||
           !parsedParams||
           !parser){
                return CLI_ERROR_INVALID_POINTER;
        }
        // Reset the parser structure.
        memset(parser,0,sizeof(CLI_Parser_t));
        // Setup the input buffer.
        parser->inp=inputBuffer;
        parser->inl=inputBufferLength;
        // Setup the command line commands and their parameters.
        parser->clicmd=cliCmds;
        parser->cliccnt=cliCmdCount;
        parser->cliprm=cliParams;
        parser->clipcnt=cliMaxParams;
        // Setup the parsed parameter buffer.
        parser->pprm=parsedParams;
        // Setup and enable the echo.
        if(echo){
                parser->ecbk=echo;
                parser->eena=true;
        }
        // Set the user defined enumerations.
        parser->clienu=cliEnums;
        parser->cliecnt=cliEnumCount;
        // Set user data.
        parser->ud=userData;
        // Set the command line input pointer to the beginning of the input
        // buffer.
        parser->iptr=&parser->inp[0];
        // Enable the parser by default.
        parser->pena=true;
        // Set the help header.
        parser->hhdr=helpHeader;
        return RESULT_OK;
}

Result_t
CLI_EnableParser(
        CLI_Parser_t *parser
)
{
        if(!parser){
                return CLI_ERROR_INVALID_POINTER;
        }
        parser->pena=true;
        return RESULT_OK;
}

Result_t
CLI_DisableParser(
        CLI_Parser_t *parser
)
{
        if(!parser){
                return CLI_ERROR_INVALID_POINTER;
        }
        parser->pena=false;
        return RESULT_OK;
}

Result_t
CLI_EnableEcho(
        CLI_Parser_t *parser
)
{
        if(!parser){
                return CLI_ERROR_INVALID_POINTER;
        }
        parser->eena=true;
        return RESULT_OK;
}

Result_t
CLI_BeginNewInput(
        CLI_Parser_t *parser,
        bool newLine
)
{
        if(!parser){
                return CLI_ERROR_INVALID_POINTER;
        }
        cpapi_BeginNewInput(parser,newLine);
        return RESULT_OK;
}

Result_t
CLI_DisableEcho(
        CLI_Parser_t *parser
)
{
        if(!parser){
                return CLI_ERROR_INVALID_POINTER;
        }
        parser->eena=false;
        return RESULT_OK;
}

Result_t
CLI_InputChar(
        CLI_Parser_t *parser,
        uint8_t input
)
{
        Result_t result=RESULT_OK;

        if(!parser){
                return CLI_ERROR_INVALID_POINTER;
        }
        // Handle character input.
        if(input>=32){
                // Check the input buffer capacity.
                if(parser->icnt>=parser->inl){
                        // Input buffer is full. Nothing to do.
                        return RESULT_OK;
                }
                // Echo the input.
                cpapi_Echo(parser,(char*)&input,1);
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
                // Clear the input and send an echo. Then return the value
                // CLI_ERROR_INTERRUPT_PROCESS to indicate that the user 
                // interrupted the operation.
                cpapi_Echo(parser," ^C",3);
                cpapi_BeginNewInput(parser,true);
                return CLI_ERROR_INTERRUPT_PROCESS;
        // Backspace command.
        case 8:
                // Check the input buffer pointer.
                if(!parser->icnt){
                        // Input buffer is empty. Nothing to do.
                        break;
                }
                // Echo the backspace.
                cpapi_Echo(parser,"\x08 \x08",3);
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
                        // Input buffer is empty. Begin a new input.
                        cpapi_BeginNewInput(parser,true);
                        break;
                }
                cpapi_NewLine(parser);
                // Parse the input and return the result.
                result=cpapi_ParseInput(parser);
                // Clear the input.
                parser->iptr=&parser->inp[0];
                parser->icnt=0;
                break;
        // Escape command.
        case 27:
                // Escape command clears the input.
                cpapi_BeginNewInput(parser,true);
                break;
        }
        return result;
}

/* EOF */
