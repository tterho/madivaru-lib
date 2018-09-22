/***************************************************************************//**
**  @file       cli_cmd_proc.c
**  @ingroup    cli-example
**  @brief      CLI command processor.
**  @copyright  Copyright (C) 2012-2018 Tuomas Terho. All rights reserved.
**
*******************************************************************************/
/*
**  BSD 3-Clause License
**
**  Copyright (c) 2018, Tuomas Terho
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

#include "cli_cmd_proc.h"
#include <string.h>

CLI_Parser_t CLIParser;

/******************************************************************************\
**
**  CLI CONSTANTS
**
\******************************************************************************/

/// @brief The amount of CLI commands supported by this application.
///
/// Change this value according to the amount of different commands supported by
/// your application. Remember to specify every command and its parameters in
/// the particular lists below.
///
/// In this application we support two commands, getver and setled (help command
/// is supported by the CLI API automatically).
#define CLI_CMD_COUNT 3

/// @brief The maximum amount of parameters per CLI command supported by this
///     application.
///
/// Change this value according to the maximum amount of different parameters a
/// command can get in your application. Remember to specify every command and
/// its parameters in the particular lists below.
///
/// In this application we support up to two parameters (getver has one, and 
/// setled two). The -help parameter is supported for each command automatically
/// by the CLI API.
#define CLI_MAX_PARAM_COUNT 2

/// @brief The size of the input buffer.
///
/// Specify the maximum length of one command line input string.
#define CLI_INPUT_BUFFER_SIZE 64

/******************************************************************************\
**
**  CLI ENUMERATIONS
**
\******************************************************************************/

/*-------------------------------------------------------------------------*//**
**  @brief The list of the enumerated command line parameter values.
**
**  The CLI parser can search the input line for strings that represent numeric 
**  enumeration values. When found a matching string, the parameter gets the 
**  index of the string in the enumeration list as the value.
**
**  This enumeration type is the list of indexes. It is not necessary, but helps
**  identifying the parameter value as the enumeration value.
**
**  In this example we use the enumeration to declare control values for a 
**  bicolor LED.
*/
typedef enum 
CLI_Enum_t{
        /// Set a bicolor LED off.
        CLI_ENUM_off=0,
        /// Set a bicolor LED to red.
        CLI_ENUM_red,
        /// Set a bicolor LED to green.
        CLI_ENUM_green,
        /// Set a bicolor LED to yellow (both red and green on).
        CLI_ENUM_yellow,
        /// The amount of enums in this list.
        CLI_ENUM_COUNT
} CLI_Enum_t;

/*-------------------------------------------------------------------------*//**
**  @brief The list of the enumerated command line parameter strings.
**
**  The items in this array are sorted in the same order than the values in the 
**  above enumeration.
*/
static const char *
cliEnums[CLI_ENUM_COUNT]={
        "off",
        "red",
        "green",
        "yellow"
};

/******************************************************************************\
**
**  CLI COMMAND HANDLER FUNCTION PROTOTYPES
**
\******************************************************************************/

/*-------------------------------------------------------------------------*//**
**  @brief Handles a getver command.
**
**  @param[in] cmdId The command identifier.
**  @param[in] params The list of parameters.
**  @param[in] paramCount The amount of parameters in the list.
**  @param[in] userData A pointer to user specified data.
**
**  @retval RESULT_OK Successful.
*/
static Result_t
cliCmdProc_getver(
        uint8_t cmdId,
        CLI_Param_t *params,
        uint8_t paramCount,
        void *userData
);

/*-------------------------------------------------------------------------*//**
**  @brief Handles a sayhello command.
**
**  @param[in] cmdId The command identifier.
**  @param[in] params The list of parameters.
**  @param[in] paramCount The amount of parameters in the list.
**  @param[in] userData A pointer to user specified data.
**
**  @retval RESULT_OK Successful.
*/
static Result_t
cliCmdProc_sayhello(
        uint8_t cmdId,
        CLI_Param_t *params,
        uint8_t paramCount,
        void *userData
);

/*-------------------------------------------------------------------------*//**
**  @brief Handles a setled command.
**
**  @param[in] cmdId The command identifier.
**  @param[in] params The list of parameters.
**  @param[in] paramCount The amount of parameters in the list.
**  @param[in] userData A pointer to user specified data.
**
**  @retval RESULT_OK Successful.
*/
static Result_t
cliCmdProc_setled(
        uint8_t cmdId,
        CLI_Param_t *params,
        uint8_t paramCount,
        void *userData
);

/******************************************************************************\
**
**  COMMAND AND PARAMETER DECLARATIONS
**
\******************************************************************************/

/*-------------------------------------------------------------------------*//**
**  @brief CLI commands.
*/
const CLI_CmdDescr_t 
cliCmds[CLI_CMD_COUNT]={
        // getver
        {
                // The name of the command searched from the CLI input line.
                "getver",
                // The parameter desription line.
                "[-sw|-hw]",
                // The command description line.
                "Get software and hardware version numbers.",
                // The command handler function.
                cliCmdProc_getver
        },
        // sayhello
        {
                "sayhello",
                "(no parameters)",
                "Prints \"Hello world!\" on the screen.",
                cliCmdProc_sayhello
        },
        // setled
        {
                "setled",
                "[-led=<1|2|3>][-color=<off|red|green|yellow>]",
                "Set a sensor module power on and off.",
                cliCmdProc_setled
        }
};

/*-------------------------------------------------------------------------*//**
**  @brief CLI parameters.
*/
const CLI_ParamDescr_t 
cliParams[CLI_CMD_COUNT*CLI_MAX_PARAM_COUNT]={
        // getver
        {
                // The parameter type (alternative / mandatory / optional / no 
                // parameter).
                CLI_PARAMTYPE_ALTERNATIVE,
                // The parameter name searched from the CLI input line.
                "sw",
                // The description of the parameter.
                "Prints the software version number in format \"x.x.x\"",
                // A flag that indicates if the parameter has value or not.
                false,
                // Parameter value type.
                VARTYPE_NONE
        },
        {
                CLI_PARAMTYPE_ALTERNATIVE,
                "hw",
                "Prints the hardware version number.",
                false,
                VARTYPE_NONE
        },
        // sayhello
        {
                CLI_PARAMTYPE_NONE
        },
        {
                CLI_PARAMTYPE_NONE
        },        
        // setled
        {
                CLI_PARAMTYPE_MANDATORY,
                "led",
                "Select the LED (1-3).",
                true,
                VARTYPE_U8
        },
        {
                CLI_PARAMTYPE_MANDATORY,
                "color",
                "Set the LED color.",
                true,
                VARTYPE_ENUM
        }
};

/******************************************************************************\
**
**  CLI PARSER DATA
**
\******************************************************************************/

/// @brief A buffer for the parser command line input.
static char cliInputBfr[CLI_INPUT_BUFFER_SIZE];

/// @brief A storage for parsed parameters.
static CLI_Param_t cliParsedParams[CLI_MAX_PARAM_COUNT];

/******************************************************************************\
**
**  LOCAL DATA
**
\******************************************************************************/

/// @brief A callback function to be used for echo inside this module.
static CLI_EchoCbk_t cliEcho;

/// @brief A buffer for the output string.
static char cliOutputBfr[128];

static const char *cliHelpHeader=
        "CLI Example Application HELP\r\n"\
        "Type \"<command> -help\" for the details of the command.";

/******************************************************************************\
**
**  LOCAL FUNCTIONS
**
\******************************************************************************/

/*-------------------------------------------------------------------------*//**
**  @brief A pseudo-function to demonstrate LED color setup.
*/
#define SetLED(A,B)
/// @brief A pseudo color value to demonstrated LED color setup.
#define LED_OFF
/// @brief A pseudo color value to demonstrated LED color setup.
#define LED_COLOR_GREEN
/// @brief A pseudo color value to demonstrated LED color setup.
#define LED_COLOR_RED
/// @brief A pseudo color value to demonstrated LED color setup.
#define LED_COLOR_YELLOW

/*-------------------------------------------------------------------------*//**
**  @brief Outputs a string.
**
**  @param[in] str String to output.
**
**  @return No return value.
*/
static void 
cliOutputStr(
        char *str
)
{
        if(!cliEcho){
                return;
        }
        cliEcho(str,(uint16_t)strlen(str),0);
}

/*-------------------------------------------------------------------------*//**
**  @brief Verboses a parser error.
** 
**  @param[in] result An error code to verbose.
**
**  @return No return value.
*/
static void 
cliVerboseError(
        Result_t result
)
{
        switch(result){
        default :
        case RESULT_OK :
                break;
        case CLI_ERROR_PARSER_INVALID_COMMAND :
                cliOutputStr("Error: Invalid command.");
                CLI_BeginNewInput(&CLIParser,true);
                break;
        case CLI_ERROR_PARSER_UNKNOWN_COMMAND :
                cliOutputStr("Error: Unknown command.");
                CLI_BeginNewInput(&CLIParser,true);
                break;
        case CLI_ERROR_PARSER_PARAMETER_MISSING :
                cliOutputStr("Error: Missing parameter(s).");
                CLI_BeginNewInput(&CLIParser,true);
                break;
        case CLI_ERROR_PARSER_INVALID_PARAMETER :
                cliOutputStr("Error: Invalid parameter.");
                CLI_BeginNewInput(&CLIParser,true);
                break;
        case CLI_ERROR_PARSER_UNKNOWN_PARAMETER :
                cliOutputStr("Error: Unknown parameter.");
                CLI_BeginNewInput(&CLIParser,true);
                break;
        case CLI_ERROR_PARSER_MISSING_PARAMETER_VALUE :
                cliOutputStr("Error: Missing parameter value.");
                CLI_BeginNewInput(&CLIParser,true);
                break;
        case CLI_ERROR_PARSER_INVALID_PARAMETER_VALUE :
                cliOutputStr("Error: Invalid parameter value.");
                CLI_BeginNewInput(&CLIParser,true);
                break;
        case CLI_ERROR_PARSER_PARAMETER_VALUE_OUT_OF_RANGE :
                cliOutputStr("Error: Parameter value out of range.");
                CLI_BeginNewInput(&CLIParser,true);
                break;
        case CLI_ERROR_PARSER_EXTRA_PARAMETER :
                cliOutputStr("Error: Too many parameters.");
                CLI_BeginNewInput(&CLIParser,true);
                break;
        case CLI_ERROR_PARSER_UNEXPECTED_EOL :
                cliOutputStr("Error: Unexpected end of line.");
                CLI_BeginNewInput(&CLIParser,true);
                break;
        case CLI_ERROR_PARSER_UNEXPECTED_PARAMETER_VALUE :
                cliOutputStr("Error: Unexpected parameter value.");
                CLI_BeginNewInput(&CLIParser,true);
                break;
        case CLI_ERROR_PARSER_DUPLICATED_PARAMETER :
                cliOutputStr("Error: Duplicated parameter.");
                CLI_BeginNewInput(&CLIParser,true);
                break;
        }
}

/******************************************************************************\
**
**  CLI COMMAND HANDLER FUNCTIONS
**
\******************************************************************************/

static Result_t
cliCmdProc_getver(
        uint8_t cmdId,
        CLI_Param_t *params,
        uint8_t paramCount,
        void *userData
)
{
        if(params[0].id==0){
                // Parameter -sw found. Echo the software version number.
                sprintf(cliOutputBfr,"Software version: %d.%d.%d",1,2,3);
        }
        else if(params[0].id==1){
                // Parameter -hw found. Echo the hardware version number.
                sprintf(cliOutputBfr,"Hardware version: %d",123);
        }
        cliOutputStr(cliOutputBfr);
        return RESULT_OK;
}

static Result_t
cliCmdProc_sayhello(
        uint8_t cmdId,
        CLI_Param_t *params,
        uint8_t paramCount,
        void *userData
)
{
        cliOutputStr("Hello world!");
        return RESULT_OK;
}

static Result_t 
cliCmdProc_setled(
        uint8_t cmdId,
        CLI_Param_t *params,
        uint8_t paramCount,
        void *userData
)
{
        uint8_t i;
        uint8_t led=0;
        CLI_Enum_t color=(CLI_Enum_t)0;
    
        // Find the LED selection and the color settings from the parameters.
        for(i=0;i<paramCount;i++){
                switch(params[i].id){
                default:return CLI_ERROR_PARSER_INVALID_PARAMETER;
                case 0:led=params[i].val.U8;break;
                case 1:color=(CLI_Enum_t)params[i].val.Enum; break;
                }
        }
        // Check the LED selection. Only LEDs 1-3 are allowed (as described in
        // the parameter description). Return the invalid parameter value error
        // for other values.
        if(!led||led>3){
                return CLI_ERROR_PARSER_INVALID_PARAMETER_VALUE;
        }
        // Set the LED color.
        switch(color){
        default:
                return CLI_ERROR_PARSER_INVALID_PARAMETER_VALUE;
        case CLI_ENUM_off:
                SetLED(led,LED_OFF); 
                sprintf(cliOutputBfr,"LED %d is OFF",led);
                break;
        case CLI_ENUM_red:
                SetLED(led,LED_RED);
                sprintf(cliOutputBfr,"LED %d is RED",led);
                break;
        case CLI_ENUM_green:
                SetLED(led,LED_GREEN);
                sprintf(cliOutputBfr,"LED %d is GREEN",led);
                break;
        case CLI_ENUM_yellow:
                SetLED(led,LED_YELLOW);
                sprintf(cliOutputBfr,"LED %d is YELLOW",led);
                break;
        }
        cliOutputStr(cliOutputBfr);
        return RESULT_OK;
}

/******************************************************************************\
**
**  API FUNCTIONS
**
\******************************************************************************/

void CLICmdProc_Init (CLI_EchoCbk_t echo)
{
        // Create a CLI parser.
        CLI_CreateParser(
                cliInputBfr,
                CLI_INPUT_BUFFER_SIZE,
                cliCmds,
                CLI_CMD_COUNT,
                cliParams,
                cliParsedParams,
                CLI_MAX_PARAM_COUNT,
                cliEnums,
                CLI_ENUM_COUNT,
                echo,
                cliHelpHeader,
                0,
                &CLIParser
        );
        // Copy the echo callback pointer locally to allow this module to print
        // messages too.
        cliEcho=echo;
        // Print a welcome message.
        cliOutputStr(
                "Example CLI Application\r\n"\
                "Copyright \xb8 Tuomas Terho 2018.\r\n"\
                "All rights reserved."
        );
        // Begin new input.
        CLI_BeginNewInput(&CLIParser,true);
}

void CLICmdProc_Process (uint8_t input)
{
    Result_t result;
    
    // Put the input character into the CLI parser.
    result=CLI_InputChar(&CLIParser,input);
    // Verbose possible errors.
    cliVerboseError(result);
}

/* EOF */