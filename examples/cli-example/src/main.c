/***************************************************************************//**
**
**  @defgroup   cli-example CLI example application
**
**  A Windows Console application for Visual Studio Community. This application
**  demonstrates how to create a CLI interface by using the madivaru-lib CLI 
**  API.
**
**  @file       main.c
**  @ingroup    cli-example
**  @brief      The application main.
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

#include <conio.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "types.h"
#include "cli_api.h"
#include "cli_cmd_proc.h"

/******************************************************************************\
**
**  LOCAL FUNCTIONS
**
\******************************************************************************/

/*-------------------------------------------------------------------------*//**
**  @brief A callback handler for the CLI echo.
**
**  @param[in] str A pointer to a string to echo.
**  @param[in] length The length of the string.
**  @param[in] userData A pointer to user defined data.
**
**  @return No return value.
*/
static void
cliEchoCbk(
        char *str,
        uint16_t length,
        void *userData
)
{
        // Print the string to the command line application window.
        printf(str);
}

/*-------------------------------------------------------------------------*//**
**  @brief Application main.
**
**  @return The exit status of the application.
*/
int main(
        void
)
{
        uint8_t ch;

        // Initialize the CLI command processor.
        CLICmdProc_Init(cliEchoCbk);
        while(1){
                // Get a character from the user input.
                ch=_getch();
                // Process the character by the command processor.
                CLICmdProc_Process(ch);
        }
        return 1;
}

/* EOF */