/***************************************************************************//**
**
**  @file       serialport.c
**  @ingroup    serialcomm
**  @brief      Serial port API.
**  @copyright  Copyright (C) 2012-2018 Tuomas Terho. All rights reserved.
**
*******************************************************************************/
/*
**  BSD 3-Clause License
**
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

#include "serialport.h"

/******************************************************************************\
**
**  DATA DECLARATIONS
**
\******************************************************************************/

/*-------------------------------------------------------------------------*//**
**  @brief Default configuration for serial ports.
*/
const SP_Config_t 
spDefaultConfig={
        /// Baud rate = 9600 bps.
        SP_BR_9600,
        /// Data bits = 8 data bits.
        SP_DB_8,
        /// Parity = NONE.
        SP_PA_NONE,
        /// Stop bits = One stop bit.
        SP_SB_ONE,
        /// Flow control = NONE.
        SP_FC_NONE,
};

/******************************************************************************\
**
**  PUBLIC FUNCTION DECLARATIONS
**
\******************************************************************************/

Result_t
SP_InitPort(
        SP_COMPort_t *port,
        SP_TransferCompletedCbk_t rxCallback,
        SP_TransferCompletedCbk_t txCallback,
        TimerSys_t *timerSys
)
{
        Result_t result;
        
        if(!port){
                return SP_ERROR_INVALID_POINTER;
        }        
        // Initialize the rx descriptor.
        port->rx.len=0;
        port->rx.ton=0;
        port->rx.xlen=0;
        port->rx.cmpl=0;
        port->rx.cbk=rxCallback;
        // Initialize the tx descriptor.
        port->tx.len=0;
        port->tx.ton=0;
        port->tx.xlen=0;
        port->tx.cmpl=0;
        port->tx.cbk=txCallback;
        // Set default configuration.
        port->cfg=spDefaultConfig;
        // Set the timer system.
        port->tsys=timerSys;
        // Initialize the driver.
        return port->Init();        
}

Result_t
SP_GetCurrentConfiguration(
        SP_COMPort_t *port,
        SP_Config_t *config
)
{
        if(!port||!config){
                return SP_ERROR_INVALID_POINTER;
        }
        // Configuration data output.
        *config=port->cfg;
        return RESULT_OK;
}

Result_t
SP_Open(
        SP_COMPort_t *port,
        SP_Config_t *config,
        Handle_t *handle
)
{
        Result_t result;
        
        if(!port||!config||!handle){
                return SP_ERROR_INVALID_POINTER;
        }
        // Check the handle (it should be null).
        if(*handle){
                return SP_ERROR_RESOURCE_IN_USE;
        }
        // Configure and open the port.
        port->cfg=*config;
        result=port->Open();
        if(!SUCCESSFUL(result)){
                return result;
        }
        // Handle output.
        *handle=(Handle_t)port;
        return RESULT_OK;
}

Result_t
SP_Close(
        Handle_t *handle
)
{
        Result_t result;
        SP_COMPort_t *port;
        
        if(!handle){
                return SP_ERROR_INVALID_POINTER;
        }        
        // Check the handle.
        if(!*handle){
                return SP_ERROR_INVALID_PARAMETER;
        }
        port=(SP_COMPort_t*)*handle;
        // Close the port.
        result=port->Close();
        if(!SUCCESSFUL(result)){
                return result;
        }
        // Handle reset.
        *handle=(Handle_t)0;            
        return RESULT_OK;
}

Result_t
SP_ChangeConfig(
        Handle_t handle,
        SP_Config_t *config
)
{
        Result_t result;
        SP_COMPort_t *port;
        
        if(!config){
                return SP_ERROR_INVALID_POINTER;
        }
        // Check the handle.
        if(!handle){
                return SP_ERROR_INVALID_PARAMETER;
        }
        // Port access.
        port=(SP_COMPort_t*)handle;
        // Close the port.
        result=port->Close();
        if(!SUCCESSFUL(result)){
                return result;
        }
        // Port re-configuration and re-opening.
        port->cfg=*config;
        return port->Open();
}

Result_t
SP_Read(
        Handle_t handle,
        uint32_t length,
        uint8_t *data,
        uint32_t *bytesRead,
        uint32_t timeout
)
{
        SP_COMPort_t *port;
        Result_t result;
        Timer_t tmr;
        uint32_t time;
        
        if(!data){
                return SP_ERROR_INVALID_POINTER;
        }
        // Check the handle.
        if(!handle){
                return SP_ERROR_INVALID_PARAMETER;
        }
        // Reset the length output parameter.
        if(bytesRead){
                *bytesRead=0;
        }
        // Check the data length (nothing to receive if zero).
        if(!length){
                return RESULT_OK;
        }        
        // Port access.
        port=(SP_COMPort_t*)handle;
        // Invoke the driver implementation if existing.
        if(port->Read){
                return port->Read(length,data,bytesRead,timeout);
        }
        // If the Read function is not implemented, the GetChar function is
        // a requirement.
        if(!port->GetChar){
                return SP_ERROR_DRIVER_INTERNAL_ERROR;
        }        
        // Local implementation.
        // Initialize the timer.
        result=TimerAPI_StartTimer(port->tsys,&tmr);        
        if(!SUCCESSFUL(result)){
                return SP_ERROR_DRIVER_INTERNAL_ERROR;
        }
        // Receive data.
        while(length){
                // Read data from port.
                result=port->GetChar(data);
                // Data received.
                if(SUCCESSFUL(result)){
                        data++;
                        length--;
                        if(bytesRead){
                                (*bytesRead)++;
                        }
                        // Restart the timeout timer.
                        TimerAPI_StartTimer(port->tsys,&tmr);
                        continue;
                }
                // An error occurred, or the Rx buffer is empty and timeout has
                // not been specified.
                if(result!=SP_ERROR_RX_BUFFER_EMPTY||!timeout){
                        return result;
                }
                // Rx buffer is empty. Check the timeout.
                TimerAPI_GetTimeLapse(port->tsys,tmr,TIMER_TU_MS,&time);
                if(time>timeout){
                        return SP_ERROR_TIMEOUT;
                }
        }
        return RESULT_OK;
}

Result_t
SP_Write(
        Handle_t handle,
        uint32_t length,
        uint8_t *data,
        uint32_t *bytesWritten,
        uint32_t timeout
)
{
        SP_COMPort_t *port;
        Result_t result;
        Timer_t tmr;
        uint32_t time;
        
        if(!data){
                return SP_ERROR_INVALID_POINTER;
        }
        // Check the port handle.
        if(!handle){
                return SP_ERROR_INVALID_PARAMETER;
        }
        // Check the data length (nothing to send if zero).
        if(!length){
                return RESULT_OK;
        }
        // Reset the length output parameter.
        if(bytesWritten){
                *bytesWritten=0;
        }
        // Port access.
        port=(SP_COMPort_t*)handle;
        // Invoke the driver implementation if existing.
        if(port->Write){
                return port->Write(length,data,bytesWritten,timeout);
        }
        // If the Write function is not implemented, the PutChar function is
        // a requirement.
        if(!port->PutChar){
                return SP_ERROR_DRIVER_INTERNAL_ERROR;
        }        
        // Initialize the timer.
        result=TimerAPI_StartTimer(port->tsys,&tmr);
        if(!SUCCESSFUL(result)){
                return SP_ERROR_DRIVER_INTERNAL_ERROR;
        }
        // Data transmission.
        while(length){
                // Write data to the port.
                result=port->PutChar(*data);
                // Data sent.
                if(SUCCESSFUL(result)){
                        data++;
                        length--;                        
                        if(bytesWritten){
                                (*bytesWritten)++;
                        }
                        // Restart the timer.
                        TimerAPI_StartTimer(port->tsys,&tmr);
                        continue;
                }                
                // An error occurred, or buffer is full and timeout has not
                // been specified.
                if(result!=SP_ERROR_TX_BUFFER_FULL||!timeout){
                        return result;
                }                        
                // Buffer is full. Check the timeout.
                TimerAPI_GetTimeLapse(port->tsys,tmr,TIMER_TU_MS,&time);
                if(time>timeout){
                        return SP_ERROR_TIMEOUT;
                }
        }
        return RESULT_OK;
}

Result_t
SP_GetChar(
        Handle_t handle,
        uint8_t *data
)
{
        SP_COMPort_t *port;
        
        if(!data){
                return SP_ERROR_INVALID_POINTER;
        }
        // Check the handle.
        if(!handle){
                return SP_ERROR_INVALID_PARAMETER;
        }
        // Port access.
        port=(SP_COMPort_t*)handle;
        // Read data.
        return port->GetChar(data);
}

Result_t
SP_PutChar(
        Handle_t handle,
        uint8_t data
)
{
        SP_COMPort_t *port;

        // Check the handle.
        if(!handle){
                return SP_ERROR_INVALID_PARAMETER;
        }
        // Port access.
        port=(SP_COMPort_t*)handle;
        // Write data.
        return port->PutChar(data);
}

/* EOF */