/***************************************************************************//**
**
**  @file       SerialPort.c
**  @ingroup    serialcomm
**  @brief      Serial port API.
**  @copyright  Copyright (C) 2012-2018 Tuomas Terho. All rights reserved.
**
**  Common serial port interface which can be easily ported for different 
**  platforms without need to change the control interface.
**
**  This interface doesn't support port enumeration which is typically used in
**  Windows environment to find hot-pluggable devices, such as USB-to-serial 
**  port adapters.
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
**  Driver mapping table.
*/
SPDrv_t *spDrvMap[SP_MAX_PORTS]={0};

/******************************************************************************\
**
**  PRIVATE FUNCTION DECLARATIONS
**
\******************************************************************************/

/*------------------------------------------------------------------------------
**  Checks validity of a serial port handle.
*/
static Result_t
spIsHandleValid(
        void *handle
)
{
        uint8_t i;
        
        if(!handle){
                return SP_ERROR_INVALID_POINTER;
        }
        // Handle check using driver mapping table.
        for(i=0;i<SP_MAX_PORTS;i++){
                if(spDrvMap[i]==(SPDrv_t*)handle){
                        return RESULT_OK;
                }
        }
        return SP_ERROR_INVALID_PARAMETER;
}

/******************************************************************************\
**
**  PUBLIC DATA DECLARATIONS
**
\******************************************************************************/

const SP_Config_t 
SP_DefaultConfig={
        SP_BR_9600, // .BaudRate
        SP_DB_8,    // .DataBits
        SP_PA_NONE, // .Parity
        SP_SB_ONE,  // .StopBits
        SP_FC_NONE, // .FlowControl
};

/******************************************************************************\
**
**  PUBLIC FUNCTION DECLARATIONS
**
\******************************************************************************/

/*------------------------------------------------------------------------------
**  Maps a driver to a serial port number.
*/
Result_t
SP_MapDriver(
        SP_COMPort_t port,
        SPDrv_t *driver
)
{
        Result_t result;
        
        if(!driver){
                return SP_ERROR_INVALID_POINTER;
        }        
        // Check the port range.
        if(port>=SP_MAX_PORTS)
        {
                return SP_ERROR_INVALID_PARAMETER;
        }        
        // Check port existence.
        if(spDrvMap[port]){
                return SP_ERROR_INVALID_PARAMETER;
        }
        // Initialize the driver.
        result=driver->Init();        
        if(!SUCCESSFUL(result)){
                return result;
        }        
        // Map the driver.
        spDrvMap[port]=driver;
        return RESULT_OK;
}

/*------------------------------------------------------------------------------
**  Returns the current configuration of the given serial port.
*/
Result_t
SP_GetCurrentConfiguration(
        SP_COMPort_t port,
        SP_Config_t *config
)
{
        SPDrv_t *drv;
        
        if(!config){
                return SP_ERROR_INVALID_POINTER;
        }
        // Check the port range.
        if(port>=SP_MAX_PORTS){
                return SP_ERROR_INVALID_PARAMETER;
        }        
        // Find the driver.
        drv=spDrvMap[port];
        if(!drv){
                return SP_ERROR_INVALID_PARAMETER;
        }
        // Configuration data output.
        *config=drv->cfg;
        return RESULT_OK;
}

/*------------------------------------------------------------------------------
**  Opens a serial port.
*/
Result_t
SP_Open(
        SP_COMPort_t port,
        SP_Config_t *config,
        void **handle
)
{
        Result_t result;
        SPDrv_t *drv;
        
        if(!config||!handle){
                return SP_ERROR_INVALID_POINTER;
        }        
        // Check the port range.
        if(port>=SP_MAX_PORTS){
                return SP_ERROR_INVALID_PARAMETER;
        }
        // Check the handle (it should be free).
        if(SUCCESSFUL(spIsHandleValid(*handle))){
                return SP_ERROR_RESOURCE_IN_USE;
        }
        // Find the driver.
        drv=spDrvMap[port];        
        if(!drv){
                return SP_ERROR_INVALID_PARAMETER;
        }
        // Configure and open the port.
        drv->cfg=*config;
        result=drv->Open();
        if(!SUCCESSFUL(result)){
                return result;
        }
        // Handle output.
        *handle=(void*)drv;
        return RESULT_OK;
}

/*------------------------------------------------------------------------------
**  Closes a serial port.
*/
Result_t
SP_Close(
        void **handle
)
{
        Result_t result;
        SPDrv_t *drv;
        
        if(!handle){
                return SP_ERROR_INVALID_POINTER;
        }        
        // Check the handle.
        if(!SUCCESSFUL(spIsHandleValid(*handle))){
                return SP_ERROR_INVALID_PARAMETER;
        }
        // Port access.
        drv=(SPDrv_t*)(*handle);
        // Port closing.
        result=drv->Close();
        if(!SUCCESSFUL(result)){
                return result;
        }
        // Handle reset.
        *handle=(void*)0;                
        return RESULT_OK;
}

/*------------------------------------------------------------------------------
**  Changes serial port's configuration.
*/
Result_t
SP_ChangeConfig(
        void *handle,
        SP_Config_t *config
)
{
        Result_t result;
        SPDrv_t *drv;
        
        if(!config){
                return SP_ERROR_INVALID_POINTER;
        }
        // Check the handle.
        if(!SUCCESSFUL(spIsHandleValid(handle))){
                return SP_ERROR_INVALID_PARAMETER;
        }
        // Port access.
        drv=(SPDrv_t*)handle;
        // Close the port.
        result=drv->Close();
        if(!SUCCESSFUL(result)){
                return result;
        }
        // Port re-configuration and re-opening.
        drv->cfg=*config;
        return drv->Open();
}

/*------------------------------------------------------------------------------
**  Reads data from the serial port.
*/
Result_t
SP_Read(
        void *handle,
        uint32_t length,
        uint8_t *data,
        uint32_t *bytesRead,
        uint32_t timeout
)
{
        Result_t result;
        SPDrv_t *drv;
        Timer_t tmr;
        uint32_t time;
        
        if(!data){
                return SP_ERROR_INVALID_POINTER;
        }
        // Check the handle.
        if(!SUCCESSFUL(spIsHandleValid(handle))){
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
        drv=(SPDrv_t*)handle;
        // Invoke the driver implementation if existing.
        if(drv->Read){
                return drv->Read(length,data,bytesRead,timeout);
        }
        // Local implementation.
        // Initialize the timer.
        result=TimerAPI_StartTimer(drv->tsys,&tmr);        
        if(!SUCCESSFUL(result)){
                return SP_ERROR_DRIVER_INTERNAL_ERROR;
        }
        // Receive data.
        while(length){
                // Read data from port.
                result=drv->GetChar(data);
                // Data received.
                if(SUCCESSFUL(result)){
                        data++;
                        length--;
                        if(bytesRead){
                                (*bytesRead)++;
                        }
                        // Restart the timeout timer.
                        TimerAPI_StartTimer(drv->tsys,&tmr);
                        continue;
                }
                // An error occurred, or the Rx buffer is empty and timeout has
                // not been specified.
                if(result!=SP_ERROR_RX_BUFFER_EMPTY||!timeout){
                        return result;
                }
                // Rx buffer is empty. Check the timeout.
                TimerAPI_GetTimeLapse(drv->tsys,tmr,&time);
                if(time>timeout){
                        return SP_ERROR_TIMEOUT;
                }
        }
        return RESULT_OK;
}

/*------------------------------------------------------------------------------
**  Writes data to the serial port.
*/
Result_t
SP_Write(
        void *handle,
        uint32_t length,
        uint8_t *data,
        uint32_t *bytesWritten,
        uint32_t timeout
)
{
        SPDrv_t *drv;
        Result_t result;
        Timer_t tmr;
        uint32_t time;
        
        if(!data){
                return SP_ERROR_INVALID_POINTER;
        }
        // Check the port handle.
        if(!SUCCESSFUL(spIsHandleValid(handle))){
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
        drv=(SPDrv_t*)handle;
        // Invoke the driver implementation if existing.
        if(drv->Write){
                return drv->Write(length,data,bytesWritten,timeout);
        }
        // Initialize the timer.
        result=TimerAPI_StartTimer(drv->tsys,&tmr);
        if(!SUCCESSFUL(result)){
                return SP_ERROR_DRIVER_INTERNAL_ERROR;
        }
        // Data transmission.
        while(length){
                // Write data to the port.
                result=drv->PutChar(*data);
                // Data sent.
                if(SUCCESSFUL(result)){
                        data++;
                        length--;                        
                        if(bytesWritten){
                                (*bytesWritten)++;
                        }
                        // Restart the timer.
                        TimerAPI_StartTimer(drv->tsys,&tmr);
                        continue;
                }                
                // An error occurred, or buffer is full and timeout has not
                // been specified.
                if(result!=SP_ERROR_TX_BUFFER_FULL||!timeout){
                        return result;
                }                        
                // Buffer is full. Check the timeout.
                TimerAPI_GetTimeLapse(drv->tsys,tmr,&time);
                if(time>timeout){
                        return SP_ERROR_TIMEOUT;
                }
        }
        return RESULT_OK;
}

/*------------------------------------------------------------------------------
**  Gets a single character from a serial port.
*/
Result_t
SP_GetChar(
        void *handle,
        uint8_t *data
)
{
        SPDrv_t *drv;
        
        if(!data){
                return SP_ERROR_INVALID_POINTER;
        }
        // Check the handle.
        if(!SUCCESSFUL(spIsHandleValid(handle))){
                return SP_ERROR_INVALID_PARAMETER;
        }
        // Port access.
        drv=(SPDrv_t*)handle;
        // Read data.
        return drv->GetChar(data);
}

/*------------------------------------------------------------------------------
**  Puts a single character to a serial port.
*/
Result_t
SP_PutChar(
        void *handle,
        uint8_t data
)
{
        SPDrv_t *drv;

        // Check the handle.
        if(!SUCCESSFUL(spIsHandleValid(handle))){
                return SP_ERROR_INVALID_PARAMETER;
        }
        // Port access.
        drv=(SPDrv_t*)handle;
        // Write data.
        return drv->PutChar(data);
}

/* EOF */