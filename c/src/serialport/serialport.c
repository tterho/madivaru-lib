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

#include <string.h>

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
        SP_FC_NONE
};

/******************************************************************************\
**
**  LOCAL FUNCTION DECLARATIONS
**
\******************************************************************************/

/*-------------------------------------------------------------------------*//**
**  @brief Initializes a transfer.
**
**  @param[out] tfer A pointer to a transfer descriptor.
**  @param[in] length Data length.
**  @param[in] data A pointer to data.
**  @param[in] plen A pointer to a variable for transferred data length.
**  @param[in] timeout A timeout value.
**
**  @return No return value.
**
**  Marks the transfer incompleted and resets all the required values.
*/
static void
spInitTransfer(
        SP_Transfer_t *tfer,
        uint32_t length,
        uint8_t *data,
        uint32_t *plen,
        uint32_t timeout
)
{
        tfer->t_on=1;
        tfer->len=length;
        tfer->data=data;
        tfer->plen=plen;
        tfer->tout=timeout;
        tfer->tleft=length;
        tfer->ptleft=0;
        // Reset the transferred data length output.
        if(tfer->plen){
                *tfer->plen=0;
        }
        // Start the timeout timer.
        if(tfer->tout){
                TimerAPI_StartTimer(tfer->tsys,&tfer->tmr);
        }
}

/*-------------------------------------------------------------------------*//**
**  @brief Completes a transfer.
**
**  @param[out] tfer A pointer to a transfer descriptor.
**  @param[in] result Result of the transfer.
**
**  @return No return value.
**
**  Marks the transfer completed and invokes a callback if the callback has been
**  set.
*/
static void
spTransferCompleted(
        SP_Transfer_t *tfer,
        Result_t result
)
{
        tfer->t_on=0;

        // Set the transferred data length output.
        if(tfer->plen){
                *tfer->plen=tfer->len-tfer->tleft;
        }
        // Invoke the callback.
        if(tfer->cbk){
                tfer->cbk(result,tfer->ud);
        }
}

/*-------------------------------------------------------------------------*//**
**  @brief Cancels a possibly ongoing asynchronous transfer.
**
**  @param[out] tfer A pointer to a transfer descriptor.
**
**  @return No return value.
**
**  Marks the transfer completed and invokes a callback if the callback has been
**  set.
*/
static void
spCancelTransfer(
        SP_Transfer_t *tfer
)
{
        if(!tfer->t_on){
                // No ongoing transfer. Nothing to do.
                return;
        }

        spTransferCompleted(tfer,SP_ERROR_ASYNC_TRANSFER_CANCELLED);
}

/*-------------------------------------------------------------------------*//**
**  @brief Performs an asynchronous transfer.
**
**  @param[in] tferFunc Driver Read/Write operation.
**  @param[in] tfer A pointer to a transfer descriptor.
**
**  @retval RESULT_OK Successful.
**  @retval SP_ERROR_TIMEOUT Timeout.
*/
static Result_t
spAsyncTransfer(
        SPDrv_Func_Transfer_t tferFunc,
        SP_Transfer_t *tfer
)
{
        Result_t result=RESULT_OK;
        uint32_t time;

        // If there is no transfer ongoing, do nothing.
        if(!tfer->t_on){
                return RESULT_OK;
        }
        // Retrieve data.
        result=tferFunc(tfer);
        if(!SUCCESSFUL(result)&&
           result!=SP_ERROR_RX_BUFFER_EMPTY&&
           result!=SP_ERROR_TX_BUFFER_FULL){
                // Something went wrong.
                spTransferCompleted(tfer,result);
                return result;
        }
        // Check the length of data to be transferred.
        if(!tfer->tleft){
                // All data retrieved. Complete the transfer with successful
                // result.
                spTransferCompleted(tfer,result);
                return RESULT_OK;
        }
        // If timeout not specified, continue infinitely.
        if(!tfer->tout){
                return RESULT_OK;
        }
        // If some data got transferred, reset the timeout timer and continue
        // reception.
        if(tfer->tleft!=tfer->ptleft){
                tfer->ptleft=tfer->tleft;
                TimerAPI_StartTimer(tfer->tsys,&tfer->tmr);
                return RESULT_OK;
        }
        // Check the timeout.
        TimerAPI_GetTimeLapse(tfer->tsys,tfer->tmr,tfer->tu,&time);
        if(time>tfer->tout){
                result=SP_ERROR_TIMEOUT;
                spTransferCompleted(tfer,result);
                return result;
        }
        // Nothing happened, but transfer is still in progress.
        return RESULT_OK;
}

/*-------------------------------------------------------------------------*//**
**  @brief Performs a synchronous transfer.
**
**  @param[in] tferFunc Driver Read/Write operation.
**  @param[in] tfer A pointer to a transfer descriptor.
**
**  @retval RESULT_OK Successful.
**  @retval SP_ERROR_TIMEOUT Timeout.
*/
static Result_t
spSyncTransfer(
        SPDrv_Func_Transfer_t tferFunc,
        SP_Transfer_t *tfer
)
{
        Result_t result;

        // Loop until the transfer completed.
        while(tfer->t_on){
                // Use the asynchronous transfer function to retrieve data
                // and manage timeouts.
                result=spAsyncTransfer(tferFunc,tfer);
                if(!SUCCESSFUL(result)){
                        return result;
                }
        }
        return RESULT_OK;
}
/*-------------------------------------------------------------------------*//**
**  @brief Transfers data to/from serial port.
**
**  @param[in] tferFunc Driver Read/Write function.
**  @param[in] tfer A pointer to a transfer descriptor.
**  @param[in] length Data length;
**  @param[in] data A pointer to data.
**  @param[in] plen A pointer to transferred data length.
**  @param[in] timeout Timeout time in milliseconds.
**
**  @retval RESULT_OK Successful.
**  @retval SP_ERROR_TIMEOUT Timeout.
*/
static Result_t
spTransfer(
        SPDrv_Func_Transfer_t tferFunc,
        SP_Transfer_t *tfer,
        uint32_t length,
        uint8_t *data,
        uint32_t *plen,
        uint32_t timeout
)
{
        // Check if transfer is already in progress.
        if(tfer->t_on){
                return SP_ERROR_ASYNC_TRANSFER_IN_PROGRESS;
        }
        // Init transfer.
        spInitTransfer(
                tfer,
                length,
                data,
                plen,
                timeout
        );
        // Check the data length (nothing to transfer if zero).
        if(!tfer->len){
                spTransferCompleted(tfer,RESULT_OK);
                return RESULT_OK;
        }
        if(!tfer->cbk){
                // No callback defined. Use synchronous transfer.
                return spSyncTransfer(tferFunc,tfer);
        }
        else{
                return spAsyncTransfer(tferFunc,tfer);
        }

        return RESULT_OK;
}

/******************************************************************************\
**
**  API FUNCTION DECLARATIONS
**
\******************************************************************************/

Result_t
SP_SetupDriverInterface(
        SP_COMPort_t *port,
        SPDrv_Func_Init_t funcInit,
        SPDrv_Func_Open_t funcOpen,
        SPDrv_Func_Close_t funcClose,
        SPDrv_Func_Transfer_t funcRead,
        SPDrv_Func_Transfer_t funcWrite,
        SPDrv_Func_RunDriver_t funcRunDriver
)
{
        if(!port){
                return SP_ERROR_INVALID_POINTER;
        }

        if(!funcInit||!funcOpen||!funcClose||!funcRead||!funcWrite){
                return SP_ERROR_INVALID_PARAMETER;
        }

        port->drvfuncInit=funcInit;
        port->drvfuncOpen=funcOpen;
        port->drvfuncClose=funcClose;
        port->drvfuncRead=funcRead;
        port->drvfuncWrite=funcWrite;
        port->drvfuncRunDriver=funcRunDriver;
        return RESULT_OK;
}

Result_t
SP_InitPort(
        SP_COMPort_t *port,
        SP_TransferCompletedCbk_t rxCallback,
        SP_TransferCompletedCbk_t txCallback,
        TimerSys_t *timerSys,
        Timer_TimeUnit_t timeUnit,
        void *userData
)
{
        if(!port){
                return SP_ERROR_INVALID_POINTER;
        }
        if(!port->drvfuncInit){
                return SP_ERROR_NO_DRIVER_INTERFACE;
        }
        // Initialize the rx descriptor.
        memset(&port->rxd,0,sizeof(SP_Transfer_t));
        port->rxd.tsys=timerSys;
        port->rxd.tu=timeUnit;
        port->rxd.cbk=rxCallback;
        port->rxd.ud=userData;
        // Initialize the tx descriptor.
        memset(&port->txd,0,sizeof(SP_Transfer_t));
        port->txd.tsys=timerSys;
        port->txd.tu=timeUnit;
        port->txd.cbk=txCallback;
        port->txd.ud=userData;
        // Set default configuration.
        port->cfg=spDefaultConfig;
        // Set initialization status.
        port->init=true;
        // Initialize the driver.
        return port->drvfuncInit();        
}

Result_t
SP_GetCurrentConfig(
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
        // Check the port initialization status.
        if(!port->init){
                return SP_ERROR_PORT_NOT_INITIALIZED;
        }
        // Configure and open the port.
        port->cfg=*config;
        result=port->drvfuncOpen(&port->cfg);
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

        // Check the handle.
        if(!handle){
                return SP_ERROR_INVALID_POINTER;
        }
        port=(SP_COMPort_t*)*handle;
        // Cancel possibly ongoing asynchronous transfers.
        spCancelTransfer(&port->rxd);
        spCancelTransfer(&port->txd);
        // Close the port.
        result=port->drvfuncClose();
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
        SP_COMPort_t *port;
        Result_t result;

        if(!config){
                return SP_ERROR_INVALID_POINTER;
        }
        // Check the handle.
        if(!handle){
                return SP_ERROR_INVALID_PARAMETER;
        }
        // Port access.
        port=(SP_COMPort_t*)handle;
        // Cancel possibly ongoing asynchronous transfers.
        spCancelTransfer(&port->rxd);
        spCancelTransfer(&port->txd);
        // Close the port.
        result=port->drvfuncClose();
        if(!SUCCESSFUL(result)){
                return result;
        }
        // Port re-configuration and re-opening.
        port->cfg=*config;
        return port->drvfuncOpen(&port->cfg);
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

        if(!data){
                return SP_ERROR_INVALID_POINTER;
        }
        // Check the port handle.
        if(!handle){
                return SP_ERROR_INVALID_PARAMETER;
        }
        // Port access.
        port=(SP_COMPort_t*)handle;
        // Start transfer.
        return spTransfer(
                port->drvfuncRead,
                &port->rxd,
                length,
                data,
                bytesRead,
                timeout
        );
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

        if(!data){
                return SP_ERROR_INVALID_POINTER;
        }
        // Check the port handle.
        if(!handle){
                return SP_ERROR_INVALID_PARAMETER;
        }
        // Port access.
        port=(SP_COMPort_t*)handle;
        // Init transfer.
        return spTransfer(
                port->drvfuncWrite,
                &port->txd,
                length,
                data,
                bytesWritten,
                timeout
        );
}

Result_t
SP_GetChar(
        Handle_t handle,
        uint8_t *data
)
{
        return SP_Read(handle,1,data,0,0);
}

Result_t
SP_PutChar(
        Handle_t handle,
        uint8_t data
)
{
        return SP_Write(handle,1,&data,0,0);
}

Result_t
SP_Run(
        Handle_t handle
)
{
        SP_COMPort_t *port;
        Result_t result;

        // Check the handle.
        if(!handle){
                return SP_ERROR_INVALID_PARAMETER;
        }
        // Port access.
        port=(SP_COMPort_t*)handle;

        // Run the driver.
        if(port->drvfuncRunDriver){
                result=port->drvfuncRunDriver(&port->rxd,&port->txd);
                if(!SUCCESSFUL(result)){
                        return result;
                }
        }

        // Perform asynchronous transfers.
        spAsyncTransfer(port->drvfuncRead,&port->rxd);
        spAsyncTransfer(port->drvfuncWrite,&port->txd);
        return RESULT_OK;
}
/* EOF */