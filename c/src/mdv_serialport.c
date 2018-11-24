/***************************************************************************//**
**
**  @file       mdv_serialport.c
**  @ingroup    madivaru-lib
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

#include "mdv_serialport.h"

#include <string.h>

/******************************************************************************\
**
**  DATA DEFINITIONS
**
\******************************************************************************/

/*-------------------------------------------------------------------------*//**
**  @brief Default configuration for serial ports.
*/
const MdvSerialPortConfig_t 
serialPortDefaultConfig={
        /// Baud rate = 9600 bps.
        MDV_SERIALPORT_BAUDRATE_9600,
        /// Data bits = 8 data bits.
        MDV_SERIALPORT_DATABITS_8,
        /// Parity = NONE.
        MDV_SERIALPORT_PARITY_NONE,
        /// Stop bits = One stop bit.
        MDV_SERIALPORT_STOPBITS_ONE,
        /// Flow control = NONE.
        MDV_SERIALPORT_FLOWCONTROL_NONE
};

/******************************************************************************\
**
**  LOCAL FUNCTION DEFINITIONS
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
serialport_init_transfer(
        MdvSerialPortTransfer_t *tfer,
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
                mdv_timer_start(tfer->tsys,&tfer->tmr);
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
serialport_complete_transfer(
        MdvSerialPortTransfer_t *tfer,
        MdvResult_t result
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
serialport_cancel_transfer(
        MdvSerialPortTransfer_t *tfer
)
{
        if(!tfer->t_on){
                // No ongoing transfer. Nothing to do.
                return;
        }

        serialport_complete_transfer(
                tfer,
                MDV_SERIALPORT_ERROR_ASYNC_TRANSFER_CANCELLED
        );
}

/*-------------------------------------------------------------------------*//**
**  @brief Performs an asynchronous transfer.
**
**  @param[in] tferFunc Driver Read/Write operation.
**  @param[in] tfer A pointer to a transfer descriptor.
**
**  @retval MDV_RESULT_OK Successful.
**  @retval MDV_SERIALPORT_ERROR_TIMEOUT Timeout.
*/
static MdvResult_t
serialport_asynchronous_transfer(
        MdvSerialPortDriverInterface_Transfer_t tferFunc,
        MdvSerialPortTransfer_t *tfer
)
{
        MdvResult_t result=MDV_RESULT_OK;
        uint32_t time;

        // If there is no transfer ongoing, do nothing.
        if(!tfer->t_on){
                return MDV_RESULT_OK;
        }
        // Retrieve data.
        result=tferFunc(tfer);
        if(!MDV_SUCCESSFUL(result)&&
           result!=MDV_SERIALPORT_ERROR_RX_BUFFER_EMPTY&&
           result!=MDV_SERIALPORT_ERROR_TX_BUFFER_FULL){
                // Something went wrong.
                serialport_complete_transfer(tfer,result);
                return result;
        }
        // Check the length of data to be transferred.
        if(!tfer->tleft){
                // All data retrieved. Complete the transfer with successful
                // result.
                serialport_complete_transfer(tfer,result);
                return MDV_RESULT_OK;
        }
        // If timeout not specified, continue infinitely.
        if(!tfer->tout){
                return MDV_RESULT_OK;
        }
        // If some data got transferred, reset the timeout timer and continue
        // reception.
        if(tfer->tleft!=tfer->ptleft){
                tfer->ptleft=tfer->tleft;
                mdv_timer_start(tfer->tsys,&tfer->tmr);
                return MDV_RESULT_OK;
        }
        // Check the timeout.
        mdv_timer_get_time(tfer->tsys,tfer->tmr,tfer->tu,&time);
        if(time>tfer->tout){
                result=MDV_SERIALPORT_ERROR_TIMEOUT;
                serialport_complete_transfer(tfer,result);
                return result;
        }
        // Nothing happened, but transfer is still in progress.
        return MDV_RESULT_OK;
}

/*-------------------------------------------------------------------------*//**
**  @brief Performs a synchronous transfer.
**
**  @param[in] tferFunc Driver Read/Write operation.
**  @param[in] tfer A pointer to a transfer descriptor.
**
**  @retval MDV_RESULT_OK Successful.
**  @retval MDV_SERIALPORT_ERROR_TIMEOUT Timeout.
*/
static MdvResult_t
serialport_synchronous_transfer(
        MdvSerialPortDriverInterface_Transfer_t tferFunc,
        MdvSerialPortTransfer_t *tfer
)
{
        MdvResult_t result;

        // Loop until the transfer completed.
        while(tfer->t_on){
                // Use the asynchronous transfer function to retrieve data
                // and manage timeouts.
                result=serialport_asynchronous_transfer(tferFunc,tfer);
                if(!MDV_SUCCESSFUL(result)){
                        return result;
                }
        }
        return MDV_RESULT_OK;
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
**  @retval MDV_RESULT_OK Successful.
**  @retval MDV_SERIALPORT_ERROR_TIMEOUT Timeout.
*/
static MdvResult_t
serialport_transfer(
        MdvSerialPortDriverInterface_Transfer_t tferFunc,
        MdvSerialPortTransfer_t *tfer,
        uint32_t length,
        uint8_t *data,
        uint32_t *plen,
        uint32_t timeout
)
{
        // Check if transfer is already in progress.
        if(tfer->t_on){
                return MDV_SERIALPORT_ERROR_ASYNC_TRANSFER_IN_PROGRESS;
        }
        // Init transfer.
        serialport_init_transfer(
                tfer,
                length,
                data,
                plen,
                timeout
        );
        // Check the data length (nothing to transfer if zero).
        if(!tfer->len){
                serialport_complete_transfer(tfer,MDV_RESULT_OK);
                return MDV_RESULT_OK;
        }
        if(!tfer->cbk){
                // No callback defined. Use synchronous transfer.
                return serialport_synchronous_transfer(tferFunc,tfer);
        }
        else{
                return serialport_asynchronous_transfer(tferFunc,tfer);
        }
        return MDV_RESULT_OK;
}

/******************************************************************************\
**
**  API FUNCTION DEFINITIONS
**
\******************************************************************************/

MdvResult_t
mdv_serialport_setup_driver_interface(
        MdvSerialPort_t *port,
        MdvSerialPortDriverInterface_Init_t funcInit,
        MdvSerialPortDriverInterface_Open_t funcOpen,
        MdvSerialPortDriverInterface_Close_t funcClose,
        MdvSerialPortDriverInterface_Transfer_t funcRead,
        MdvSerialPortDriverInterface_Transfer_t funcWrite,
        MdvSerialPortDriverInterface_Run_t funcRun
)
{
        if(!port){
                return MDV_SERIALPORT_ERROR_INVALID_POINTER;
        }
        if(!funcInit||!funcOpen||!funcClose||!funcRead||!funcWrite){
                return MDV_SERIALPORT_ERROR_INVALID_PARAMETER;
        }
        port->drv.funcInit=funcInit;
        port->drv.funcOpen=funcOpen;
        port->drv.funcClose=funcClose;
        port->drv.funcRead=funcRead;
        port->drv.funcWrite=funcWrite;
        port->drv.funcRun=funcRun;
        port->drv.init=true;
        return MDV_RESULT_OK;
}

MdvResult_t
mdv_serialport_init(
        MdvSerialPort_t *port,
        MdvSerialPortTransferCompletedCallback_t rxCallback,
        MdvSerialPortTransferCompletedCallback_t txCallback,
        MdvTimerSystem_t *timerSys,
        MdvTimeUnit_t timeUnit,
        void *userData
)
{
        if(!port){
                return MDV_SERIALPORT_ERROR_INVALID_POINTER;
        }
        if(!port->drv.init){
                return MDV_SERIALPORT_ERROR_NO_DRIVER_INTERFACE;
        }
        // Initialize the rx descriptor.
        memset(&port->rxd,0,sizeof(MdvSerialPortTransfer_t));
        port->rxd.tsys=timerSys;
        port->rxd.tu=timeUnit;
        port->rxd.cbk=rxCallback;
        port->rxd.ud=userData;
        // Initialize the tx descriptor.
        memset(&port->txd,0,sizeof(MdvSerialPortTransfer_t));
        port->txd.tsys=timerSys;
        port->txd.tu=timeUnit;
        port->txd.cbk=txCallback;
        port->txd.ud=userData;
        // Set default configuration.
        port->cfg=serialPortDefaultConfig;
        // Set initialization status.
        port->init=true;
        // Initialize the driver.
        return port->drv.funcInit();        
}

MdvResult_t
mdv_serialport_get_current_configuration(
        MdvSerialPort_t *port,
        MdvSerialPortConfig_t *config
)
{
        if(!port||!config){
                return MDV_SERIALPORT_ERROR_INVALID_POINTER;
        }

        // Configuration data output.
        *config=port->cfg;
        return MDV_RESULT_OK;
}

MdvResult_t
mdv_serialport_open(
        MdvSerialPort_t *port,
        MdvSerialPortConfig_t *config,
        MdvHandle_t *handle
)
{
        MdvResult_t result;

        if(!port||!config||!handle){
                return MDV_SERIALPORT_ERROR_INVALID_POINTER;
        }
        // Check the handle (it should be null).
        if(*handle){
                return MDV_SERIALPORT_ERROR_RESOURCE_IN_USE;
        }
        // Check the port initialization status.
        if(!port->init){
                return MDV_SERIALPORT_ERROR_PORT_NOT_INITIALIZED;
        }
        // Configure and open the port.
        port->cfg=*config;
        result=port->drv.funcOpen(&port->cfg);
        if(!MDV_SUCCESSFUL(result)){
                return result;
        }
        // Handle output.
        *handle=(MdvHandle_t)port;
        return MDV_RESULT_OK;
}


MdvResult_t
mdv_serialport_close(
        MdvHandle_t *handle
)
{
        MdvResult_t result;
        MdvSerialPort_t *port;

        // Check the handle.
        if(!handle){
                return MDV_SERIALPORT_ERROR_INVALID_POINTER;
        }
        port=(MdvSerialPort_t*)*handle;
        // Cancel possibly ongoing asynchronous transfers.
        serialport_cancel_transfer(&port->rxd);
        serialport_cancel_transfer(&port->txd);
        // Close the port.
        result=port->drv.funcClose();
        if(!MDV_SUCCESSFUL(result)){
                return result;
        }
        // Handle reset.
        *handle=(MdvHandle_t)0;
        return MDV_RESULT_OK;
}

MdvResult_t
mdv_serialport_change_configuration(
        MdvHandle_t handle,
        MdvSerialPortConfig_t *config
)
{
        MdvSerialPort_t *port;
        MdvResult_t result;

        if(!config){
                return MDV_SERIALPORT_ERROR_INVALID_POINTER;
        }
        // Check the handle.
        if(!handle){
                return MDV_SERIALPORT_ERROR_INVALID_PARAMETER;
        }
        // Port access.
        port=(MdvSerialPort_t*)handle;
        // Cancel possibly ongoing asynchronous transfers.
        serialport_cancel_transfer(&port->rxd);
        serialport_cancel_transfer(&port->txd);
        // Close the port.
        result=port->drv.funcClose();
        if(!MDV_SUCCESSFUL(result)){
                return result;
        }
        // Port re-configuration and re-opening.
        port->cfg=*config;
        return port->drv.funcOpen(&port->cfg);
}

MdvResult_t
mdv_serialport_read(
        MdvHandle_t handle,
        uint32_t length,
        uint8_t *data,
        uint32_t *bytesRead,
        uint32_t timeout
)
{
        MdvSerialPort_t *port;

        if(!data){
                return MDV_SERIALPORT_ERROR_INVALID_POINTER;
        }
        // Check the port handle.
        if(!handle){
                return MDV_SERIALPORT_ERROR_INVALID_PARAMETER;
        }
        // Port access.
        port=(MdvSerialPort_t*)handle;
        // Start transfer.
        return serialport_transfer(
                port->drv.funcRead,
                &port->rxd,
                length,
                data,
                bytesRead,
                timeout
        );
}

MdvResult_t
mdv_serialport_write(
        MdvHandle_t handle,
        uint32_t length,
        uint8_t *data,
        uint32_t *bytesWritten,
        uint32_t timeout
)
{
        MdvSerialPort_t *port;

        if(!data){
                return MDV_SERIALPORT_ERROR_INVALID_POINTER;
        }
        // Check the port handle.
        if(!handle){
                return MDV_SERIALPORT_ERROR_INVALID_PARAMETER;
        }
        // Port access.
        port=(MdvSerialPort_t*)handle;
        // Init transfer.
        return serialport_transfer(
                port->drv.funcWrite,
                &port->txd,
                length,
                data,
                bytesWritten,
                timeout
        );
}

MdvResult_t
mdv_serialport_getchar(
        MdvHandle_t handle,
        uint8_t *data
)
{
        return mdv_serialport_read(handle,1,data,0,0);
}

MdvResult_t
mdv_serialport_putchar(
        MdvHandle_t handle,
        uint8_t data
)
{
        return mdv_serialport_write(handle,1,&data,0,0);
}

MdvResult_t
mdv_serialport_runtime_process(
        MdvHandle_t handle
)
{
        MdvSerialPort_t *port;
        MdvResult_t result;

        // Check the handle.
        if(!handle){
                return MDV_SERIALPORT_ERROR_INVALID_PARAMETER;
        }
        // Port access.
        port=(MdvSerialPort_t*)handle;

        // Run the driver.
        if(port->drv.funcRun){
                result=port->drv.funcRun(&port->rxd,&port->txd);
                if(!MDV_SUCCESSFUL(result)){
                        return result;
                }
        }

        // Perform asynchronous transfers.
        serialport_asynchronous_transfer(port->drv.funcRead,&port->rxd);
        serialport_asynchronous_transfer(port->drv.funcWrite,&port->txd);
        return MDV_RESULT_OK;
}

/* EOF */