/***************************************************************************//**
**
**  @file       mdv_serialport.c
**  @ingroup    madivaru-lib
**  @brief      Serial port API.
**  @copyright  Copyright (c) Tuomas Terho. All rights reserved.
**
*******************************************************************************/
/*
**  BSD 3-Clause License
**
**  COPYRIGHT (c) Tuomas Terho
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
        /// @brief Baud rate = 9600 bps.
        MDV_SERIALPORT_BAUDRATE_9600,
        /// @brief Data bits = 8 data bits.
        MDV_SERIALPORT_DATABITS_8,
        /// @brief Parity = NONE.
        MDV_SERIALPORT_PARITY_NONE,
        /// @brief Stop bits = One stop bit.
        MDV_SERIALPORT_STOPBITS_ONE,
        /// @brief Flow control = NONE.
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
        MdvSerialPortTransfer_t *const tfer,
        uint32_t length,
        uint8_t *const data,
        uint32_t *const plen,
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
                *(tfer->plen)=0;
        }
        // Start the timeout timer.
        if(tfer->tout){
                mdv_timer_start(&(tfer->tmr));
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
        MdvSerialPortTransfer_t *const tfer,
        MdvResult_t result
)
{
        tfer->t_on=0;

        // Set the transferred data length output.
        if(tfer->plen){
                *(tfer->plen)=tfer->len-tfer->tleft;
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
        MdvSerialPortTransfer_t *const tfer
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
**  @param[in] instance A pointer to a driver instance.
**  @param[in] tferFunc Driver Read/Write operation.
**  @param[in] tfer A pointer to a transfer descriptor.
**
**  @retval MDV_RESULT_OK Successful.
**  @retval MDV_ERROR_TIMEOUT Timeout.
*/
static MdvResult_t
serialport_asynchronous_transfer(
        MdvDriverInstance_t *const instance,
        MdvSerialPortDriverTransferFunction_t tferFunc,
        MdvSerialPortTransfer_t *const tfer
)
{
        MdvResult_t result=MDV_RESULT_OK;
        uint32_t time;

        // If there is no transfer ongoing, do nothing.
        if(!tfer->t_on){
                return MDV_RESULT_OK;
        }
        // Transfer data.
        result=tferFunc(instance,tfer);
        if(!MDV_SUCCESSFUL(result)&&
           result!=MDV_SERIALPORT_ERROR_RX_BUFFER_EMPTY&&
           result!=MDV_SERIALPORT_ERROR_TX_BUFFER_FULL){
                // Something went wrong.
                serialport_complete_transfer(tfer,result);
                return result;
        }
        // Check the length of data to be transferred.
        if(!tfer->tleft){
                // All data transferred. Complete the transfer with successful
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
                mdv_timer_start(&(tfer->tmr));
                return MDV_RESULT_OK;
        }
        // Check the timeout.
        result=mdv_timer_get_time(&(tfer->tmr),MDV_TIMER_OM_MS,&time);
        if(!MDV_SUCCESSFUL(result)){
                return result;
        }
        if(time>tfer->tout){
                result=MDV_ERROR_TIMEOUT;
                serialport_complete_transfer(tfer,result);
                return result;
        }
        // Nothing happened, but transfer is still in progress.
        return MDV_RESULT_OK;
}

/*-------------------------------------------------------------------------*//**
**  @brief Performs a synchronous transfer.
**
**  @param[in] instance A pointer to a driver instance.
**  @param[in] tferFunc Driver Read/Write operation.
**  @param[in] tfer A pointer to a transfer descriptor.
**
**  @retval MDV_RESULT_OK Successful.
**  @retval MDV_ERROR_TIMEOUT Timeout.
*/
static MdvResult_t
serialport_synchronous_transfer(
        MdvDriverInstance_t *const instance,
        MdvSerialPortDriverTransferFunction_t tferFunc,
        MdvSerialPortTransfer_t *const tfer
)
{
        MdvResult_t result;

        // Loop until the transfer completed.
        while(tfer->t_on){
                // Use the asynchronous transfer function to transfer data
                // and manage timeouts.
                result=serialport_asynchronous_transfer(instance,tferFunc,tfer);
                if(!MDV_SUCCESSFUL(result)){
                        return result;
                }
        }
        return MDV_RESULT_OK;
}

/*-------------------------------------------------------------------------*//**
**  @brief Transfers data to/from serial port.
**
**  @param[in] instance A pointer to a driver instance.
**  @param[in] tferFunc Driver Read/Write function.
**  @param[in] tfer A pointer to a transfer descriptor.
**  @param[in] length Data length;
**  @param[in] data A pointer to data.
**  @param[in] plen A pointer to transferred data length.
**  @param[in] timeout Timeout time in milliseconds.
**
**  @retval MDV_RESULT_OK Successful.
**  @retval MDV_ERROR_TIMEOUT Timeout.
*/
static MdvResult_t
serialport_transfer(
        MdvDriverInstance_t *const instance,
        MdvSerialPortDriverTransferFunction_t tferFunc,
        MdvSerialPortTransfer_t *const tfer,
        uint32_t length,
        uint8_t *const data,
        uint32_t *const plen,
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
                return serialport_synchronous_transfer(
                        instance,
                        tferFunc,
                        tfer
                );
        }
        return serialport_asynchronous_transfer(
                instance,
                tferFunc,
                tfer
        );
}

/******************************************************************************\
**
**  API FUNCTION DEFINITIONS
**
\******************************************************************************/

MdvResult_t
mdv_serialport_setup_driver_interface(
        MdvSerialPort_t *const port,
        MdvSerialPortDriverTransferFunction_t funcRead,
        MdvSerialPortDriverTransferFunction_t funcWrite
)
{
        if(!port){
                return MDV_ERROR_INVALID_POINTER;
        }
        if(!funcRead||!funcWrite){
                return MDV_ERROR_INVALID_PARAMETER;
        }
        port->drv.specific.func.read=funcRead;
        port->drv.specific.func.write=funcWrite;
        port->drv.specific.initialized=true;
        return MDV_RESULT_OK;
}

MdvResult_t
mdv_serialport_init(
        MdvSerialPort_t *const port,
        MdvSerialPortTransferCompletedCallback_t rxCallback,
        MdvSerialPortTransferCompletedCallback_t txCallback,
        MdvTimerSystem_t *const tsys,
        void *const userData
)
{
        if(!port){
                return MDV_ERROR_INVALID_POINTER;
        }
        if(!port->drv.specific.initialized){
                return MDV_ERROR_INITIALIZATION_FAILED;
        }
        // Initialize the rx transfer descriptor.
        memset(&port->rxd,0,sizeof(MdvSerialPortTransfer_t));
        mdv_timer_init(&(port->rxd.tmr),tsys);
        port->rxd.cbk=rxCallback;
        port->rxd.ud=userData;
        // Initialize the tx transfer descriptor.
        memset(&port->txd,0,sizeof(MdvSerialPortTransfer_t));
        mdv_timer_init(&(port->txd.tmr),tsys);
        port->txd.cbk=txCallback;
        port->txd.ud=userData;
        // Set the port initialization status.
        port->initialized=true;
        // Set default configuration.
        port->cfg=serialPortDefaultConfig;
        // Initialize the driver.
        return mdv_driver_safe_init(&(port->drv.common));
}

MdvResult_t
mdv_serialport_get_current_configuration(
        MdvSerialPort_t *const port,
        MdvSerialPortConfig_t *const config
)
{
        if(!port||!config){
                return MDV_ERROR_INVALID_POINTER;
        }
        // Configuration data output.
        *config=port->cfg;
        return MDV_RESULT_OK;
}

MdvResult_t
mdv_serialport_open(
        MdvSerialPort_t *const port,
        MdvSerialPortConfig_t *const config,
        MdvHandle_t *const handle
)
{
        MdvResult_t result;

        if(!port||!config||!handle){
                return MDV_ERROR_INVALID_POINTER;
        }
        if(!port->drv.specific.initialized){
                return MDV_ERROR_INITIALIZATION_FAILED;
        }
        // Check the handle (it should be invalid).
        if(mdv_handle_is_valid(handle)){
                return MDV_ERROR_RESOURCE_IN_USE;
        }
        // Check the port initialization status.
        if(!port->initialized){
                return MDV_SERIALPORT_ERROR_PORT_NOT_INITIALIZED;
        }
        // Configure and open the port.
        port->cfg=*config;
        result=mdv_driver_safe_open(&(port->drv.common));
        if(!MDV_SUCCESSFUL(result)){
                return result;
        }
        // Handle output.
        return mdv_handle_create(handle,port,1);
}

MdvResult_t
mdv_serialport_close(
        MdvHandle_t *const handle
)
{
        MdvResult_t result;
        MdvSerialPort_t *port;

        if(!handle){
                return MDV_ERROR_INVALID_POINTER;
        }
        result=mdv_handle_request_object(handle,(void*)&port);
        if(!MDV_SUCCESSFUL(result)){
                return result;
        }
        // Cancel possibly ongoing asynchronous transfers.
        serialport_cancel_transfer(&port->rxd);
        serialport_cancel_transfer(&port->txd);
        // Close the port.
        mdv_driver_safe_close(&(port->drv.common));
        mdv_handle_release_object(handle,(void*)&port);
        return mdv_handle_destroy(handle);
}

MdvResult_t
mdv_serialport_change_configuration(
        MdvHandle_t *const handle,
        MdvSerialPortConfig_t *const config
)
{
        MdvResult_t result;
        MdvSerialPort_t *port;

        if(!config||!handle){
                return MDV_ERROR_INVALID_POINTER;
        }
        result=mdv_handle_request_object(handle,(void*)&port);
        if(!MDV_SUCCESSFUL(result)){
                return result;
        }
        // Cancel possibly ongoing asynchronous transfers.
        serialport_cancel_transfer(&port->rxd);
        serialport_cancel_transfer(&port->txd);
        // Close the port.
        result=mdv_driver_safe_close(&(port->drv.common));
        if(!MDV_SUCCESSFUL(result)){
                mdv_handle_release_object(handle,(void*)&port);
                return result;
        }
        // Port re-configuration and re-opening.
        port->cfg=*config;
        result=mdv_driver_safe_open(&(port->drv.common));
        if(!MDV_SUCCESSFUL(result)){
                mdv_handle_release_object(handle,(void*)&port);
                return result;
        }
        mdv_handle_release_object(handle,(void*)&port);
        return MDV_RESULT_OK;
}

MdvResult_t
mdv_serialport_read(
        MdvHandle_t *const handle,
        uint32_t length,
        uint8_t *const data,
        uint32_t *const bytesRead,
        uint32_t timeout
)
{
        MdvResult_t result;
        MdvSerialPort_t *port;

        if(!handle||!data){
                return MDV_ERROR_INVALID_POINTER;
        }
        result=mdv_handle_request_object(handle,(void*)&port);
        if(!MDV_SUCCESSFUL(result)){
                return result;
        }
        // Start transfer.
        result=serialport_transfer(
                port->drv.common.instance,
                port->drv.specific.func.read,
                &port->rxd,
                length,
                data,
                bytesRead,
                timeout
        );
        mdv_handle_release_object(handle,(void*)&port);
        return result;
}

MdvResult_t
mdv_serialport_write(
        MdvHandle_t *const handle,
        uint32_t length,
        uint8_t *const data,
        uint32_t *const bytesWritten,
        uint32_t timeout
)
{
        MdvResult_t result;
        MdvSerialPort_t *port;

        if(!handle||!data){
                return MDV_ERROR_INVALID_POINTER;
        }
        result=mdv_handle_request_object(handle,(void*)&port);
        if(!MDV_SUCCESSFUL(result)){
                return result;
        }
        // Start transfer.
        result=serialport_transfer(
                port->drv.common.instance,
                port->drv.specific.func.write,
                &port->txd,
                length,
                data,
                bytesWritten,
                timeout
        );
        mdv_handle_release_object(handle,(void*)&port);
        return result;
}

MdvResult_t
mdv_serialport_getchar(
        MdvHandle_t *handle,
        uint8_t *const data
)
{
        return mdv_serialport_read(handle,1,data,0,0);
}

MdvResult_t
mdv_serialport_putchar(
        MdvHandle_t *const handle,
        uint8_t data
)
{
        return mdv_serialport_write(handle,1,&data,0,0);
}

MdvResult_t
mdv_serialport_runtime_process(
        MdvHandle_t *const handle
)
{
        MdvResult_t result;
        MdvSerialPort_t *port;

        if(!handle){
                return MDV_ERROR_INVALID_POINTER;
        }
        result=mdv_handle_request_object(handle,(void*)&port);
        if(!MDV_SUCCESSFUL(result)){
                return result;
        }
        result=mdv_driver_safe_run(&(port->drv.common));
        if(!MDV_SUCCESSFUL(result)){
                mdv_handle_release_object(handle,(void*)&port);
                return result;
        }
        // Perform asynchronous transfers.
        serialport_asynchronous_transfer(
                port->drv.common.instance,
                port->drv.specific.func.read,
                &port->rxd
        );
        serialport_asynchronous_transfer(
                port->drv.common.instance,
                port->drv.specific.func.write,
                &port->txd
        );
        mdv_handle_release_object(handle,(void*)&port);
        return MDV_RESULT_OK;
}

/* EOF */