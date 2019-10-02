/***************************************************************************//**
**
**  @file       mdv_spi.c
**  @ingroup    madivaru-lib
**  @brief      SPI communication API.
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

#include "mdv_spi.h"

/******************************************************************************\
**
**  API FUNCTION DEFINITIONS
**
\******************************************************************************/

MdvResult_t
mdv_spi_setup_driver_interface(
        MdvSpi_t *const spi,
        MdvSpiDriverSelectSlave_t funcSelectSlave,
        MdvSpiDriverTransferByte_t funcTransferByte,
        MdvSpiDriverTransfer_t funcTransfer
)
{
        if(!spi){
                return MDV_ERROR_INVALID_POINTER;
        }
        spi->drv.specific.func.selectSlave=funcSelectSlave;
        spi->drv.specific.func.transferByte=funcTransferByte;
        spi->drv.specific.func.transfer=funcTransfer;
        spi->drv.specific.initialized=true;
        return MDV_RESULT_OK;
}

MdvResult_t
mdv_spi_open(
        MdvSpi_t *const spi,
        MdvSpiConfig_t *const config,
        MdvSpiTransferCompletedCallback_t callback,
        void *const userData,
        MdvHandle_t *const handle
)
{
        MdvResult_t result;

        if(!spi||!config||!handle){
                return MDV_ERROR_INVALID_POINTER;
        }
        if(!spi->drv.specific.initialized){
                return MDV_ERROR_DRIVER_INTERFACE;
        }
        // Check the handle (it should be invalid).
        if(mdv_handle_is_valid(handle)){
                return MDV_ERROR_RESOURCE_IN_USE;
        }
        // Initialize the spi.
        result=mdv_driver_safe_init(&(spi->drv.common));
        if(!MDV_SUCCESSFUL(result)){
                return result;
        }
        // Store the port configuration.
        spi->cfg=*config;
        // Set the transfer completion callback and user data.
        spi->cbk=callback;
        spi->ud=userData;
        // Open the port.
        result=mdv_driver_safe_open(&(spi->drv.common));
        if(!MDV_SUCCESSFUL(result)){
                return result;
        }
        // Handle output.
        return mdv_handle_create(handle,spi,1);
}

MdvResult_t
mdv_spi_close(
        MdvHandle_t *const handle
)
{
        MdvResult_t result;
        MdvSpi_t *spi;

        if(!handle){
                return MDV_ERROR_INVALID_POINTER;
        }
        result=mdv_handle_request_object(handle,(void*)&spi);
        if(!MDV_SUCCESSFUL(result)){
                return result;
        }
        mdv_driver_safe_close(&(spi->drv.common));
        mdv_handle_release_object(handle,(void*)&spi);
        return mdv_handle_destroy(handle);
}

MdvResult_t
mdv_spi_select_slave(
        MdvHandle_t *const handle,
        uint8_t slaveAddress
)
{
        MdvSpi_t *spi;
        MdvResult_t result;

        if(!handle){
                return MDV_ERROR_INVALID_POINTER;
        }
        result=mdv_handle_request_object(handle,(void*)&spi);
        if(!MDV_SUCCESSFUL(result)){
                return result;
        }
        // Check the port configuration. The operation is not allowed for
        // a port configured as a slave.
        if(spi->cfg.operatingMode==MDV_SPI_OPERATING_MODE_SLAVE){
                mdv_handle_release_object(handle,(void*)&spi);
                return MDV_ERROR_INVALID_OPERATION;
        }
        // Select the slave.
        result=spi->drv.specific.func.selectSlave(
                spi->drv.common.instance,
                slaveAddress
        );
        if(!MDV_SUCCESSFUL(result)){
                mdv_handle_release_object(handle,(void*)&spi);
                return MDV_ERROR_INVALID_PARAMETER;
        }
        mdv_handle_release_object(handle,(void*)&spi);
        return MDV_RESULT_OK;
}

MdvResult_t
mdv_spi_transfer(
        MdvHandle_t *const handle,
        uint8_t *dout,
        uint16_t outsz,
        uint8_t *din,
        uint16_t insz
)
{
        MdvSpi_t *spi;
        uint8_t tmp=0;
        MdvResult_t result;

        if(!handle||!dout||!din){
                return MDV_ERROR_INVALID_POINTER;
        }
        result=mdv_handle_request_object(handle,(void*)&spi);
        if(!MDV_SUCCESSFUL(result)){
                return result;
        }
        // Check if the driver is capable of to transfer all data at once.
        if(spi->drv.specific.func.transfer){
                // Transfer data.
                result=spi->drv.specific.func.transfer(
                        spi->drv.common.instance,
                        dout,
                        outsz,
                        din,
                        insz
                );
                // If the driver doesn't support inequal buffer sizes, it
                // returns this error code.
                if(result==MDV_SPI_ERROR_INVALID_BUFFER_SIZE){
                        mdv_handle_release_object(handle,(void*)&spi);
                        return result;
                }
                // Check other possible driver errors.
                if(!MDV_SUCCESSFUL(result)){
                        mdv_handle_release_object(handle,(void*)&spi);
                        return MDV_SPI_ERROR_TRANSMISSION_FAILED;
                }
                mdv_handle_release_object(handle,(void*)&spi);
                return MDV_RESULT_OK;
        }
        // Both Transfer and TransferByte implementations are missing. This is
        // an interface initialization error.
        if(!spi->drv.specific.func.transferByte){
                mdv_handle_release_object(handle,(void*)&spi);
                return MDV_ERROR_DRIVER_INTERFACE;
        }
        // Transfer all bi-directional data.
        while(outsz&&insz){
                result=spi->drv.specific.func.transferByte(
                        spi->drv.common.instance,
                        *dout,
                        din
                );
                if(!MDV_SUCCESSFUL(result)){
                        mdv_handle_release_object(handle,(void*)&spi);
                        return MDV_SPI_ERROR_TRANSMISSION_FAILED;
                }
                ++dout;
                ++din;
                --outsz;
                --insz;
        }
        // Transfer remaining output data (if any).
        while(outsz){
                result=spi->drv.specific.func.transferByte(
                        spi->drv.common.instance,
                        *dout,
                        &tmp
                );
                if(!MDV_SUCCESSFUL(result)){
                        mdv_handle_release_object(handle,(void*)&spi);
                        return MDV_SPI_ERROR_TRANSMISSION_FAILED;
                }
                ++dout;
                --outsz;
        }
        // Transfer remaining input data (if any).
        while(insz){
                result=spi->drv.specific.func.transferByte(
                        spi->drv.common.instance,
                        tmp,
                        din
                );
                if(!MDV_SUCCESSFUL(result)){
                        mdv_handle_release_object(handle,(void*)&spi);
                        return MDV_SPI_ERROR_TRANSMISSION_FAILED;
                }
                ++din;
                --insz;
        }
        mdv_handle_release_object(handle,(void*)&spi);
        return MDV_RESULT_OK;
}

/* EOF */