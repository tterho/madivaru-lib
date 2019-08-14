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
        MdvSpi_t *spi,
        MdvSpiDriverInterface_SelectSlave_t funcSelectSlave,
        MdvSpiDriverInterface_TransferByte_t funcTransferByte,
        MdvSpiDriverInterface_Transfer_t funcTransfer
)
{
        if(!spi){
                return MDV_ERROR_INVALID_POINTER;
        }
        spi->drv.funcSelectSlave=funcSelectSlave;
        spi->drv.funcTransferByte=funcTransferByte;
        spi->drv.funcTransfer=funcTransfer;
        return MDV_RESULT_OK;
}

MdvResult_t
mdv_spi_open(
        MdvSpi_t *spi,
        MdvSpiConfig_t *config,
        MdvSpiTransferCompletedCallback_t callback,
        void *userData,
        MdvHandle_t *handle
)
{
        MdvResult_t result;

        if(!spi||!config||!handle){
                return MDV_ERROR_INVALID_POINTER;
        }
        if(!spi->drv.essentials.initialized){
                return MDV_ERROR_DRIVER_INTERFACE;
        }
        if(*handle){
                return MDV_ERROR_INVALID_PARAMETER;
        }
        // Initialize the spi.
        result=spi->drv.essentials.funcInit(spi->drv.essentials.instance);
        if(!MDV_SUCCESSFUL(result)){
                return MDV_ERROR_INITIALIZATION_FAILED;
        }
        // Store the port configuration.
        spi->cfg=*config;
        // Set the transfer completion callback and user data.
        spi->cbk=callback;
        spi->ud=userData;
        // Setup the port hardware and open the port.
        result=spi->drv.essentials.funcWakeup(spi->drv.essentials.instance);
        if(!MDV_SUCCESSFUL(result)){
                return result;
        }
        result=spi->drv.essentials.funcOpen(spi->drv.essentials.instance);
        if(!MDV_SUCCESSFUL(result)){
                spi->drv.essentials.funcSleep(spi->drv.essentials.instance);
                return result;
        }
        // Set the handle to point to the spi.
        *handle=(MdvHandle_t)spi;
        return MDV_RESULT_OK;
}

MdvResult_t
mdv_spi_close(
        MdvHandle_t *handle
)
{
        MdvSpi_t *spi;

        if(!handle){
                return MDV_ERROR_INVALID_POINTER;
        }
        if(!*handle){
                return MDV_ERROR_INVALID_PARAMETER;
        }
        // Get an access to the driver.
        spi=(MdvSpi_t*)*handle;
        // Close the port.
        spi->drv.essentials.funcClose(spi->drv.essentials.instance);
        spi->drv.essentials.funcSleep(spi->drv.essentials.instance);
        // Set the handle to null.
        *handle=(MdvHandle_t)0;
        return MDV_RESULT_OK;
}

MdvResult_t
mdv_spi_select_slave(
        MdvHandle_t handle,
        uint8_t slaveAddress
)
{
        MdvSpi_t *spi;
        MdvResult_t result;

        if(!handle){
                return MDV_ERROR_INVALID_PARAMETER;
        }
        // Get an access to the driver.
        spi=(MdvSpi_t*)handle;
        // Check the port configuration. The operation is not allowed for
        // a port configured as a slave.
        if(spi->cfg.operatingMode==MDV_SPI_OPERATING_MODE_SLAVE){
                return MDV_ERROR_INVALID_OPERATION;
        }
        // Select the slave.
        result=spi->drv.funcSelectSlave(
                spi->drv.essentials.instance,
                slaveAddress
        );
        if(!MDV_SUCCESSFUL(result)){
                return MDV_ERROR_INVALID_PARAMETER;
        }
        return MDV_RESULT_OK;
}

MdvResult_t
mdv_spi_transfer(
        MdvHandle_t handle,
        uint8_t *dout,
        uint16_t outsz,
        uint8_t *din,
        uint16_t insz
)
{
        MdvSpi_t *spi;
        uint8_t tmp=0;
        MdvResult_t result;

        if(!dout||!din){
                return MDV_ERROR_INVALID_POINTER;
        }
        if(!handle){
                return MDV_ERROR_INVALID_PARAMETER;
        }
        // Get an access to the driver.
        spi=(MdvSpi_t*)handle;
        // Check if the driver is capable of to transfer all data at once.
        if(spi->drv.funcTransfer){
                // Transfer data.
                result=spi->drv.funcTransfer(
                        spi->drv.essentials.instance,
                        dout,
                        outsz,
                        din,
                        insz
                );
                // If the driver doesn't support inequal buffer sizes, it
                // returns this error code.
                if(result==MDV_SPI_ERROR_INVALID_BUFFER_SIZE){
                        return result;
                }
                // Check other possible driver errors.
                if(!MDV_SUCCESSFUL(result)){
                        return MDV_SPI_ERROR_TRANSMISSION_FAILED;
                }
                return MDV_RESULT_OK;
        }
        // Both Transfer and TransferByte implementations are missing. This is
        // an interface initialization error.
        if(!spi->drv.funcTransferByte){
                return MDV_ERROR_DRIVER_INTERFACE;
        }
        // Transfer all bi-directional data.
        while(outsz&&insz){
                result=spi->drv.funcTransferByte(
                        spi->drv.essentials.instance,
                        *dout,
                        din
                );
                if(!MDV_SUCCESSFUL(result)){
                        return MDV_SPI_ERROR_TRANSMISSION_FAILED;
                }
                dout++;
                din++;
                outsz--;
                insz--;
        }
        // Transfer remaining output data (if any).
        while(outsz){
                result=spi->drv.funcTransferByte(
                        spi->drv.essentials.instance,
                        *dout,
                        &tmp
                );
                if(!MDV_SUCCESSFUL(result)){
                        return MDV_SPI_ERROR_TRANSMISSION_FAILED;
                }
                dout++;
                outsz--;
        }
        // Transfer remaining input data (if any).
        while(insz){
                result=spi->drv.funcTransferByte(
                        spi->drv.essentials.instance,
                        tmp,
                        din
                );
                if(!MDV_SUCCESSFUL(result)){
                        return MDV_SPI_ERROR_TRANSMISSION_FAILED;
                }
                din++;
                insz--;
        }
        return MDV_RESULT_OK;
}

/* EOF */