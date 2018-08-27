/***************************************************************************//**
**
**  @file       spi_api.c
**  @ingroup    serialcomm
**  @brief      SPI communication API.
**  @copyright  Copyright (C) 2012-2018 Tuomas Terho. All rights reserved.
**
**  Common synchronous serial port interface which can be easily ported for 
**  different platforms without need to change the control interface.
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

#include "spi_api.h"

/*------------------------------------------------------------------------------
**  Opens a SPI port.
*/
Result_t
SPI_Open(
        SPIDrv_t *driver,
        SPI_Config_t *config,
        Handle_t *handle
)
{
        if(!driver||!config||!handle){
                return SPI_ERROR_INVALID_POINTER;
        }
        if(*handle){
                return SPI_ERROR_INVALID_PARAMETER;
        }
        // Initialize the driver.
        if(!SUCCESSFUL(driver->Init(config))){
                return SPI_ERROR_DRIVER_INITIALIZATION_FAILED;
        }
        // Store the port configuration.
        driver->cfg=*config;
        // Open the port.
        driver->Open();
        // Set the handle to point to the driver.
        *handle=(Handle_t)driver;
        return RESULT_OK;
}

/*------------------------------------------------------------------------------
**  Closes a SPI port.
*/
Result_t
SPI_Close(
        Handle_t *handle
)
{
        SPIDrv_t *drv;
        
        if(!handle){
                return SPI_ERROR_INVALID_POINTER;
        }
        if(!*handle){
                return SPI_ERROR_INVALID_PARAMETER;
        }
        // Get an access to the driver.
        drv=(SPIDrv_t*)*handle;
        // Close the port.
        drv->Close();
        // Set the handle to null.
        *handle=(Handle_t)0;
        return RESULT_OK;
}

/*------------------------------------------------------------------------------
**  Selects a slave for communication.
*/
Result_t
SPI_SelectSlave(
        Handle_t handle,
        uint8_t slaveAddress
)
{
        SPIDrv_t *drv;
        
        if(!handle){
                return SPI_ERROR_INVALID_PARAMETER;
        }
        // Get an access to the driver.
        drv=(SPIDrv_t*)handle;
        // Check the port configuration. The operation is not allowed for 
        // a port configured as a slave.
        if(drv->cfg.OperatingMode==SPI_OM_SLAVE){
                return SPI_ERROR_INVALID_OPERATION;
        }
        // Select the slave.
        if(!SUCCESSFUL(drv->SelectSlave(slaveAddress))){
                return SPI_ERROR_INVALID_PARAMETER;
        }
        return RESULT_OK;
}

/*------------------------------------------------------------------------------
**  Transmits data in both directions.
*/
Result_t
SPI_Transmit(
        Handle_t handle,
        uint8_t *dout,
        uint16_t outsz,
        uint8_t *din,
        uint16_t insz
)
{
        SPIDrv_t *drv;
        uint8_t tmp=0;
        Result_t result;
        
        if(!dout||!din){
                return SPI_ERROR_INVALID_POINTER;
        }
        if(!handle){
                return SPI_ERROR_INVALID_PARAMETER;
        }
        // Get an access to the driver.
        drv=(SPIDrv_t*)handle;
        // Check if the driver is capable of to transmit all data at once.
        if(drv->Transmit){
                // Transmit data.
                result=drv->Transmit(dout,outsz,din,insz);
                // If the driver doesn't support inequal buffer sizes, it 
                // returns this error code.
                if(result==SPI_ERROR_INVALID_BUFFER_SIZE){
                        return result;
                }
                // Check other possible driver errors.
                if(!SUCCESSFUL(result)){
                        return SPI_ERROR_TRANSMISSION_FAILED;
                }
                return RESULT_OK;
        }        
        // Both Transmit and TransmitByte implementations are missing. This is 
        // an initialization error.
        if(!drv->TransmitByte){
                return SPI_ERROR_DRIVER_INITIALIZATION_FAILED;
        }
        // Transmit all bi-directional data.
        while(outsz&&insz){
                if(!SUCCESSFUL(drv->TransmitByte(*dout,din))){
                        return SPI_ERROR_TRANSMISSION_FAILED;
                }
                dout++;
                din++;
                outsz--;
                insz--;
        }
        // Transmit remaining output data (if any).
        while(outsz){
                if(!SUCCESSFUL(drv->TransmitByte(*dout,&tmp))){
                        return SPI_ERROR_TRANSMISSION_FAILED;
                }
                dout++;
                outsz--;
        }
        // Transmit remaining input data (if any).
        while(insz){
                if(!SUCCESSFUL(drv->TransmitByte(tmp,din))){
                        return SPI_ERROR_TRANSMISSION_FAILED;
                }
                din++;
                insz--;
        }
        return RESULT_OK;
}

/* EOF */