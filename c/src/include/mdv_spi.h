/***************************************************************************//**
**
**  @file       mdv_spi.h
**  @ingroup    madivaru-lib
**  @brief      SPI communication API.
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

#ifndef mdv_spi_H
#define mdv_spi_H

#include "mdv_types.h"

/******************************************************************************\
**
**  ERROR CODES
**
\******************************************************************************/

/// @brief Invalid pointer.
///
/// At least one of the pointer parameters is invalid (points to null).
#define MDV_SPI_ERROR_INVALID_POINTER -1

/// @brief Invalid pointer.
///
/// At least one of the parameter values is invalid.
#define MDV_SPI_ERROR_INVALID_PARAMETER -2

/// @brief Driver initialization failed.
///
/// Something went wrong during the driver initialization. Check the driver
/// configuration.
#define MDV_SPI_ERROR_DRIVER_INITIALIZATION_FAILED -3

/// @brief Transmission error.
///
/// Something went wrong during the data transmission.
#define MDV_SPI_ERROR_TRANSMISSION_FAILED -4

/// @brief Invalid operation.
///
/// The operation is invalid. Check the port configuration.
#define MDV_SPI_ERROR_INVALID_OPERATION -5

/// @brief Invalid buffer size.
///
/// The driver supports only equal size Rx and Tx buffers.
#define MDV_SPI_ERROR_INVALID_BUFFER_SIZE -6

/******************************************************************************\
**
**  TYPE DEFINITIONS
**
\******************************************************************************/

/*-------------------------------------------------------------------------*//**
**  @brief SPI operating modes.
*/
typedef enum
MdvSpiOperatingMode_t{
        /// The SPI device operates in Master mode (default).
        MDV_SPI_OPERATING_MODE_MASTER=0,
        /// The SPI device operates in Slave mode.
        MDV_SPI_OPERATING_MODE_SLAVE,
} MdvSpiOperatingMode_t;

/*-------------------------------------------------------------------------*//**
**  @brief SPI configuration structure.
*/
typedef struct
MdvSpiConfig_t{
        /// @brief Operating mode of the SPI device.
        MdvSpiOperatingMode_t operatingMode;
        union{
                /// @brief Configuration bits.
                uint8_t bits;
                struct{
                        /// @brief Clock phase option (default = 0).
                        ///
                        /// Selects the clock phase mode:
                        /// 0 = Output changes the data on the trailing edge and 
                        /// the input captures the data on the leading edge of
                        /// the clock signal.
                        /// 1 = Output changes the data on the leading edge and
                        /// the input captures the data on the trailing edge of
                        /// the clock signal.
                        uint8_t CPHA:1;
                        /// @brief Clock polarity option (default = 0).
                        ///
                        /// Selects the clock polarity mode:
                        /// 0 = Active high polarity. The clock signal is low 
                        /// during idle (default). The rising edge is the 
                        /// leading edge of the clock pulse.
                        /// 1 = Active low polarity. The clock signal is high 
                        /// during idle. The falling edge is the leading edge of 
                        /// the clock pulse.
                        uint8_t CPOL:1;
                        /// @brief LSB first option (default = 0).
                        ///
                        /// Selects whether the MSB or LSB bit is transmitted
                        /// first to the SPI bus.
                        /// 0 = MSB is transmitted first (default).
                        /// 1 = LSB is transmitted first.
                        uint8_t LSBFE:1;
                };
        };
        /// @brief Clock speed.
        ///
        /// Specifies the transfer rate of the SPI communication. The nearest
        /// possible value is used depending on the hardware capability.
        uint32_t clkSpeed;
} MdvSpiConfig_t;

/*-------------------------------------------------------------------------*//**
**  @brief Transfer completion callback.
**
**  @param[in] userData A pointer to user specified data to be passed to the
**      callback handler.
**
**  This callback is invoked by the driver on the completion of an asynchronous
**  transfer.
*/
typedef void
(*MdvSpiTransferCompletedCallback_t)(
        void *userData
);

/******************************************************************************\
**
**  DRIVER INTERFACE FUNCTION TYPES
**
\******************************************************************************/

/*-------------------------------------------------------------------------*//**
**  @brief Initializes a SPI driver.
**
**  @param[in] config SPI port configuration.
**
**  @retval MDV_RESULT_OK Initialization successful.
**  @retval MDV_SPI_ERROR_INVALID_POINTER The config parameter points to null.
**  @return Negative value: a driver specific error code. See the driver
**      implementation.
*/
typedef
MdvResult_t
(*MdvSpiDriverInterface_Init_t)(
        MdvSpiConfig_t *config
);

/*-------------------------------------------------------------------------*//**
**  @brief Opens a SPI port.
**
**  @return No return value.
*/
typedef
void
(*MdvSpiDriverInterface_Open_t)(
        void
);

/*-------------------------------------------------------------------------*//**
**  @brief Closes a SPI port.
**
**  @return No return value.
*/
typedef
void
(*MdvSpiDriverInterface_Close_t)(
        void
);

/*-------------------------------------------------------------------------*//**
**  @brief Selects a slave for communication.
**
**  @param[in] slaveAddress The address of the slave to select.
**
**  @retval MDV_RESULT_OK Slave selected successfully.
**  @retval MDV_SPI_ERROR_INVALID_PARAMETER Slave address out of range.
*/
typedef
MdvResult_t
(*MdvSpiDriverInterface_SelectSlave_t)(
        uint8_t slaveAddress
);

/*-------------------------------------------------------------------------*//**
**  @brief Transfers one byte of data in both directions.
**
**  @param[in] dout Data to send. If there is no data to send out, set to zero.
**  @param[out] din Data to receive. If there is no data to receive, set to
**      null.
**
**  @retval MDV_RESULT_OK Transmission successful.
**
**  @remarks Implement this function if the driver is capable of to transmit one
**  byte at the time. Otherwise, implement the function
**  @ref MdvSpiDriverInterface_Transfer_t.
*/
typedef
MdvResult_t
(*MdvSpiDriverInterface_TransferByte_t)(
        uint8_t dout,
        uint8_t *din
);

/*-------------------------------------------------------------------------*//**
**  @brief Transfers data in both directions.
**
**  @param[in] dout Data to send. If there is no data to send out, set to null.
**  @param[in] outsz Size of the output data.
**  @param[out] din Data to receive. If there is no data to receive, set to
**      null.
**  @param[in] dinsz Size of the input data.
**
**  @retval MDV_RESULT_OK Transmission successful.
**
**  @remarks Implement this function if the driver is capable of to transmit
**  multiple bytes at the time. Otherwise, implement the function
**  @ref MdvSpiDriverInterface_TransferByte_t.
*/
typedef
MdvResult_t
(*MdvSpiDriverInterface_Transfer_t)(
        uint8_t *dout,
        uint16_t outsz,
        uint8_t *din,
        uint16_t insz
);

/******************************************************************************\
**
**  DRIVER INTERFACE DEFINITION
**
\******************************************************************************/

/*-------------------------------------------------------------------------*//**
**  @brief SPI driver interface structure.
*/
typedef struct
MdvSpiDriverInterface_t{
        /// Initializes the driver.
        MdvSpiDriverInterface_Init_t funcInit;
        /// Opens the driver.
        MdvSpiDriverInterface_Open_t funcOpen;
        /// Closes the driver.
        MdvSpiDriverInterface_Close_t funcClose;
        /// Selects a slave for communication.
        MdvSpiDriverInterface_SelectSlave_t funcSelectSlave;
        /// Transfers one byte in both directions.
        MdvSpiDriverInterface_TransferByte_t funcTransferByte;
        /// Transfers data in both directions.
        MdvSpiDriverInterface_Transfer_t funcTransfer;
        /// Initialization status of the interface.
        bool init;
} MdvSpiDriverInterface_t;

#ifdef __cplusplus
extern "C"{
#endif // ifdef __cplusplus

/******************************************************************************\
**
**  SPI DEFINITION
**
\******************************************************************************/

/*-------------------------------------------------------------------------*//**
**  @brief SPI structure.
*/
typedef struct
MdvSpi_t{
        /// Pointer to a SPI driver interface.
        MdvSpiDriverInterface_t drv;
        /// Stores the port configuration.
        MdvSpiConfig_t cfg;
        /// Transfer completion callback.
        MdvSpiTransferCompletedCallback_t cbk;
        /// A pointer to user specified data.
        void *ud;
} MdvSpi_t;

/******************************************************************************\
**
**  API FUNCTION DECLARATIONS
**
\******************************************************************************/

/*-------------------------------------------------------------------------*//**
**  @brief Sets up the driver interface.
**
**  @param[out] spi SPI port which driver interface to set up.
**  @param[in] funcInit (Mandatory) A function to initialize the port hardware.
**  @param[in] funcOpen (Mandatory) A function to open the port.
**  @param[in] funcClose (Mandatory) A function to close the port.
**  @param[in] funcSelectSlave (Optional) A function to select a SPI slave.
**  @param[in] funcTransferByte (Optional) A function to transfer one byte of
**      data in both directions.
**  @param[in] funcTransfer (Optional) A function to transfer multiple bytes of
**      data in both directions.
**
**  @retval MDV_RESULT_OK Successful.
**  @retval MDV_SPI_ERROR_INVALID_POINTER The port parameter points to null.
**  @retval MDV_SPI_ERROR_INVALID_PARAMETER At least one of the mandatory
**      function pointers is null.
*/
MdvResult_t
mdv_spi_setup_driver_interface(
        MdvSpi_t *spi,
        MdvSpiDriverInterface_Init_t funcInit,
        MdvSpiDriverInterface_Open_t funcOpen,
        MdvSpiDriverInterface_Close_t funcClose,
        MdvSpiDriverInterface_SelectSlave_t funcSelectSlave,
        MdvSpiDriverInterface_TransferByte_t funcTransferByte,
        MdvSpiDriverInterface_Transfer_t funcTransfer
);

/*-------------------------------------------------------------------------*//**
**  @brief Opens a SPI port.
**
**  @param[in] spi SPI port to open.
**  @param[in] config SPI port configuration.
**  @param[in] callback Callback for asynchronous transfer completed events.
**      This parameter is optional and can be set to null.
**  @param[in] userData A pointer to optiona user specified data to be passed to
**      the transfer completed callback handler.
**  @param[out] handle Handle to the opened port.
**
**  @retval MDV_RESULT_OK Port opened successfully.
**  @retval MDV_SPI_ERROR_INVALID_POINTER Either the driver, config or handle 
**      parameter points to null.
**  @retval MDV_SPI_ERROR_INVALID_PARAMETER The handle is not null.
**  @retval MDV_SPI_ERROR_DRIVER_INITIALIZATION_FAILED Driver initialization 
**      failed.
*/
MdvResult_t
mdv_spi_open(
        MdvSpi_t *spi,
        MdvSpiConfig_t *config,
        MdvSpiTransferCompletedCallback_t callback,
        void *userData,
        MdvHandle_t *handle
);

/*-------------------------------------------------------------------------*//**
**  @brief Closes a SPI port.
**
**  @param[in] handle A handle to the port to be closed.
**
**  @retval MDV_RESULT_OK Port closed successfully.
**  @retval MDV_SPI_ERROR_INVALID_POINTER The handle parameter points to null.
**  @retval MDV_SPI_ERROR_INVALID_PARAMETER The handle is invalid (null).
*/
MdvResult_t
mdv_spi_close(
        MdvHandle_t *handle
);

/*-------------------------------------------------------------------------*//**
**  @brief Selects a slave for communication.
**
**  @param[in] handle A handle to the port.
**  @param[in] slaveAddress The address of the slave to select.
**
**  @retval MDV_RESULT_OK Slave selected successfully.
**  @retval MDV_SPI_ERROR_INVALID_PARAMETER The handle is invalid (null), or the 
**      slave address is out of range.
**  @retval MDV_SPI_ERROR_INVALID_OPERATION The slave selection works only on 
**      master device. Check the port configuration.
*/
MdvResult_t
mdv_spi_select_slave(
        MdvHandle_t handle,
        uint8_t slaveAddress
);                

/*-------------------------------------------------------------------------*//**
**  @brief Transfers data in both directions.
**
**  @param[in] handle Handle to a port.
**  @param[in] dout Data to be sent out.
**  @param[in] outsz Output data length in bytes.
**  @param[out] din Data to be received.
**  @param[in] insz Input data length in bytes.
**
**  @retval MDV_RESULT_OK Initialization successful.
**  @retval MDV_SPI_ERROR_INVALID_POINTER The config parameter points to null.
**  @retval MDV_SPI_ERROR_DRIVER_INITIALIZATION_FAILED Driver initialization 
**      failed.
**  @retval MDV_SPI_ERROR_TRANSMISSION_FAILED Transmission failed for some 
**      reason.
**  @retval MDV_SPI_ERROR_INVALID_BUFFER_SIZE The driver doesn't support inequal
**      buffer sizes.
**
**  Transfers equal amount of data in both directions. If the transmission 
**  buffer size is smaller than the reception buffer size, the function
**  transmits nulls until all input data has been received. If the reception
**  buffer size is smaller than the transmission buffer size, all bytes are
**  transmitted and the received bytes beyond the input buffer size are lost.
**
**  @remarks An underlying driver doesn't necessarily support inequal buffer 
**  sizes. If the buffer sizes don't match, in that case the driver returns
**  an error.
*/
MdvResult_t
mdv_spi_transfer(
        MdvHandle_t handle,
        uint8_t *dout,
        uint16_t outsz,
        uint8_t *din,
        uint16_t insz
);

#ifdef __cplusplus
}
#endif // ifdef __cplusplus

#endif // ifndef mdv_spi_H

/* EOF */