/***************************************************************************//**
**
**  @file       spi_api.h
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

#ifndef spi_api_H
#define spi_api_H

#include "types.h"

/******************************************************************************\
**
**  ERROR CODES
**
\******************************************************************************/

/// @brief Invalid pointer.
///
/// At least one of the pointer parameters is invalid (points to null).
#define SPI_ERROR_INVALID_POINTER -1

/// @brief Invalid pointer.
///
/// At least one of the parameter values is invalid.
#define SPI_ERROR_INVALID_PARAMETER -2

/// @brief Driver initialization failed.
///
/// Something went wrong during the driver initialization. Check the driver
/// configuration.
#define SPI_ERROR_DRIVER_INITIALIZATION_FAILED -3

/// @brief Transmission error.
///
/// Something went wrong during the data transmission.
#define SPI_ERROR_TRANSMISSION_FAILED -4

/// @brief Invalid operation.
///
/// The operation is invalid. Check the port configuration.
#define SPI_ERROR_INVALID_OPERATION -5

/// @brief Invalid buffer size.
///
/// The driver supports only equal size Rx and Tx buffers.
#define SPI_ERROR_INVALID_BUFFER_SIZE -6

/******************************************************************************\
**
**  PORT CONFIGURATION DATA TYPES
**
\******************************************************************************/

/*-------------------------------------------------------------------------*//**
**  @brief SPI operating modes.
*/
typedef enum
SPI_OperatingMode_t{
        /// The SPI device operates in Master mode (default).
        SPI_OM_MASTER=0,
        /// The SPI device operates in Slave mode.
        SPI_OM_SLAVE,
} SPI_OperatingMode_t;

/*-------------------------------------------------------------------------*//**
**  @brief SPI configuration structure.
*/
typedef struct
SPI_Config_t{
        /// @brief Operating mode of the SPI device.
        SPI_OperatingMode_t OperatingMode;
        union{
                /// @brief Configuration bits.
                uint8_t Bits;
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
        uint32_t ClkSpeed;
} SPI_Config_t;

/******************************************************************************\
**
**  DRIVER API FUNCTIONS
**
\******************************************************************************/

/*-------------------------------------------------------------------------*//**
**  @brief Initializes a SPI driver.
**
**  @param[in] config SPI port configuration.
**
**  @retval RESULT_OK Initialization successful.
**  @retval SPI_ERROR_INVALID_POINTER The config parameter points to null.
**  @return Negative value: a driver specific error code. See the driver 
**      implementation.
*/
typedef 
Result_t 
(*SPIDrv_Init_t)(
        SPI_Config_t *config
);

/*-------------------------------------------------------------------------*//**
**  @brief Opens a SPI port.
**
**  @return No return value.
*/
typedef
void
(*SPIDrv_Open_t)(
        void
);                 

/*-------------------------------------------------------------------------*//**
**  @brief Closes a SPI port.
**
**  @return No return value.
*/
typedef
void 
(*SPIDrv_Close_t)(
        void
);

/*-------------------------------------------------------------------------*//**
**  @brief Selects a slave for communication.
**
**  @param[in] slaveAddress The address of the slave to select.
**
**  @retval RESULT_OK Slave selected successfully.
**  @retval SPI_ERROR_INVALID_PARAMETER Slave address out of range.
*/
typedef
Result_t
(*SPIDrv_SelectSlave_t)(
        uint8_t slaveAddress
);

/*-------------------------------------------------------------------------*//**
**  @brief Transfers one byte of data in both directions.
**
**  @param[in] dout Data to send. If there is no data to send out, set to zero.
**  @param[out] din Data to receive. If there is no data to receive, set to 
**      null.
**
**  @retval RESULT_OK Transmission successful.
**
**  @remarks Implement this function if the driver is capable of to transmit one 
**  byte at the time. Otherwise, implement the function @ref SPIDrv_Transfer_t 
**  function.
*/
typedef
Result_t
(*SPIDrv_TransferByte_t)(
        uint8_t dout,
        uint8_t *din
);

/*-------------------------------------------------------------------------*//**
**  @brief Transfers data in both directions.
**
**  @param[in] dout Data to send. If there is no data to send out, set to zero.
**  @param[out] din Data to receive. If there is no data to receive, set to 
**      null.
**
**  @retval RESULT_OK Transmission successful.
**
**  @remarks Implement this function if the driver is capable of to transmit 
**  multiple bytes at the time. Otherwise, implement the function 
**  @ref SPIDrv_TransferByte_t.
*/
typedef
Result_t
(*SPIDrv_Transfer_t)(
        uint8_t *dout,
        uint16_t outsz,
        uint8_t *din,
        uint16_t insz
);

/*-------------------------------------------------------------------------*//**
**  @brief Transfer completion callback.
**
**  This callback is invoked by the driver on the completion of an asynchronous
**  transfer.
*/
typedef void
(*SPI_TransferCompletedCbk_t)(
        void
);

/******************************************************************************\
**
**  DRIVER INTERFACE
**
\******************************************************************************/

/*-------------------------------------------------------------------------*//**
**  @brief SPI driver interface structure.
*/
typedef struct
SPIDrv_t{
        /// Initializes the driver.
        SPIDrv_Init_t Init;
        /// Opens the driver.
        SPIDrv_Open_t Open;
        /// @brief 
        SPIDrv_Close_t Close;
        /// Selects a slave for communication.
        SPIDrv_SelectSlave_t SelectSlave;
        /// Transfers one byte in both directions.
        SPIDrv_TransferByte_t TransferByte;
        /// Transfers data in both directions.
        SPIDrv_Transfer_t Transfer;
        /// Stores the port configuration.
        SPI_Config_t cfg;
        /// Transfer completion callback.
        SPI_TransferCompletedCbk_t cbk;
} SPIDrv_t;

/******************************************************************************\
**
**  SPI API FUNCTIONS
**
\******************************************************************************/

/*-------------------------------------------------------------------------*//**
**  @brief Opens a SPI port.
**
**  @param[in] driver Driver to initialize.
**  @param[in] config SPI port configuration.
**  @param[in] callback Callback for asynchronous transfer completed events.
**      This parameter is optional and can be set to null.
**  @param[out] handle Handle to the opened port.
**
**  @retval RESULT_OK Port opened successfully.
**  @retval SPI_ERROR_INVALID_POINTER Either the driver, config or handle 
**      parameter points to null.
**  @retval SPI_ERROR_INVALID_PARAMETER The handle is not null.
**  @retval SPI_ERROR_DRIVER_INITIALIZATION_FAILED Driver initialization failed.
*/
Result_t
SPI_Open(
        SPIDrv_t *driver,
        SPI_Config_t *config,
        SPI_TransferCompletedCbk_t callback,
        Handle_t *handle
);

/*-------------------------------------------------------------------------*//**
**  @brief Closes a SPI port.
**
**  @param[in] handle A handle to the port to be closed.
**
**  @retval RESULT_OK Port closed successfully.
**  @retval SPI_ERROR_INVALID_POINTER The handle parameter points to null.
**  @retval SPI_ERROR_INVALID_PARAMETER The handle is invalid (null).
*/
Result_t
SPI_Close(
        Handle_t *handle
);

/*-------------------------------------------------------------------------*//**
**  @brief Selects a slave for communication.
**
**  @param[in] handle A handle to the port.
**  @param[in] slaveAddress The address of the slave to select.
**
**  @retval RESULT_OK Slave selected successfully.
**  @retval SPI_ERROR_INVALID_PARAMETER The handle is invalid (null), or the 
**      slave address is out of range.
**  @retval SPI_ERROR_INVALID_OPERATION The slave selection works only on master
**      device. Check the port configuration.
*/
Result_t
SPI_SelectSlave(
        Handle_t handle,
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
**  @retval RESULT_OK Initialization successful.
**  @retval SPI_ERROR_INVALID_POINTER The config parameter points to null.
**  @retval SPI_ERROR_DRIVER_INITIALIZATION_FAILED Driver initialization failed.
**  @retval SPI_ERROR_TRANSMISSION_FAILED Transmission failed for some reason.
**  @retval SPI_ERROR_INVALID_BUFFER_SIZE The driver doesn't support inequal
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
Result_t
SPI_Transfer(
        Handle_t handle,
        uint8_t *dout,
        uint16_t outsz,
        uint8_t *din,
        uint16_t insz
);

#endif // spi_api_H

/* EOF */