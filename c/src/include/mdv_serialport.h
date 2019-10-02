/***************************************************************************//**
**
**  @file       mdv_serialport.h
**  @ingroup    madivaru-lib
**  @brief      Serial port API.
**  @copyright  Copyright (c) Tuomas Terho. All rights reserved.
**
**  Common serial port interface which can be easily ported for different
**  platforms without need to change the control interface.
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

#ifndef mdv_serialport_H
#define mdv_serialport_H

#include "mdv_driver.h"
#include "mdv_timer.h"
#include "mdv_handle.h"

/******************************************************************************\
**
**  ERROR CODES
**
\******************************************************************************/

/// @brief Port not initialized.
///
/// The serial port has not been initialized.
#define MDV_SERIALPORT_ERROR_PORT_NOT_INITIALIZED \
        MDV_MODULE_SPECIFIC_ERROR(0)

/// @brief Asynchronous transfer cancelled.
///
/// An asynchronous transfer was cancelled by user.
#define MDV_SERIALPORT_ERROR_ASYNC_TRANSFER_CANCELLED \
        MDV_MODULE_SPECIFIC_ERROR(1)

/// @brief Asynchronous transfer in progress.
///
/// Incompleted asynchronous transfers can't be queued.
#define MDV_SERIALPORT_ERROR_ASYNC_TRANSFER_IN_PROGRESS \
        MDV_MODULE_SPECIFIC_ERROR(2)

/// @brief Rx buffer is empty.
///
/// The receiver buffer is empty.
#define MDV_SERIALPORT_ERROR_RX_BUFFER_EMPTY \
        MDV_MODULE_SPECIFIC_ERROR(3)

/// @brief Tx buffer is full.
///
/// The transmission buffer is full.
#define MDV_SERIALPORT_ERROR_TX_BUFFER_FULL \
        MDV_MODULE_SPECIFIC_ERROR(4)

/******************************************************************************\
**
**  TYPE DEFINITIONS
**
\******************************************************************************/

/*-------------------------------------------------------------------------*//**
**  @brief Most common baud rate values.
**
**  These values are the most common baud rate values used in serial port
**  transmission. The values as well as other values are available depending on
**  the driver implementation.
**
**  Default: 9.6 kbps
*/
typedef enum
MdvSerialPortBaudRate_t{
        /// @brief 9.6 kilobits per second
        MDV_SERIALPORT_BAUDRATE_9600=9600,
        /// @brief 14.4 kilobits per second
        MDV_SERIALPORT_BAUDRATE_14400=14400,
        /// @brief 19.2 kilobits per second
        MDV_SERIALPORT_BAUDRATE_19200=19200,
        /// @brief 38.4 kilobits per second
        MDV_SERIALPORT_BAUDRATE_38400=38400,
        /// @brief 57.6 kilobits per second
        MDV_SERIALPORT_BAUDRATE_57600=57600,
        /// @brief 76.8 kilobits per second
        MDV_SERIALPORT_BAUDRATE_76800=76800,
        /// @brief 115.2 kilobits per second
        MDV_SERIALPORT_BAUDRATE_115200=115200,
        /// @brief 230.4 kilobits per second
        MDV_SERIALPORT_BAUDRATE_230400=230400,
        /// @brief 460.8 kilobits per second
        MDV_SERIALPORT_BAUDRATE_460800=460800,
        /// @brief 921.6 kilobits per second
        MDV_SERIALPORT_BAUDRATE_921600=921600
} MdvSerialPortBaudRate_t;

/*-------------------------------------------------------------------------*//**
**  @brief Data bits per character
**
**  These values can be used to specify the amount of data bits per character.
**  It depends on the hardware and the driver implementation which values are
**  supported.
**
**  Default: 8 bits
*/
typedef enum
MdvSerialPortDataBits_t{
        /// @brief 5 data bits per character
        MDV_SERIALPORT_DATABITS_5=5,
        /// @brief 6 data bits per character
        MDV_SERIALPORT_DATABITS_6,
        /// @brief 7 data bits per character
        MDV_SERIALPORT_DATABITS_7,
        /// @brief 8 data bits per character
        MDV_SERIALPORT_DATABITS_8,
        /// @brief 9 data bits per character
        MDV_SERIALPORT_DATABITS_9
} MdvSerialPortDataBits_t;

/*-------------------------------------------------------------------------*//**
**  @brief Parity on/off and polarity control values
**
**  These values are used to enable/disable parity and control its polarity.
**  It depends on the hardware and the driver implementation which values are
**  supported.
**
**  Default: No parity
*/
typedef enum
MdvSerialPortParity_t{
        /// @brief No parity.
        MDV_SERIALPORT_PARITY_NONE=0,
        /// @brief Even parity.
        MDV_SERIALPORT_PARITY_EVEN,
        /// @brief Odd parity.
        MDV_SERIALPORT_PARITY_ODD,
        /// @brief Parity value is permanent '1'.
        MDV_SERIALPORT_PARITY_STICK_1,
        /// @brief Parity value is permanent '0'.
        MDV_SERIALPORT_PARITY_STICK_0
} MdvSerialPortParity_t;

/*-------------------------------------------------------------------------*//**
**  @brief Stop bit control values
**
**  These values are used to specify the amount of stop bits.
**
**  Default: One stop bit
*/
typedef enum
MdvSerialPortStopBits_t{
        /// @brief One stop bit.
        MDV_SERIALPORT_STOPBITS_ONE=1,
        /// @brief Two stop bits.
        MDV_SERIALPORT_STOPBITS_TWO
} MdvSerialPortStopBits_t;

/*-------------------------------------------------------------------------*//**
**  @brief Flow control
**
**  These values are used to enable/disable flow control and select the flow
**  control method.
**
**  Default: No control
*/
typedef enum
MdvSerialPortFlowControl_t{
        /// @brief No flow control.
        MDV_SERIALPORT_FLOWCONTROL_NONE,
        /// @brief xOn/xOff flow control.
        MDV_SERIALPORT_FLOWCONTROL_XONXOFF,
        /// @brief Hardware RTS/CTS flow control.
        MDV_SERIALPORT_FLOWCONTROL_HARDWARE
} MdvSerialPortFlowControl_t;

/*-------------------------------------------------------------------------*//**
**  @brief Serial port configuration structure
**
**  This structure holds all the serial port information used in this interface.
*/
typedef struct
MdvSerialPortConfig_t{
        /// @brief Baud rate.
        MdvSerialPortBaudRate_t baudRate;
        /// @brief Data bits.
        MdvSerialPortDataBits_t dataBits;
        /// @brief Parity.
        MdvSerialPortParity_t parity;
        /// @brief Stop bits.
        MdvSerialPortStopBits_t stopBits;
        /// @brief Flow control.
        MdvSerialPortFlowControl_t flowControl;
} MdvSerialPortConfig_t;

/*-------------------------------------------------------------------------*//**
**  @brief Transfer completion callback.
**
**  @param[in] result Result of the transfer.
**  @param[in] userData A pointer to user specified data to be passed to the
**      callback handler.
**
**  This callback is invoked by the driver on the completion of an asynchronous
**  transfer.
*/
typedef void
(*MdvSerialPortTransferCompletedCallback_t)(
        MdvResult_t result,
        void *const userData
);

/*-------------------------------------------------------------------------*//**
**  @brief Asynchronous transfer descriptor.
**
**  This structure holds the parameters used by an asynchronous transfer
**  operation.
*/
typedef struct
MdvSerialPortTransfer_t{
        struct{
                /// @brief Transfer ongoing flag.
                uint8_t t_on:1;
        };
        /// @brief Data length in bytes.
        ///
        /// This value is set by the library and shouldn't be modified by the
        /// driver.
        uint32_t len;
        /// @brief Pointer to the length of data transferred.
        ///
        /// This pointer points to the bytesRead or bytesWritten variable. The
        /// value is set by the library and shouldn't be used by the driver.
        uint32_t *plen;
        /// @brief Pointer to data.
        ///
        /// A pointer to the data buffer where to ro from the data is
        /// transferred. The driver should use this to access the data buffer.
        /// The pointer can be modified by the driver, because it is not used in
        /// the library.
        uint8_t *data;
        /// @brief Length of data to be transferred.
        ///
        /// The initial value equals to the len value. The driver must decrease
        /// this value during transfer.
        uint32_t tleft;
        /// @brief Previous length of data to be transferred.
        ///
        /// The library uses this value to track chaegs in data transfer state.
        /// The driver must not use this.
        uint32_t ptleft;
        /// @brief Transfer timeout in time units.
        ///
        /// Timeouts are fully controlled by the library. Do not use them in the
        /// driver.
        uint32_t tout;
        /// @brief Timer for the timeout.
        ///
        /// Timeouts are fully controlled by the library. Do not use them in the
        /// driver.
        MdvTimer_t tmr;
        /// @brief Timer system to use.
        ///
        /// Timeouts are fully controlled by the library. Do not use them in the
        /// driver.
        MdvTimerSystem_t *tsys;
        /// @brief Time units.
        ///
        /// Timeouts are fully controlled by the library. Do not use them in the
        /// driver.
        MdvTimeUnit_t tu;
        /// @brief Transfer completed callback.
        ///
        /// Callbacks are fully controlled by the library. Do not use them in
        /// the driver.
        MdvSerialPortTransferCompletedCallback_t cbk;
        /// @brief A pointer to user specified data for the callback.
        ///
        /// Callbacks are fully controlled by the library. Do not use them in
        /// the driver.
        void *ud;
} MdvSerialPortTransfer_t;

/******************************************************************************\
**
**  SERIAL PORT SPECIFIC DRIVER INTERFACE FUNCTION TYPES
**
\******************************************************************************/

/*-------------------------------------------------------------------------*//**
**  @brief Transfers data to/from the serial port.
**
**  @param[in] instance A pointer to driver instance data.
**  @param[in] tfer A pointer to a transfer descriptor.
**
**  @retval MDV_RESULT_OK Successful.
**  @return On error returns a negative error value specified by the driver
**      implementation.
*/
typedef
MdvResult_t
(*MdvSerialPortDriverTransferFunction_t)(
        MdvDriverInstance_t *const instance,
        MdvSerialPortTransfer_t *const tfer
);

/******************************************************************************\
**
**  SERIAL PORT SPECIFIC DRIVER INTERFACE DEFINITION
**
\******************************************************************************/

/*-------------------------------------------------------------------------*//**
**  @brief Driver interface
**
**  This structure is a common interface for serial port drivers.
*/
typedef struct
MdvSerialPortDriverInterface_t{
        /// @brief Driver common API.
        MdvDriver_t common;
        /// @brief Serial port specific driver interface extension.
        struct{
                /// @brief Interface initialization status.
                bool initialized;
                /// @brief Serial port specific driver functions.
                struct{
                        /// @brief Serial port specific driver interface.
                        /// @brief Reads data from the serial port.
                        MdvSerialPortDriverTransferFunction_t read;
                        /// @brief Writes data to the serial port.
                        MdvSerialPortDriverTransferFunction_t write;
                } func;
        } specific;
} MdvSerialPortDriverInterface_t;

/******************************************************************************\
**
**  SERIAL PORT TYPE DEFINITION
**
\******************************************************************************/

/*-------------------------------------------------------------------------*//**
**  @brief Serial port
*/
typedef struct
MdvSerialPort_t{
        /// @brief Driver interface.
        MdvSerialPortDriverInterface_t drv;
        /// @brief Port initialization status.
        bool initialized;
        /// @brief Serial port configuration data.
        MdvSerialPortConfig_t cfg;
        /// @brief Reception descriptor.
        MdvSerialPortTransfer_t rxd;
        /// @brief Transmission descriptor.
        MdvSerialPortTransfer_t txd;
} MdvSerialPort_t;

/******************************************************************************\
**
**  DATA DEFINITIONS
**
\******************************************************************************/

/*-------------------------------------------------------------------------*//**
**  @brief Default port configuration.
*/
extern const MdvSerialPortConfig_t MdvSerialportDefaultConfig;

/******************************************************************************\
**
**  API FUNCTION DECLARATIONS
**
\******************************************************************************/

#ifdef __cplusplus
extern "C"{
#endif // ifdef __cplusplus

/*-------------------------------------------------------------------------*//**
**  @brief Sets up the serial port specific driver interface.
**
**  @param[out] port Serial port instance whose driver interface to set up.
**  @param[in] instance A pointer to a driver instance that will be associated
**             to the port.
**  @param[in] funcRead (Mandatory) A function to read data from the port.
**  @param[in] funcWrite (Mandatory) A function to write data to the port.
**
**  @retval MDV_RESULT_OK Successful.
**  @retval MDV_ERROR_INVALID_POINTER The port parameter points to NULL.
**  @retval MDV_ERROR_INVALID_PARAMETER At least one of the mandatory function
**          ponters is NULL.
*/
MdvResult_t
mdv_serialport_setup_driver_interface(
        MdvSerialPort_t *const port,
        MdvSerialPortDriverTransferFunction_t funcRead,
        MdvSerialPortDriverTransferFunction_t funcWrite
);

/*-------------------------------------------------------------------------*//**
**  @brief Initializes a serial port driver.
**
**  @param[out] port Serial port to initialize.
**  @param[in] rxCompleted RX completed callback for asynchronous read
**             operations. Set to NULL for synchronous reads.
**  @param[in] txCompleted TX completed callback for asynchronous write
**             operations. Set to NULL for synchronous writes.
**  @param[in] timerSys Timer system to use.
**  @param[in] timeUnit Time units to use.
**  @param[in] userData A pointer to user specified data.
**
**  @retval MDV_RESULT_OK Successful.
**  @retval MDV_ERROR_INVALID_POINTER The port parameter points to NULL.
**  @retval MDV_ERROR_INITIALIZATION_FAILED The driver interface has not been
**          initialized. Use the @ref mdv_serial_port_setup_driver_interface
**          function to initialize the driver interface.
**  @return On a driver error returns a negative error code. See the driver
**          implementation for more information.
**
**  Initializes the serial port and the hardware by calling driver's
**  initialization method. The driver interface must be initialized before this
**  operation.
*/
MdvResult_t
mdv_serialport_init(
        MdvSerialPort_t *const port,
        MdvSerialPortTransferCompletedCallback_t rxCompleted,
        MdvSerialPortTransferCompletedCallback_t txCompleted,
        MdvTimerSystem_t *const timerSys,
        MdvTimeUnit_t timeUnit,
        void *const userData
);

/*-------------------------------------------------------------------------*//**
**  @brief Returns the current configuration of the given serial port.
**
**  @param[in] port Serial port for which to get the configuration.
**  @param[out] config Pointer to a configuration structure where the
**              information will be copied.
**
**  @retval MDV_RESULT_OK Configuration got successfully.
**  @return MDV_ERROR_INVALID_POINTER The port or config parameter points to
**          NULL.
*/
MdvResult_t
mdv_serialport_get_current_configuration(
        MdvSerialPort_t *const port,
        MdvSerialPortConfig_t *const config
);

/*-------------------------------------------------------------------------*//**
**  @brief Opens a serial port.
**
**  @param[in] port Serial port to open.
**  @param[in] config Pointer to a configuration structure.
**  @param[in] handle Pointer to a handle.
**
**  @retval MDV_RESULT_OK Open successful.
**  @retval MDV_ERROR_INVALID_POINTER The port, config or handle pointer
**          points to NULL.
**  @return On a driver error returns a negative error code. See the driver
**          implementation for more information.
**
**  Opens a serial port using the given configuration. Returns a port handle as
**  an output parameter.
*/
MdvResult_t
mdv_serialport_open(
        MdvSerialPort_t *const port,
        MdvSerialPortConfig_t *const config,
        MdvHandle_t *const handle
);

/*-------------------------------------------------------------------------*//**
**  @brief Closes a serial port.
**
**  @param[out] handle Pointer to the handle of the port being closed.
**
**  @retval MDV_RESULT_OK Closed successfully.
**  @retval MDV_ERROR_INVALID_POINTER The handle pointer points to NULL.
**  @return On a driver error returns a negative error code. See the driver
**          implementation for more information.
**
**  Closes a serial port and resets the given port handle.
*/
MdvResult_t
mdv_serialport_close(
        MdvHandle_t *const handle
);

/*-------------------------------------------------------------------------*//**
**  @brief Changes serial port's configuration.
**
**  @param[in] handle Serial port handle.
**  @param[in] config Pointer to a configuration structure.
**
**  @retval MDV_RESULT_OK Successful
**  @retval MDV_ERROR_INVALID_POINTER The config or handle pointer points to
**          NULL.
**  @retval MDV_ERROR_INVALID_HANDLE The handle is invalid.
**  @return On a driver error returns a negative error code. See the driver
**          implementation for more information.
**
**  Closes a serial port and re-opens it with the new configuration values.
*/
MdvResult_t
mdv_serialport_change_configuration(
        MdvHandle_t *const handle,
        MdvSerialPortConfig_t *const config
);

/*-------------------------------------------------------------------------*//**
**  @brief Reads data from the serial port.
**
**  @param[in] handle Serial port handle.
**  @param[in] length Data length in bytes.
**  @param[out] data Pointer to an output buffer.
**  @param[out] bytesRead Pointer to a variable for received data length. This
**              parameter is optional and can be NULL.
**  @param[in] timeout Maximum time in milliseconds between two bytes or an
**             initial wait timeout.
**
**  @retval MDV_RESULT_OK Read successful.
**  @retval MDV_ERROR_INVALID_POINTER The handle or data pointer points to NULL.
**  @retval MDV_ERROR_INVALID_HANDLE The handle is invalid.
**  @retval MDV_ERROR_TIMEOUT Reception timed out.
**  @return On a driver error returns a negative error code. See the driver
**          implementation for more information.
**
**  Reads a chunk of data from the serial port. If not enough data received
**  within the given timeout time, returns an error.
*/
MdvResult_t
mdv_serialport_read(
        MdvHandle_t *const handle,
        uint32_t length,
        uint8_t *const data,
        uint32_t *const bytesRead,
        uint32_t timeout
);

/*-------------------------------------------------------------------------*//**
**  @brief Writes data to the serial port.
**
**  @param[in] handle Serial port handle.
**  @param[in] length Data length in bytes.
**  @param[in] data Pointer to a data buffer.
**  @param[out] bytesWritten Pointer to a variable for transmitted data
**              length. This parameter is optional and can be NULL.
**  @param[in] timeout Maximum time in milliseconds between two bytes or an
**             initial wait timeout.
**
**  @retval MDV_RESULT_OK Write successful.
**  @retval MDV_ERROR_INVALID_POINTER The handle or data pointer points to NULL.
**  @retval MDV_ERROR_INVALID_HANDLE The handle is invalid.
**  @retval MDV_ERROR_TIMEOUT Transmission timed out.
**  @return On a driver error returns a negative error code. See the driver
**          implementation for more information.
**
**  Writes a chunk of data to a TX buffer.
*/
MdvResult_t
mdv_serialport_write(
        MdvHandle_t *const handle,
        uint32_t length,
        uint8_t *const data,
        uint32_t *const bytesWritten,
        uint32_t timeout
);

/*-------------------------------------------------------------------------*//**
**  @brief Gets a single character from a serial port.
**
**  @param[in] handle Serial port handle.
**  @param[out] data Pointer to a character variable.
**
**  @retval MDV_RESULT_OK Character got successfully.
**  @retval MDV_ERROR_INVALID_POINTER The handle or data pointer points to NULL.
**  @retval MDV_ERROR_INVALID_HANDLE The handle is invalid.
**  @retval MDV_SERIALPORT_ERROR_RX_BUFFER_EMPTY The RX buffer is empty.
**
**  Gets a single character from the serial port. If the RX buffer is empty,
**  returns an error without waiting.
*/
MdvResult_t
mdv_serialport_getchar(
        MdvHandle_t *const handle,
        uint8_t *const data
);

/*-------------------------------------------------------------------------*//**
**  @brief Puts a single character to a serial port.
**
**  @param[in] handle Serial port handle.
**  @param[in] data Character to put.
**
**  @retval MDV_RESULT_OK Character put successfully.
**  @retval MDV_ERROR_INVALID_POINTER The handle pointer points to NULL.
**  @retval MDV_ERROR_INVALID_HANDLE The handle is invalid.
**  @retval MDV_SERIALPORT_ERROR_TX_BUFFER_FULL The TX buffer is full.
**  @return On a driver error returns a negative error code. See the driver
**          implementation for more information.
**
**  Puts a single character to the TX buffer. If the buffer is full, returns
**  an error without waiting.
*/
MdvResult_t
mdv_serialport_putchar(
        MdvHandle_t *const handle,
        uint8_t data
);

/*-------------------------------------------------------------------------*//**
**  @brief Runs asynchronous processes.
**
**  @param[in] handle Serial port handle.
**
**  @return MDV_RESULT_OK Successful.
**  @return MDV_ERROR_INVALID_POINTER The handle pointer points to NULL.
**
**  This function must be called periodically in order to run asynchronous
**  transfers. The callback functions are called in this context.
*/
MdvResult_t
mdv_serialport_runtime_process(
        MdvHandle_t *const handle
);

#ifdef __cplusplus
}
#endif // ifdef __cplusplus

#endif // ifndef mdv_serialport_H

/* EOF */
