/***************************************************************************//**
**
**  @defgroup   serialcomm Serial port communication library
**
**  A general purpose serial communication API and driver API library.
**
**  @file       serialport.h
**  @ingroup    serialcomm
**  @brief      Serial port API.
**  @copyright  Copyright (C) 2012-2018 Tuomas Terho. All rights reserved.
**
**  Common serial port interface which can be easily ported for different 
**  platforms without need to change the control interface.
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

#ifndef serialport_H
#define serialport_H

#include "timer.h"

#ifdef __cplusplus
extern "C"{
#endif // ifdef __cplusplus

/******************************************************************************\
**
**  ERROR CODES
**
\******************************************************************************/

/// @brief Invalid pointer.
///
/// At least one of the pointer parameters is invalid (points to null).
#define SP_ERROR_INVALID_POINTER -1

/// @brief Invalid parameter.
///
/// At least one of the parameter values is invalid (out of range).
#define SP_ERROR_INVALID_PARAMETER -2

/// @brief Driver interface not specified.
///
/// The driver interface functions are not initialized. Use the @ref 
/// SP_SetupDriverInterface function to initialize the interface.
#define SP_ERROR_NO_DRIVER_INTERFACE -3

/// @brief Resource is in use.
///
/// The serial port handle already points to an opened port and can't be used
/// to open another port before closing.
#define SP_ERROR_RESOURCE_IN_USE -4

/// @brief Port not initialized.
///
/// The serial port has not been initialized.
#define SP_ERROR_PORT_NOT_INITIALIZED -5

/// @brief Asynchronous transfer cancelled.
///
/// An asynchronous transfer was cancelled by user.
#define SP_ERROR_ASYNC_TRANSFER_CANCELLED -6

/// @brief Asynchronous transfer in progress.
///
/// Incompleted asynchronous transfers can't be queued.
#define SP_ERROR_ASYNC_TRANSFER_IN_PROGRESS -7

/// @brief Rx buffer is empty.
///
/// The receiver buffer is empty.
#define SP_ERROR_RX_BUFFER_EMPTY -8

/// @brief Tx buffer is full.
///
/// The transmission buffer is full.
#define SP_ERROR_TX_BUFFER_FULL -9

/// @brief Error in driver.
///
/// The driver has not been initialized correctly, e.g. the TimerSys pointer 
/// points to null.
#define SP_ERROR_DRIVER_INTERNAL_ERROR -10

/// @brief Timeout.
///
/// The operation has been timed out.
#define SP_ERROR_TIMEOUT -11

/// @brief Invalid configuration.
///
/// One or more of the configuration values are not supported by the driver.
#define SP_ERROR_INVALID_CONFIGURATION -12

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
SP_BaudRate_t{
        /// @brief 9.6 kilobits per second
        SP_BR_9600=9600,
        /// @brief 14.4 kilobits per second
        SP_BR_14400=14400,
        /// @brief 19.2 kilobits per second
        SP_BR_19200=19200,
        /// @brief 38.4 kilobits per second
        SP_BR_38400=38400,
        /// @brief 57.6 kilobits per second
        SP_BR_57600=57600,
        /// @brief 76.8 kilobits per second
        SP_BR_76800=76800,
        /// @brief 115.2 kilobits per second
        SP_BR_115200=115200,
        /// @brief 230.4 kilobits per second
        SP_BR_230400=230400,
        /// @brief 460.8 kilobits per second
        SP_BR_460800=460800,
        /// @brief 921.6 kilobits per second
        SP_BR_921600=921600
} SP_BaudRate_t;

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
SP_DataBits_t{
        /// @brief 5 data bits per character
        SP_DB_5=5,
        /// @brief 6 data bits per character
        SP_DB_6,
        /// @brief 7 data bits per character
        SP_DB_7,
        /// @brief 8 data bits per character
        SP_DB_8,
        /// @brief 9 data bits per character
        SP_DB_9
} SP_DataBits_t;

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
SP_Parity_t{
        /// No parity.
        SP_PA_NONE,
        /// Even parity.
        SP_PA_EVEN,
        /// Odd parity.
        SP_PA_ODD,
        /// Parity value is permanent '1'.
        SP_PA_STICK_1,
        /// Parity value is permanent '0'.
        SP_PA_STICK_0
} SP_Parity_t;

/*-------------------------------------------------------------------------*//**
**  @brief Stop bit control values
**
**  These values are used to specify the amount of stop bits.
**
**  Default: One stop bit
*/
typedef enum
SP_StopBits_t{
        /// One stop bit.
        SP_SB_ONE=1,
        /// Two stop bits.
        SP_SB_TWO
} SP_StopBits_t;

/*-------------------------------------------------------------------------*//**
**  @brief Flow control
**
**  These values are used to enable/disable flow control and select the flow
**  control method.
**
**  Default: No control
*/
typedef enum
SP_FlowControl_t{
        /// No flow control.
        SP_FC_NONE,
        /// xOn/xOff flow control.
        SP_FC_XONXOFF,
        /// Hardware RTS/CTS flow control.
        SP_FC_HARDWARE
} SP_FlowControl_t;

/*-------------------------------------------------------------------------*//**
**  @brief Serial port configuration structure
**
**  This structure holds all the serial port information used in this interface.
*/
typedef struct
SP_Config_t{
        /// Baud rate.
        SP_BaudRate_t BaudRate;
        /// Data bits.
        SP_DataBits_t DataBits;
        /// Parity.
        SP_Parity_t Parity;
        /// Stop bits.
        SP_StopBits_t StopBits;
        /// Flow control.
        SP_FlowControl_t FlowControl;
} SP_Config_t;

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
(*SP_TransferCompletedCbk_t)(
        Result_t result,
        void *userData
);

/*-------------------------------------------------------------------------*//**
**  @brief Asynchronous transfer descriptor.
**
**  This structure holds the parameters used by an asynchronous transfer
**  operation.
*/
typedef struct
SP_Transfer_t{
        struct{
                /// Transfer ongoing flag.
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
        Timer_t tmr;
        /// @brief Timer system to use.
        ///
        /// Timeouts are fully controlled by the library. Do not use them in the
        /// driver.
        TimerSys_t *tsys;
        /// @brief Time units.
        ///
        /// Timeouts are fully controlled by the library. Do not use them in the
        /// driver.
        Timer_TimeUnit_t tu;
        /// @brief Transfer completed callback.
        ///
        /// Callbacks are fully controlled by the library. Do not use them in
        /// the driver.
        SP_TransferCompletedCbk_t cbk;
        /// @brief A pointer to user specified data for the callback.
        ///
        /// Callbacks are fully controlled by the library. Do not use them in
        /// the driver.
        void *ud;
} SP_Transfer_t;

/******************************************************************************\
**
**  DRIVER API FUNCTIONS
**
\******************************************************************************/

/*-------------------------------------------------------------------------*//**
**  @brief Initializes the serial port driver.
**
**  @return The result of the initialization.
*/
typedef
Result_t
(*SPDrv_Func_Init_t)(
        void
);

/*-------------------------------------------------------------------------*//**
**  @brief Opens the serial port driver.
**
**  @param[in] cfg Port configuration.
**
**  @retval RESULT_OK Successful
**  @return On error returns a negative error value specified by the driver
**      implementation.
**
**  Opens the serial port by using its current configuration. Enables receiver,
**  transmitter and interrupts if necessary and clears the RX and TX buffers.
*/
typedef
Result_t
(*SPDrv_Func_Open_t)(
        SP_Config_t *cfg
);

/*-------------------------------------------------------------------------*//**
**  @brief Closes the serial port.
**
**  @retval RESULT_OK Successful
**  @return On error returns a negative error value specified by the driver
**      implementation.
**
**  Disables receiver, transmitter and interrupts if necessary.
*/
typedef
Result_t
(*SPDrv_Func_Close_t)(
        void
);

/*-------------------------------------------------------------------------*//**
**  @brief Transfers data to/from the serial port.
**
**  @param[in] tfer A pointer to a transfer descriptor.
**
**  @retval RESULT_OK Successful.
**  @return On error returns a negative error value specified by the driver
**      implementation.
*/
typedef
Result_t
(*SPDrv_Func_Transfer_t)(
        SP_Transfer_t *tfer
);

/*-------------------------------------------------------------------------*//**
**  @brief Runs the driver.
**
**  @param[in] rxd A pointer to a reception descriptor.
**  @param[in] txd A pointer to a transmission descriptor.
**
**  @retval RESULT_OK Successful.
**  @return On error returns a negative error value specified by the driver
**      implementation.
*/
typedef
Result_t
(*SPDrv_Func_RunDriver_t)(
        SP_Transfer_t *rxd,
        SP_Transfer_t *txd
);

/******************************************************************************\
**
**  DRIVER INTERFACE DEFINITION
**
\******************************************************************************/

/*-------------------------------------------------------------------------*//**
**  @brief Driver interface
**
**  This structure is a common interface for serial port drivers.
*/
typedef struct
SP_COMPort_t{
        /// Initializes the driver.
        SPDrv_Func_Init_t drvfuncInit;
        /// Opens the serial port.
        SPDrv_Func_Open_t drvfuncOpen;
        /// Closes the serial port.
        SPDrv_Func_Close_t drvfuncClose;
        /// Reads data from the serial port.
        SPDrv_Func_Transfer_t drvfuncRead;
        /// Writes data to the serial port.
        SPDrv_Func_Transfer_t drvfuncWrite;
        /// Runs the driver.
        SPDrv_Func_RunDriver_t drvfuncRunDriver;
        /// Port initialization status.
        bool_t init;
        /// Serial port configuration data.
        SP_Config_t cfg;
        /// Reception descriptor.
        SP_Transfer_t rxd;
        /// Transmission descriptor.
        SP_Transfer_t txd;
} SP_COMPort_t;

/******************************************************************************\
**
**  PUBLIC DATA DEFINITIONS
**
\******************************************************************************/

/*-------------------------------------------------------------------------*//**
**  @brief Default port configuration.
*/
extern const SP_Config_t SP_DefaultConfig;

/******************************************************************************\
**
**  PUBLIC FUNCTION DEFINITIONS
**
\******************************************************************************/

/*-------------------------------------------------------------------------*//**
**  @brief Sets up the driver interface.
**
**  @param[out] port Serial port driver to set up.
**  @param[in] funcInit (Mandatory) A function to initialize the port hardware.
**  @param[in] funcOpen (Mandatory) A function to open the port.
**  @param[in] funcClose (Mandatory) A function to close the port.
**  @param[in] funcRead (Mandatory) A function to read data from the port.
**  @param[in] funcWrite (Mandatory) A function to write data to the port.
**  @param[in] funcRunDriver (Optional) A function to support asynchronous 
**      data transfer processing.
**
**  @retval RESULT_OK Successful.
**  @retval SP_ERROR_INVALID_POINTER The port parameter points to null.
**  @retval SP_ERROR_INVALID_PARAMETER At least one of the mandatory function
**      pointers is null.
*/
Result_t
SP_SetupDriverInterface(
        SP_COMPort_t *port,
        SPDrv_Func_Init_t funcInit,
        SPDrv_Func_Open_t funcOpen,
        SPDrv_Func_Close_t funcClose,
        SPDrv_Func_Transfer_t funcRead,
        SPDrv_Func_Transfer_t funcWrite,
        SPDrv_Func_RunDriver_t funcRunDriver
);

/*-------------------------------------------------------------------------*//**
**  @brief Initializes a serial port driver.
**
**  @param[out] port Serial port driver to initialize.
**  @param[in] rxCompleted RX completed callback for asynchronous read
**      operations. Set to null for synchronous reads.
**  @param[in] txCompleted TX completed callback for asynchronous write
**      operations. Set to null for synchronous writes.
**  @param[in] timerSys Timer system to use.
**  @param[in] timeUnit Time units to use.
**  @param[in] userData A pointer to user specified data.
**
**  @retval RESULT_OK Successful.
**  @retval SP_ERROR_INVALID_POINTER The port parameter points to null.
**  @return On a driver error returns a negative error code. See the driver 
**      implementation for more information.
**
**  Initializes the serial port and the hardware by invoking driver's 
**  initialization method.
*/
Result_t
SP_InitPort(
        SP_COMPort_t *port,
        SP_TransferCompletedCbk_t rxCompleted,
        SP_TransferCompletedCbk_t txCompleted,
        TimerSys_t *timerSys,
        Timer_TimeUnit_t timeUnit,
        void *userData
);

/*-------------------------------------------------------------------------*//**
**  @brief Returns the current configuration of the given serial port.
**
**  @param[in] port Serial port for which to get the configuration.
**  @param[out] config Pointer to a configuration structure where the
**      information will be copied.
**
**  @retval RESULT_OK Configuration got successfully.
**  @return SP_ERROR_INVALID_POINTER The port or config parameter points to
**      null.
*/
Result_t
SP_GetCurrentConfig(
        SP_COMPort_t *port,
        SP_Config_t *config
);

/*-------------------------------------------------------------------------*//**
**  @brief Opens a serial port.
**
**  @param[in] port Serial port to open.
**  @param[in] config Pointer to a configuration structure.
**  @param[out] handle Pointer to a variable where the serial port handle will
**      be stored.
**
**  @retval RESULT_OK Open successful.
**  @retval SP_ERROR_INVALID_POINTER The port, config or handle parameter points
**      to null.
**  @return On a driver error returns a negative error code. See the driver
**      implementation for more information.
**
**  Opens a serial port using the given configuration. Returns a port handle as
**  an output parameter.
*/
Result_t
SP_Open(
        SP_COMPort_t *port,
        SP_Config_t *config,
        Handle_t *handle
);

/*-------------------------------------------------------------------------*//**
**  @brief Closes a serial port.
**
**  @param[out] handle Pointer to the handle of the port being closed.
**
**  @retval RESULT_OK Closed successfully.
**  @retval SP_ERROR_INVALID_POINTER The handle parameter points to null.
**  @return On a driver error returns a negative error code. See the driver
**      implementation for more information.
**
**  Closes a serial port and resets the given port handle.
*/
Result_t
SP_Close(
        Handle_t *handle
);

/*-------------------------------------------------------------------------*//**
**  @brief Changes serial port's configuration.
**
**  @param[in] handle Serial port handle.
**  @param[in] config Pointer to a configuration structure.
**
**  @retval RESULT_OK Successful
**  @retval SP_ERROR_INVALID_POINTER The config parameter points to null.
**  @retval SP_ERROR_INVALID_PARAMETER The handle is invalid.
**  @return On a driver error returns a negative error code. See the driver
**      implementation for more information.
**
**  Closes a serial port and re-opens it with the new configuration values.
*/
Result_t
SP_ChangeConfig(
        Handle_t handle,
        SP_Config_t *config
);

/*-------------------------------------------------------------------------*//**
**  @brief Reads data from the serial port.
**
**  @param[in] handle Serial port handle.
**  @param[in] length Data length in bytes.
**  @param[out] data Pointer to an output buffer.
**  @param[out] bytesRead Pointer to a variable for received data length. This
**      parameter is optional and can be null.
**  @param[in] timeout Maximum time in milliseconds between two bytes or an
**      initial wait timeout.
**
**  @retval RESULT_OK Read successful.
**  @retval SP_ERROR_INVALID_POINTER The data or bytesRead parameter points to
**      null.
**  @retval SP_ERROR_INVALID_PARAMETER The handle is invalid.
**  @retval SP_ERROR_TIMEOUT Reception timed out.
**  @return On a driver error returns a negative error code. See the driver
**      implementation for more information.
**
**  Reads a chunk of data from the serial port. If not enough data received
**  within the given timeout time, returns an error.
*/
Result_t
SP_Read(
        Handle_t handle,
        uint32_t length,
        uint8_t *data,
        uint32_t *bytesRead,
        uint32_t timeout
);

/*-------------------------------------------------------------------------*//**
**  @brief Writes data to the serial port.
**
**  @param[in] handle Serial port handle.
**  @param[in] length Data length in bytes.
**  @param[in] data Pointer to a data buffer.
**  @param[out] bytesWritten Pointer to a variable for transmitted data
**      length. This parameter is optional and can be null.
**  @param[in] timeout Maximum time in milliseconds between two bytes or an
**      initial wait timeout.
**
**  @retval RESULT_OK Write successful.
**  @retval SP_ERROR_INVALID_POINTER One or more of the pointer parameters
**      handle and data points to null.
**  @retval SP_ERROR_INVALID_PARAMETER The handle is invalid.
**  @retval SP_ERROR_TIMEOUT Transmission timed out.
**  @return On a driver error returns a negative error code. See the driver
**      implementation for more information.
**
**  Writes a chunk of data to a TX buffer.
*/
Result_t
SP_Write(
        Handle_t handle,
        uint32_t length,
        uint8_t *data,
        uint32_t *bytesWritten,
        uint32_t timeout
);

/*-------------------------------------------------------------------------*//**
**  @brief Gets a single character from a serial port.
**
**  @param[in] handle Serial port handle.
**  @param[out] data Pointer to a character variable.
**
**  @retval RESULT_OK Character got successfully.
**  @retval SP_ERROR_INVALID_POINTER One or more of the pointer parameters
**      handle and data points to null.
**  @retval SP_ERROR_INVALID_PARAMETER The handle is invalid.
**  @retval SP_ERROR_RX_BUFFER_EMPTY The RX buffer is empty.
**
**  Gets a single character from the serial port. If the RX buffer is empty,
**  returns an error without waiting.
*/
Result_t
SP_GetChar(
        Handle_t handle,
        uint8_t *data
);

/*-------------------------------------------------------------------------*//**
**  @brief Puts a single character to a serial port.
**
**  @param[in] handle Serial port handle.
**  @param[in] data Character to put.
**
**  @retval RESULT_OK Character put successfully.
**  @retval SP_ERROR_INVALID_POINTER Pointer parameter handle points to null.
**  @retval SP_ERROR_INVALID_PARAMETER The handle is invalid.
**  @retval SP_ERROR_TX_BUFFER_FULL The TX buffer is full.
**  @return On a driver error returns a negative error code. See the driver
**      implementation for more information.
**
**  Puts a single character to the TX buffer. If the buffer is full, returns
**  an error without waiting.
*/
Result_t
SP_PutChar(
        Handle_t handle,
        uint8_t data
);

/*-------------------------------------------------------------------------*//**
**  @brief Runs asynchronous processes.
**
**  @param[in] handle Serial port handle.
**
**  @return RESULT_OK Successful.
**  @return SP_ERROR_INVALID_PARAMETER The handle is invalid.
*/
Result_t
SP_Run(
        Handle_t handle
);

#ifdef __cplusplus
}
#endif // ifdef __cplusplus

#endif // serialport_H

/* EOF */
