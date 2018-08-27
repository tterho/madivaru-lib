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
**  This interface doesn't support port enumeration which is typically used in
**  Windows environment to find hot-pluggable devices, such as USB-to-serial 
**  port adapters.
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

#include "fifo.h"
#include "timer.h"

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

/// @brief Resource is in use.
///
/// The serial port handle already points to an opened port and can't be used
/// to open another port before closing.
#define SP_ERROR_RESOURCE_IN_USE -3

/// @brief Error in driver.
///
/// The driver has not been initialized correctly, e.g. the TimerSys pointer 
/// points to null.
#define SP_ERROR_DRIVER_INTERNAL_ERROR -4

/// @brief Rx buffer is empty.
///
/// The receiver buffer is empty.
#define SP_ERROR_RX_BUFFER_EMPTY -5

/// @brief Tx buffer is full.
///
/// The transmission buffer is full.
#define SP_ERROR_TX_BUFFER_FULL -6

/// @brief Timeout.
///
/// The operation has been timed out.
#define SP_ERROR_TIMEOUT -7

/******************************************************************************\
**
**  DATA TYPE DEFINITIONS
**
\******************************************************************************/

/*-------------------------------------------------------------------------*//** 
**  @brief COM ports
**
**  These values are used to access serial ports. The availability of the ports 
**  depends on the system implementation.
**
**  In embedded systems the port COM1 is usually associated with the UART0, COM2
**  with UART1 etc. In Linux the COM1 stands for ttyS0, COM2 ttyS1 etc.
*/
typedef enum 
SP_COMPort_t{
        /// @brief COM1 port
        SP_COM1=0,
        /// @brief COM2 port
        SP_COM2,
        /// @brief COM3 port
        SP_COM3,
        /// @brief COM4 port
        SP_COM4,
        /// @brief Maximum ports available in the system.
        SP_MAX_PORTS 
} SP_COMPort_t;

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
        SP_SB_ONE,
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

/******************************************************************************\
**
**  DRIVER FUNCTION API
**
\******************************************************************************/

/*-------------------------------------------------------------------------*//** 
**  @brief Initializes the serial port driver.
**
**  Initializes the serial port hardware I/O and FIFOs.
**
**  @return The result of the initialization.
*/
typedef 
Result_t 
(*SPDrv_Init_t)(
        void
);

/*-------------------------------------------------------------------------*//** 
**  @brief Opens the serial port driver.
**
**  @retval RESULT_OK Successful
**  @return On error returns a negative error value specified by the driver
**          implementation.
**
**  Opens the serial port by using its current configuration. Enables receiver, 
**  transmitter and interrupts if necessary and clears the RX and TX buffers.
*/
typedef 
Result_t 
(*SPDrv_Open_t)(
        void
);

/*-------------------------------------------------------------------------*//** 
**  @brief Closes the serial port.
**
**  @retval RESULT_OK Successful
**  @return On error returns a negative error value specified by the driver
**          implementation.
**
**  Disables receiver, transmitter and interrupts if necessary.
*/
typedef 
Result_t 
(*SPDrv_Close_t)(
        void
);

/*-------------------------------------------------------------------------*//** 
**  @brief Reads data from the serial port.
**
**  @param[in] length Data length in bytes.
**  @param[out] data Pointer to an output buffer.
**  @param[out] bytesRead Pointer to a variable for received data length. This
**      parameter is optional and can be null.
**  @param[in] timeout Maximum time between two bytes or initial wait timeout.
**
**  @retval RESULT_OK Read successful.
**  @return On error returns a negative error value specified by the driver
**          implementation.
**
**  Reads a chunk of data from the RX buffer. If the buffer is empty or not
**  enough data received within the given timeout time, returns an error.
*/
typedef 
Result_t 
(*SPDrv_Read_t)(
        uint32_t length,
        uint8_t *data,
        uint32_t *bytesRead,
        uint32_t timeout
);

/*-------------------------------------------------------------------------*//** 
**  @brief Writes data to the serial port.
**
**  @param[in] length Data length in bytes.
**  @param[in] data Pointer to a data buffer.
**  @param[out] bytesWritten Pointer to a variable for transmitted data 
**      length. This parameter is optional and can be null.
**  @param[in] timeout Maximum time between two bytes or initial wait timeout.
**
**  @retval RESULT_OK Write successful.
**  @return On error returns a negative error value specified by the driver
**          implementation.
**
**  Writes a chunk of data to a TX buffer.
*/
typedef 
Result_t 
(*SPDrv_Write_t)(
        uint32_t length,
        uint8_t *data,
        uint32_t *bytesWritten,
        uint32_t timeout
);

/*-------------------------------------------------------------------------*//** 
**  @brief Gets a character from the serial port.
**
**  @param[out] data Pointer to a variable where to store the character.
**
**  @retval RESULT_OK Successful
**  @retval SP_ERROR_INVALID_POINTER data parameter points to null.
**  @return On error returns a negative error value specified by the driver
**          implementation.
**
**  Stores a character from the RX buffer to the data parameter and advances 
**  buffer's read pointer. If the buffer is empty, returns an error.
*/
typedef 
Result_t 
(*SPDrv_GetChar_t)(
        uint8_t *data
);

/*-------------------------------------------------------------------------*//** 
**  @brief Puts a character to the serial port.
**
**  @param[in] data Character to send.
**
**  @retval RESULT_OK Put successful.
**  @return On error returns a negative error value specified by the driver
**          implementation.
**
**  Stores the given character to the TX buffer and advances buffer's write
**  pointer. If the buffer is full, returns an error.
*/
typedef 
Result_t 
(*SPDrv_PutChar_t)(
        uint8_t data
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
SPDrv_t{
        /// Initializes the driver.
        SPDrv_Init_t Init;        
        /// Opens the serial port.
        SPDrv_Open_t Open;
        /// Closes the serial port.
        SPDrv_Close_t Close;         
        /// Reads data from the serial port.
        SPDrv_Read_t Read;
        /// Writes data to the serial port.
        SPDrv_Write_t Write;
        /// Gets a character from the serial port.
        SPDrv_GetChar_t GetChar;         
        /// Puts a character to the serial port.
        SPDrv_PutChar_t PutChar; 
        /// Serial port configuration data.
        SP_Config_t cfg;
        /// TX (output) buffer.
        FIFO_t txb;
        /// RX (input) buffer.
        FIFO_t rxb;
        /// Timer system to use.
        TimerSys_t *tsys;
} SPDrv_t;

/******************************************************************************\
**
**  PUBLIC DATA DEFINITIONS
**
\******************************************************************************/

/*-------------------------------------------------------------------------*//** 
**  @brief Contains default configuration values for serial ports.
*/
extern const SP_Config_t SP_DefaultConfig;

/******************************************************************************\
**
**  PUBLIC FUNCTION DEFINITIONS
**
\******************************************************************************/

/*-------------------------------------------------------------------------*//** 
**  @brief Maps a driver to a serial port number.
**
**  @param[in] port Serial port number where to map the driver.
**  @param[in] driver Pointer to a driver being mapped.
**
**  @retval RESULT_OK Mapping successful.
**  @retval SP_ERROR_INVALID_POINTER The driver parameter points to null.
**  @retval SP_ERROR_INVALID_PARAMETER The port parameter value is invalid, or 
**      the port number is already in use.
**  @return On another error returns a negative error code.
**
**  Tries to initialize the serial port hardware by using driver's 
**  initialization method. If successful, maps the driver to the given serial 
**  port number.
*/
Result_t
SP_MapDriver(
        SP_COMPort_t port,
        SPDrv_t *driver
);

/*-------------------------------------------------------------------------*//** 
**  @brief Returns the current configuration of the given serial port.
**
**  @param[in] port Serial port for which to get the configuration.
**  @param[out] config Pointer to a configuration structure where the 
**      information will be copied.
**
**  @retval RESULT_OK Configuration got successfully.
**  @return SP_ERROR_INVALID_POINTER The configuration parameter points to null.
**  @return SP_ERROR_INVALID_PARAMETER The port parameter is invalid.
*/
Result_t
SP_GetCurrentConfig(
        SP_COMPort_t port, 
        SP_Config_t *config
);

/*-------------------------------------------------------------------------*//** 
**  @brief Opens a serial port.
**
**  @param[in] port Serial port to open.
**  @param[in] configuration Pointer to a configuration structure.
**  @param[out] handle Pointer to a variable where the serial port handle will
**      be stored.
**
**  @retval RESULT_OK Open successful.
**  @retval SP_ERROR_INVALID_POINTER The configuration parameter or the handle
**      parameter points to null.
**  @retval SP_ERROR_INVALID_PARAMETER The port parameter is invalid.
**
**  Opens a serial port using the given configuration. Returns a port handle as
**  an output parameter.
*/
Result_t
SP_Open(
        SP_COMPort_t port,
        SP_Config_t *config,
        void **handle
);

/*-------------------------------------------------------------------------*//** 
**  @brief Closes a serial port.
**
**  @param[in,out] handle Pointer to the handle of the port being closed.
**
**  @retval RESULT_OK Closed successfully.
**  @retval SP_ERROR_INVALID_POINTER The handle parameter points to null.
**  @retval SP_ERROR_INVALID_PARAMETER The handle is invalid.
**
**  Closes a serial port and resets the given port handle.
*/
Result_t
SP_Close(
        void **handle
);

/*-------------------------------------------------------------------------*//** 
**  @brief Changes serial port's configuration.
**
**  @param[in] handle Serial port handle.
**  @param[in] configuration Pointer to a configuration structure.
**
**  @retval RESULT_OK Successful
**  @retval SP_ERROR_INVALID_POINTER One or more of the pointer parameters      
**      handle and configuration points to null.
**  @retval SP_ERROR_INVALID_PARAMETER The handle is invalid.
**
**  Closes a serial port and re-opens it with the new configuration values.
*/
Result_t
SP_ChangeConfig(
        void *handle,
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
**  @param[in] timeout Maximum time between two bytes or initial wait timeout.
**
**  @retval RESULT_OK Read successful.
**  @retval SP_ERROR_INVALID_POINTER One or more of the pointer parameters
**      handle and data points to null.
**  @retval SP_ERROR_INVALID_PARAMETER The handle is invalid.
**  @retval SP_ERROR_TIMEOUT Reception timed out.
**
**  Reads a chunk of data from the RX buffer. If the buffer is empty or not
**  enough data received within the given timeout time, returns an error.
*/
Result_t
SP_Read(
        void *handle,
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
**  @param[in] timeout Maximum time between two bytes or initial wait timeout.
**
**  @retval RESULT_OK Write successful.
**  @retval SP_ERROR_INVALID_POINTER One or more of the pointer parameters
**      handle and data points to null.
**  @retval SP_ERROR_INVALID_PARAMETER The handle is invalid.
**  @retval SP_ERROR_TIMEOUT Transmission timed out.
**
**  Writes a chunk of data to a TX buffer.
*/
Result_t
SP_Write(
        void *handle,
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
**  Gets a single character from the RX buffer. If the buffer is empty, returns
**  an error without waiting.
*/
Result_t
SP_GetChar(
        void *handle,
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
**
**  Puts a single character to the TX buffer. If the buffer is full, returns
**  an error without waiting.
*/
Result_t
SP_PutChar(
        void *handle,
        uint8_t data
);

#endif // serialport_H

/* EOF */
