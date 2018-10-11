/***************************************************************************//**
**
**  @file       sensor_api.h
**  @ingroup    sensor_framework
**  @brief      Generic sensor API and driver interface.
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
********************************************************************************
**
**  FUNCTION REFERENCE
**  ==================
**
**  Some functions are mandatory and the others are optional. The mandatory
**  functions must be implemented in the user application or in the sensor
**  driver, depending on the function. Mandatory functions are marked as [M] in
**  the followint list and optionals are marked as [O].
**
**  -----------------------------
**  Sensor API callback functions
**  -----------------------------
**  These functions are implemented in the user application separately for each
**  sensor instance.
**
**  [M] SensorCbk_Input_t()                Sensor input data request callback.
**  [M] SensorCbk_Output_t()               Sensor output data event callback.
**  [O] SensorCbk_CtrlCmd_t()              Sensor control command callback.
**
**  --------------------
**  Driver API functions
**  --------------------
**  These functions are implemented separately in each sensor driver.
**
**  [M] SensorDrv_Init_t()                 Initializes a sensor.
**  [O] SensorDrv_SetCalibrationParams_t() Sets calibration parameters.
**  [O] SensorDrv_StartCalibration_t()     Starts calibration.
**  [O] SensorDrv_GetCalibrationState_t()  Gets calibration state.
**  [O] SensorDrv_GetCalibrationData_t()   Gets calibration data.
**  [O] SensorDrv_SetCalibrationData_t()   Sets calibration data.
**  [O] SensorDrv_GetConfiguration_t()     Gets configuration data.
**  [O] SensorDrv_SetConfiguration_t()     Sets configuration data.
**  [O] SensorDrv_On_t()                   Turns the sensor ON.
**  [O] SensorDrv_Off_t()                  Turns the sensor OFF.
**  [M] SensorDrv_SetInput_t()             Sets input data to the sensor.
**  [M] SensorDrv_GetOutput_t()            Gets output data from the sensor.
**  [O] SensorDrv_Run_t()                  Runs the driver state machine.
**
**  --------------------
**  Sensor API functions
**  --------------------
**  These functions are implemented in the sensor_api.c and are used by
**  the user application.
**
**  SensorAPI_InitDriver()                 Initializes a sensor driver.
**  SensorAPI_GetHandle()                  Gets a handle to a sensor.
**  SensorAPI_SetCalibrationParams()       Sets calibration parameters.
**  SensorAPI_StartCalibration()           Starts a sensor calibration routine.
**  SensorAPI_GetCalibrationState()        Gets sensor calibration state.
**  SensorAPI_GetCalibrationData()         Gets sensor calibration data.
**  SensorAPI_SetCalibrationData()         Sets sensor calibration data.
**  SensorAPI_IsCalibrated()               Gets the sensor calibration status.
**  SensorAPI_GetConfiguration()           Gets configuration data.
**  SensorAPI_SetConfiguration()           Sets configuration data.
**  SensorAPI_Control()                    Controls a sensor.
**  SensorAPI_Status()                     Gets the driver control status.
**  SensorAPI_Run()                        Runs a sensor.
**
**  ---------------------------
**  Driver helper API functions
**  ---------------------------
**  These functions are implemented in the sensor_api.c and are used by the
**  sensor driver.
**
**  SensorDrv_SetOutputStatus()            Sets output status.
**  SensorDrv_SetOutputData()              Sets output data.
**
\******************************************************************************/

#ifndef sensor_api_H
#define sensor_api_H

#include "types.h"
#include "timer.h"

/******************************************************************************\
**
**  ERROR CODES
**
\******************************************************************************/

/// @brief Invalid pointer parameter.
///
/// At least one pointer parameter points to null.
#define SENSORAPI_ERROR_INVALID_POINTER -1

/// @brief Invalid parameter value.
///
/// At least one parameter value is out of range.
#define SENSORAPI_ERROR_INVALID_PARAMETER -2

/// @brief Function not implemented in the driver.
///
/// Trying to use a function which is not supported by the driver (not
/// implemented). Refer to the documentation of the particular driver.
#define SENSORAPI_ERROR_NOT_IMPLEMENTED -3

/******************************************************************************\
**
**  SENSOR API CALLBACK FUNCTIONS
**
\******************************************************************************/

/*-------------------------------------------------------------------------*//**
**  @brief Sensor input data request callback.
**
**  @param[in] index The index of the input data in a sensor-specific data set.
**
**  @return The callback must always return valid input data.
**
**  Sensor framework uses this type of callback to retrieve raw input data from
**  the user application to the sensor driver for further analysis. Typically
**  this is input from sensor hardware.
**
**  @remarks This is a mandatory function which must be implemented in the user
**  application separately for each sensor.
*/
typedef
var_t
(*SensorCbk_Input_t)(
        uint8_t index
);

/*-------------------------------------------------------------------------*//**
**  @brief Sensor event callback.
**
**  @param[in] index The index of the output data in a sensor-specific data set.
**  @param[in] data Data from the sensor.
**
**  @return No return value.
**
**  Sensor framework uses this type of callback to send the output of the sensor
**  ("the result") from the driver to the user application, whenever the output
**  changes.
**
**  @remarks This is a mandatory function which must be implemented in the user
**  application separately for each sensor.
*/
typedef
void
(*SensorCbk_Output_t)(
        uint8_t index,
        var_t data
);

/*-------------------------------------------------------------------------*//**
**  @brief Sensor control command callback.
**
**  @param[in] cmdId The ID of the calibration command.
**  @param[in] data Data related to the command.
**
**  @return Caller may require data returned by the callback (see the
**      description of the particular command for details).
**
**  Sensor framework invokes this type of callback when the sensor needs to have
**  control to a certain function in the user application, e.g. to adjust PWM,
**  control I/O etc. The use of this callback depends on the requirements of the
**  driver.
**
**  @remarks This is an optional function which may be implemented in the user
**  application separately for each sensor supporting control commands.
*/
typedef
var_t
(*SensorCbk_CtrlCmd_t)(
        uint8_t cmdId,
        var_t data
);

/******************************************************************************\
**
**  SENSOR API DATA TYPES
**
\******************************************************************************/

/*-------------------------------------------------------------------------*//**
**  @brief Sensor control.
**
**  These values are used to control the sensor on and off.
*/
typedef enum
Sensor_Control_t{
        /// Turns a sensor off.
        SENSOR_DISABLE=0,
        /// Turns a sensor on.
        SENSOR_ENABLE,
        /// Halts a sensor.
        SENSOR_HALT
} Sensor_Control_t;

/*-------------------------------------------------------------------------*//**
**  @brief Sensor output status.
**
**  Sensor output status describes the validity of the sensor data.
*/
typedef enum
Sensor_OutputStatus_t{
        /// Sensor data is not valid, for example, sensor initialization hasn't
        /// been completed yet.
        SENSOR_DATA_UNKNOWN,
        /// There is a failure in reading the sensor data, thus the data is not
        /// valid.
        SENSOR_DATA_FAILURE,
        /// Sensor data is valid.
        SENSOR_DATA_OK
} Sensor_OutputStatus_t;

/*-------------------------------------------------------------------------*//**
**  @brief Sensor output.
**
**  This structure contains all output-related properties and output data.
**  The user application shall not manipulate these values.
*/
typedef struct
Sensor_Output_t{
        /// Status of the sensor output.
        /// @remarks This value is set by a sensor driver by using the function
        /// @ref SensorDrv_SetOutputStatus.
        Sensor_OutputStatus_t st;
        /// Indicator for changes in data and status.
        /// @remarks This value is set by functions
        /// @ref SensorDrv_SetOutputStatus and @ref SensorDrv_SetOutputData. The
        /// value is resetted automatically by the API when the change is
        /// handled in the function @ref SensorAPI_Run. Do not write this value
        /// manually.
        bool ci;
        /// Output data.
        /// @remarks This value is set by a sensor driver by using the function
        /// @ref SensorDrv_SetOutputData.
        var_t data;
        /// Data refresh cycle timer.
        /// @remarks This value is used internally by the API. The value must be
        /// initialized in the sensor driver's initialization function.
        Timer_t tim;
        /// Data refresh cycle length in milliseconds.
        /// @remarks This value is used internally by the API. The value must be
        /// initialized in the sensor driver's initialization function.
        uint16_t rc;
} Sensor_Output_t;

/******************************************************************************\
**
**  DRIVER API FUNCTIONS
**
\******************************************************************************/

/*-------------------------------------------------------------------------*//**
**  @brief Initializes the driver.
**
**  @param[in] userData User defined data.
**
**  @return No return value.
**
**  This function is implemented in the sensor driver and the pointer to the
**  function is stored into the driver instance. The function must initialize
**  the parameters of the output data set. It may initialize also the driver
**  specific parameters.
**
**  @remarks This function is mandatory and must be implemented in the driver.
*/
typedef void
(*SensorDrv_Init_t)(
        void *userData
);

/*-------------------------------------------------------------------------*//**
**  @brief Sets calibration parameters.
**
**  @param[in] calibrationParams Pointer to sensor specific calibration
**      parameters.
**  @param[in] userData User defined data.
**
**  @return No return value.
**
**  This function is implemented in the sensor driver and the pointer to the
**  function is stored into the driver instance.
**
**  @remarks This function is optional. If implemented, it shall copy the
**  calibration parameters to the instance of the driver.
*/
typedef void
(*SensorDrv_SetCalibrationParams_t)(
        void *calibrationParams,
        void *userData
);

/*-------------------------------------------------------------------------*//**
**  @brief Starts calibration.
**
**  @param[in] userData User defined data.
**
**  @return No return value.
**
**  This function is implemented in the sensor driver and the pointer to the
**  function is stored into the driver instance.
**
**  @remarks This function is optional. The driver may implement this function
**  if it supports calibration and requires initializations during calibration
**  startup.
*/
typedef void
(*SensorDrv_StartCalibration_t)(
        void *userData
);

/*-------------------------------------------------------------------------*//**
**  @brief Gets calibration state.
**
**  @param[out] calibrationState Pointer to sensor specific calibration state.
**  @param[in] userData User defined data.
**
**  @return The result of the operation.
**  @retval RESULT_OK Calibration state retrieved successfully.
**  @retval <0 Calibration state retrieval failed (driver specific reason).
**
**  This function is implemented in the sensor driver and the pointer to the
**  function is stored into the driver instance.
**
**  @remarks This function is optional. The driver may implement this function
**  if the calibration is performed in steps. The states are driver specific.
*/
typedef Result_t
(*SensorDrv_GetCalibrationState_t)(
        void *calibrationState,
        void *userData
);

/*-------------------------------------------------------------------------*//**
**  @brief Gets calibration data.
**
**  @param[out] calibrationData Pointer to sensor specific calibration data.
**  @param[in] userData User defined data.
**
**  @return No return value.
**
**  This function is implemented in the sensor driver and the pointer to the
**  function is stored into the driver instance.
**
**  @remarks This function is optional. The driver must implement this function
**  if it supports calibration.
*/
typedef void
(*SensorDrv_GetCalibrationData_t)(
        void *calibrationData,
        void *userData
);

/*-------------------------------------------------------------------------*//**
**  @brief Sets calibration data.
**
**  @param[in] calibrationData Pointer to sensor specific calibration data.
**  @param[in] userData User defined data.
**
**  @return No return value.
**
**  This function is implemented in the sensor driver and the pointer to the
**  function is stored into the driver instance.
**
**  @remarks This function is optional. The driver must implement this function
**  if it supports calibration.
*/
typedef void
(*SensorDrv_SetCalibrationData_t)(
        void *calibrationData,
        void *userData
);

/*-------------------------------------------------------------------------*//**
**  @brief Gets configuration data.
**
**  @param[in] configurationData Pointer to sensor specific configuration data.
**  @param[in] userData User defined data.
**
**  @return No return value.
**
**  This function is implemented in the sensor driver and the pointer to the
**  function is stored into the driver instance.
**
**  @remarks This function is optional. The driver may implement this function
**  to support configuration. This function can be used to return the current
**  configuration state as well as the default configuration.
*/
typedef void
(*SensorDrv_GetConfiguration_t)(
        void *configurationData,
        void *userData
);

/*-------------------------------------------------------------------------*//**
**  @brief Sets configuration data.
**
**  @param[in] configurationData Pointer to sensor specific configuration data.
**  @param[in] userData User defined data.
**
**  @return The change state of the configuration data.
**  @retval true Configuration data has changed.
**  @retval false Configuration data has not changed.
**
**  This function is implemented in the sensor driver and the pointer to the
**  function is stored into the driver instance.
**
**  @remarks This function is optional. The driver may implement this function
**  to support configuration. The function must perform comparison between the
**  new and old configuration data. It must return true if the configuration
**  data has changed or false otherwise.
*/
typedef bool
(*SensorDrv_SetConfiguration_t)(
        void *configurationData,
        void *userData
);

/*-------------------------------------------------------------------------*//**
**  @brief Turns the sensor ON.
**
**  @param[in] userData User defined data.
**
**  @return No return value.
**
**  This function is implemented in the sensor driver and the pointer to the
**  function is stored into the driver instance.
**
**  @remarks This function is optional. The driver may implement this function
**  to support on/off functionality.
*/
typedef void
(*SensorDrv_On_t)(
        void *userData
);

/*-------------------------------------------------------------------------*//**
**  @brief Turns the sensor OFF.
**
**  @param[in] userData User defined data.
**
**  @return No return value.
**
**  This function is implemented in the sensor driver and the pointer to the
**  function is stored into the driver instance.
**
**  @remarks This function is optional. The driver may implement this function
**  to support on/off functionality.
*/
typedef void
(*SensorDrv_Off_t)(
        void *userData
);

/*-------------------------------------------------------------------------*//**
**  @brief Sets input data to the sensor.
**
**  @param[in] index Index of the data in the input data set.
**  @param[in] inputData Input data.
**  @param[in] userData User defined data.
**
**  @return No return value.
**
**  This function is implemented in the sensor driver and the pointer to the
**  function is stored into the driver instance.
**
**  @remarks This function is mandatory. The driver must use this function to
**  store input data to the input data set of the driver instance.
*/
typedef void
(*SensorDrv_SetInput_t)(
        uint8_t index,
        var_t inputData,
        void *userData
);

/*-------------------------------------------------------------------------*//**
**  @brief Gets output data from the sensor.
**
**  @param[in] index Index of the data in the output data set.
**  @param[in] userData User defined data.
**
**  @return Output data.
**
**  This function is implemented in the sensor driver and the pointer to the
**  function is stored into the driver instance.
**
**  @remarks This function is mandatory. The driver must use this function to
**  return the output data from the output data set of the driver instance.
**  The driver may use this function to produce the output data from the input
**  data.
*/
typedef Sensor_Output_t *
(*SensorDrv_GetOutput_t)(
        uint8_t index,
        void *userData
);

/*-------------------------------------------------------------------------*//**
**  @brief Runs the driver state machine.
**
**  @param[in] userData User defined data.
**
**  @return No return value.
**
**  This function is implemented in the sensor driver and the pointer to the
**  function is stored into the driver instance.
**
**  @remarks This function is optional. The driver may implement this function
**  to support a state machine.
**
*/
typedef
void
(*SensorDrv_Run_t)(
        void *userData
);

/******************************************************************************\
**
**  DRIVER INTERFACE
**
\******************************************************************************/

/*-------------------------------------------------------------------------*//**
**  @brief Driver interface
**
**  This structure is a common interface for all sensor drivers.
*/
typedef struct
SensorDrv_t{
        /// A function to initialize the driver (mandatory).
        SensorDrv_Init_t Init;
        /// A function to set calibration parameters (optional).
        SensorDrv_SetCalibrationParams_t SetCalibrationParams;
        /// A function to start calibration sequence (optional).
        SensorDrv_StartCalibration_t StartCalibration;
        /// A function to get calibration state (optional).
        SensorDrv_GetCalibrationState_t GetCalibrationState;
        /// A function to get calibration data (optional).
        SensorDrv_GetCalibrationData_t GetCalibrationData;
        /// A function to set calibration data (optional).
        SensorDrv_SetCalibrationData_t SetCalibrationData;
        /// A function to get the sensor configuration data (optional).
        SensorDrv_GetConfiguration_t GetConfiguration;
        /// A function to set the sensor configuration data (optional).
        SensorDrv_SetConfiguration_t SetConfiguration;
        /// A function to set sensor input data (mandatory).
        SensorDrv_SetInput_t SetInput;
        /// A function to get sensor output data (mandatory).
        SensorDrv_GetOutput_t GetOutput;
        /// A function to turn the sensor on (optional).
        SensorDrv_On_t On;
        /// A function to turn the sensor off (optional).
        SensorDrv_On_t Off;
        /// A function to run the sensor state machine (optional).
        SensorDrv_Run_t Run;
        /// Sensor data sets.
        struct{
                /// Data item count in the input data set.
                uint8_t idc;
                /// Data item count in the output data set.
                uint8_t odc;
                /// Pointer to the input data set.
                var_t *ids;
                /// Pointer to the output data set.
                Sensor_Output_t *ods;
        } ds;
        /// Driver data (don't use manually).
        struct{
                /// Input data callback.
                SensorCbk_Input_t icbk;
                /// Output data callback.
                SensorCbk_Output_t ocbk;
                /// Control command callback.
                SensorCbk_CtrlCmd_t ccbk;
                /// Request sensor data.
                bool rreq;
                /// Sensor control.
                Sensor_Control_t ctrl;
                /// Calibration state.
                bool cal;
                /// Timer system.
                TimerSys_t *tsys;
                /// User defined data.
                void *ud;
        } dd;
} SensorDrv_t;

/******************************************************************************\
**
**  DRIVER HELPER API FUNCTIONS
**
\******************************************************************************/

/*-------------------------------------------------------------------------*//**
**  @brief Sets output status.
**
**  @param[in] output Output to set.
**  @param[in] status New status of the output.
**
**  Sets output status and maintains the output changed state.
*/
void
SensorDrv_SetOutputStatus(
        Sensor_Output_t *output,
        Sensor_OutputStatus_t status
);

/*-------------------------------------------------------------------------*//**
**  @brief Sets output data.
**
**  @param[in] output Output to set.
**  @param[in] data New output data.
*/
void
SensorDrv_SetOutputData(
        Sensor_Output_t *output,
        var_t data
);

/******************************************************************************\
**
**  SENSOR API FUNCTIONS
**
\******************************************************************************/

/*-------------------------------------------------------------------------*//**
**  @brief Initializes a sensor driver.
**
**  @param[in] driver Sensor driver to be initialized.
**  @param[in] sensor Type of the sensor to be associated with the driver.
**  @param[in] funcInit Driver function pointer (mandatory).
**  @param[in] funcSetCalibrationParams Driver function pointer (optional, set
**      to null if not used).
**  @param[in] funcStartCalibration Driver function pointer (optional, set to
**      null if not used).
**  @param[in] funcGetCalibrationState Driver function pointer (optional, set to
**      null if not used).
**  @param[in] funcGetCalibrationData Driver function pointer (optional, set to
**      null if not used).
**  @param[in] funcSetCalibrationData Driver function pointer (optional, set to
**      null if not used).
**  @param[in] funcGetConfiguration Driver function pointer (optional, set to 
**      null if not used).
**  @param[in] funcSetConfiguration Driver function pointer (optional, set to 
**      null if not used).
**  @param[in] funcSetInput Driver function pointer (mandatory).
**  @param[in] funcGetOutput Driver function pointer (mandatory).
**  @param[in] funcOn Driver function pointer (optional, set to null if not
**      used).
**  @param[in] funcOff Driver function pointer (optional, set to null if not
**      used).
**  @param[in] funcRun Driver function pointer (optional, set to null if not
**      used).
**  @param[in] inputDataItemCount Item count in the input data set (driver
**      specific, must be greater than zero).
**  @param[in] outputDataItemCount Item count in the output data set (driver
**      specific, must be greater than zero).
**  @param[in] inputDataSet A pointer to an input data set (must point into a
**      table that contains at least inputDataItemCount items).
**  @param[in] outputDataSet A pointer to an output data set (must point into a
**      table that contains at least outputDataItemCount items).
**  @param[in] inputCallback Callback routine that handles raw input data
**      requests (mandatory).
**  @param[in] outputCallback Callback routine that handles sensor data events
**      (mandatory).
**  @param[in] ctrlCmdCallback Callback routine for special control commands
**      (optional, driver specific).
**  @param[in] timerSys Timer system to be used for timings.
**  @param[in] userData A pointer to user defined data.
**
**  @return No return value.
**
**  Initializes and sets up a sensor driver instance. This function is usually
**  wrapped inside a driver specific initialization function. The driver is
**  responsible to pass the function parameters according to their support in
**  the particular driver.
*/
void
SensorAPI_InitDriver(
        SensorDrv_t *driver,
        SensorDrv_Init_t funcInit,
        SensorDrv_SetCalibrationParams_t funcSetCalibrationParams,
        SensorDrv_StartCalibration_t funcStartCalibration,
        SensorDrv_GetCalibrationState_t funcGetCalibrationState,
        SensorDrv_GetCalibrationData_t funcGetCalibrationData,
        SensorDrv_SetCalibrationData_t funcSetCalibrationData,
        SensorDrv_GetConfiguration_t funcGetConfiguration,
        SensorDrv_SetConfiguration_t funcSetConfiguration,
        SensorDrv_SetInput_t funcSetInput,
        SensorDrv_GetOutput_t funcGetOutput,
        SensorDrv_On_t funcOn,
        SensorDrv_On_t funcOff,
        SensorDrv_Run_t funcRun,
        uint8_t inputDataItemCount,
        uint8_t outputDataItemCount,
        var_t *inputDataSet,
        Sensor_Output_t *outputDataSet,
        SensorCbk_Input_t inputCallback,
        SensorCbk_Output_t outputCallback,
        SensorCbk_CtrlCmd_t ctrlCmdCallback,
        TimerSys_t *timerSys,
        void *userData
);

/*-------------------------------------------------------------------------*//**
**  @brief Gets a handle to a sensor.
**
**  @param[in] driver A sensor driver.
**  @param[out] sensorHndl Pointer to a sensor handle variable.
**
**  @retval RESULT_OK Handle successfully got.
**  @retval SENSORAPI_ERROR_INVALID_POINTER The driver or the sensorHndl 
**      pointer points to null.
**  @retval SENSOR_ERROR_NOT_INITIALIZED Driver not initialized.
*/
Result_t
SensorAPI_GetHandle(
        SensorDrv_t *driver,
        Handle_t *sensorHndl
);

/*-------------------------------------------------------------------------*//**
**  @brief Sets calibration parameters.
**
**  @param[in] sensorHndl Handle to a sensor.
**  @param[in] calibrationParams Pointer to sensor specific calibration
**      parameters.
**
**  @retval RESULT_OK Successful.
**  @retval SENSOR_ERROR_INVALID_HANDLE Invalid sensor handle.
**  @retval SENSORAPI_ERROR_INVALID_POINTER The calibrationParams pointer points
**      to null.
**  @retval SENSOR_ERROR_NOT_IMPLEMENTED Sensor doesn't feature calibration.
*/
Result_t
SensorAPI_SetCalibrationParams(
        Handle_t sensorHndl,
        void *calibrationParams
);

/*-------------------------------------------------------------------------*//**
**  @brief Starts a sensor calibration routine.
**
**  @param[in] sensorHndl Handle to a sensor.
**
**  @retval RESULT_OK Successful.
**  @retval SENSOR_ERROR_INVALID_HANDLE Invalid sensor handle.
**  @retval SENSOR_ERROR_NOT_IMPLEMENTED Sensor doesn't feature calibration.
*/
Result_t
SensorAPI_StartCalibration(
        Handle_t sensorHndl
);

/*-------------------------------------------------------------------------*//**
**  @brief Gets sensor calibration state.
**
**  @param[in] sensorHndl Handle to a sensor.
**  @param[out] calibrationState Sensor specific calibration state.
**
**  @retval RESULT_OK Successful.
**  @retval SENSOR_ERROR_INVALID_HANDLE Invalid sensor handle.
**  @retval SENSORAPI_ERROR_INVALID_POINTER The calibrationState pointer points
**      to null.
**  @retval SENSOR_ERROR_NOT_IMPLEMENTED Sensor doesn't feature calibration.
*/
Result_t
SensorAPI_GetCalibrationState(
        Handle_t sensorHndl,
        void *calibrationState
);

/*-------------------------------------------------------------------------*//**
**  @brief Gets sensor calibration data.
**
**  @param[in] sensorHndl Handle to a sensor.
**  @param[out] calibrationData Pointer to sensor specific calibration data.
**
**  @retval RESULT_OK Successful.
**  @retval SENSOR_ERROR_INVALID_HANDLE Invalid sensor handle.
**  @retval SENSORAPI_ERROR_INVALID_POINTER The calibrationData pointer points
**      to null.
**  @retval SENSOR_ERROR_NOT_IMPLEMENTED Sensor doesn't feature calibration.
*/
Result_t
SensorAPI_GetCalibrationData(
        Handle_t sensorHndl,
        void *calibrationData
);

/*-------------------------------------------------------------------------*//**
**  @brief Sets sensor calibration data.
**
**  @param[in] sensorHndl Handle to a sensor.
**  @param[in] calibrationData Pointer to sensor specific calibration data.
**
**  @retval RESULT_OK Successful.
**  @retval SENSOR_ERROR_INVALID_HANDLE Invalid sensor handle.
**  @retval SENSORAPI_ERROR_INVALID_POINTER The calibrationData pointer points
**      to null.
**  @retval SENSOR_ERROR_INVALID_PARAMETER Calibration data is wrong.
**  @retval SENSOR_ERROR_NOT_IMPLEMENTED Sensor doesn't feature calibration.
**
**  Turns the sensor status to "calibrated".
*/
Result_t
SensorAPI_SetCalibrationData(
        Handle_t sensorHndl,
        void *calibrationData
);

/*-------------------------------------------------------------------------*//**
**  @brief Gets the sensor calibration status.
**
**  @param[in] sensorHndl Handle to a sensor.
**
**  @retval true Sensor is calibrated.
**  @retval false Sensor is not calibrated, or the handle is invalid.
*/
bool
SensorAPI_IsCalibrated(
        Handle_t sensorHndl
);

/*-------------------------------------------------------------------------*//**
**  @brief Gets the sensor configuration data.
**
**  @param[in] sensorHndl Handle to a sensor.
**  @param[in] configuration Pointer to sensor specific configuration data.
**
**  @retval RESULT_OK Successful.
**  @retval SENSOR_ERROR_INVALID_HANDLE Invalid sensor handle.
**  @retval SENSORAPI_ERROR_INVALID_POINTER The configuration pointer points to
**      null.
**  @retval SENSOR_ERROR_NOT_IMPLEMENTED Sensor doesn't feature configuration.
*/
Result_t
SensorAPI_GetConfiguration(
        Handle_t sensorHndl,
        void *configuration
);

/*-------------------------------------------------------------------------*//**
**  @brief Sets the sensor configuration data.
**
**  @param[in] sensorHndl Handle to a sensor.
**  @param[in] configuration Pointer to sensor specific configuration data.
**
**  @retval RESULT_OK Successful.
**  @retval SENSOR_ERROR_INVALID_HANDLE Invalid sensor handle.
**  @retval SENSORAPI_ERROR_INVALID_POINTER The configuration pointer points to
**      null.
**  @retval SENSOR_ERROR_INVALID_PARAMETER Configuration data is wrong.
**  @retval SENSOR_ERROR_NOT_IMPLEMENTED Sensor doesn't feature configuration.
*/
Result_t
SensorAPI_SetConfiguration(
        Handle_t sensorHndl,
        void *configuration
);

/*-------------------------------------------------------------------------*//**
**  @brief Controls a sensor.
**
**  @param[in] sensorHndl Handle to a sensor.
**  @param[in] control Sensor control value.
**
**  @return No return value.
**
**  If the sensor handle is invalid, does nothing.
*/
void
SensorAPI_Control(
        Handle_t sensorHndl,
        Sensor_Control_t control
);

/*-------------------------------------------------------------------------*//**
**  @brief Gets the driver control status.
**
**  @param[in] sensorHndl Handle to a sensor.
**
**  @retval Sensor control status. Returns SENSOR_DISABLE if the handle is
**          invalid.
*/
Sensor_Control_t
SensorAPI_Status(
        Handle_t sensorHndl
);

/*-------------------------------------------------------------------------*//**
**  @brief Runs a sensor.
**
**  @param[in] sensorHndl Handle to a sensor.
**
**  @return No return value.
**
**  This function can be used instead of the Sensor_RunDriver if the sensors
**  need to be run in separate threads or processes.
*/
void
SensorAPI_Run(
        Handle_t sensorHndl
);

#endif // sensor_api_H

/* EOF */