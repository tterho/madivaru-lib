/***************************************************************************//**
**
**  @file       mdv_sensor_api.h
**  @ingroup    madivaru-lib
**  @brief      Generic sensor API and driver interface.
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
\******************************************************************************/

#ifndef mdv_sensor_api_H
#define mdv_sensor_api_H

#include "mdv_types.h"
#include "mdv_timer.h"

/******************************************************************************\
**
**  ERROR CODES
**
\******************************************************************************/

/// @brief Invalid pointer parameter.
///
/// At least one pointer parameter points to null.
#define MDV_SENSORAPI_ERROR_INVALID_POINTER -1

/// @brief Invalid parameter value.
///
/// At least one parameter value is out of range.
#define MDV_SENSORAPI_ERROR_INVALID_PARAMETER -2

/// @brief Function not implemented in the driver.
///
/// Trying to use a function which is not supported by the driver (not
/// implemented). Refer to the documentation of the particular driver.
#define MDV_SENSORAPI_ERROR_NOT_IMPLEMENTED -3

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
MdvVar_t
(*MdvSensorInputCallback_t)(
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
(*MdvSensorOutputCallback_t)(
        uint8_t index,
        MdvVar_t data
);

/*-------------------------------------------------------------------------*//**
**  @brief Sensor control command callback.
**
**  @param[in] cmdId The ID of the calibration command.
-**  @param[in] data Data related to the command.
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
MdvVar_t
(*MdvSensorControlCommandCallback_t)(
        uint8_t cmdId,
        MdvVar_t data
);

/******************************************************************************\
**
**  TYPE DEFINITIONS
**
\******************************************************************************/

/*-------------------------------------------------------------------------*//**
**  @brief Sensor control.
**
**  These values are used to control the sensor on and off.
*/
typedef enum
MdvSensorControl_t{
        /// Turns a sensor off.
        MDV_SENSOR_DISABLE=0,
        /// Turns a sensor on.
        MDV_SENSOR_ENABLE,
        /// Halts a sensor.
        MDV_SENSOR_HALT
} MdvSensorControl_t;

/*-------------------------------------------------------------------------*//**
**  @brief Sensor output data status.
**
**  Sensor output data status indicates the validity of the sensor data.
*/
typedef enum
MdvSensorOutputDataStatus_t{
        /// Sensor data is not valid, for example, sensor initialization hasn't
        /// been completed yet.
        MDV_SENSOR_OUTPUT_DATA_UNKNOWN,
        /// There is a failure in reading the sensor data, thus the data is not
        /// valid.
        MDV_SENSOR_OUTPUT_DATA_FAILURE,
        /// Sensor data is valid.
        MDV_SENSOR_OUTPUT_DATA_OK
} MdvSensorOutputDataStatus_t;

/*-------------------------------------------------------------------------*//**
**  @brief Sensor output.
**
**  This structure contains all output-related properties and output data.
**  The user application shall not manipulate these values.
*/
typedef struct
MdvSensorOutput_t{
        /// Status of the sensor output.
        /// @remarks This value is set by a sensor driver by using the function
        /// @ref mdv_sensor_set_output_status.
        MdvSensorOutputDataStatus_t st;
        /// Indicator for changes in data and status.
        /// @remarks This value is set by functions
        /// @ref mdv_sensor_set_output_status and @ref mdv_sensor_set_output_data. The
        /// value is resetted automatically by the API when the change is
        /// handled in the function @ref mdv_sensor_run. Do not write this value
        /// manually.
        bool ci;
        /// Output data.
        /// @remarks This value is set by a sensor driver by using the function
        /// @ref mdv_sensor_set_output_data.
        MdvVar_t data;
        /// Data refresh cycle timer.
        /// @remarks This value is used internally by the API. The value must be
        /// initialized in the sensor driver's initialization function.
        MdvTimer_t tim;
        /// Data refresh cycle length in milliseconds.
        /// @remarks This value is used internally by the API. The value must be
        /// initialized in the sensor driver's initialization function.
        uint16_t rc;
} MdvSensorOutput_t;

/******************************************************************************\
**
**  DRIVER API FUNCTION DECLARATIONS
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
(*MdvSensorDriverInterface_Init_t)(
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
(*MdvSensorDriverInterface_SetCalibrationParams_t)(
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
(*MdvSensorDriverInterface_StartCalibration_t)(
        void *userData
);

/*-------------------------------------------------------------------------*//**
**  @brief Gets calibration state.
**
**  @param[out] calibrationState Pointer to sensor specific calibration state.
**  @param[in] userData User defined data.
**
**  @return The result of the operation.
**  @retval MDV_RESULT_OK Calibration state retrieved successfully.
**  @retval <0 Calibration state retrieval failed (driver specific reason).
**
**  This function is implemented in the sensor driver and the pointer to the
**  function is stored into the driver instance.
**
**  @remarks This function is optional. The driver may implement this function
**  if the calibration is performed in steps. The states are driver specific.
*/
typedef MdvResult_t
(*MdvSensorDriverInterface_GetCalibrationState_t)(
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
(*MdvSensorDriverInterface_GetCalibrationData_t)(
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
(*MdvSensorDriverInterface_SetCalibrationData_t)(
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
(*MdvSensorDriverInterface_GetConfiguration_t)(
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
(*MdvSensorDriverInterface_SetConfiguration_t)(
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
(*MdvSensorDriverInterface_On_t)(
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
(*MdvSensorDriverInterface_Off_t)(
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
(*MdvSensorDriverInterface_SetInput_t)(
        uint8_t index,
        MdvVar_t inputData,
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
typedef MdvSensorOutput_t *
(*MdvSensorDriverInterface_GetOutput_t)(
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
(*MdvSensorDriverInterface_Run_t)(
        void *userData
);

/******************************************************************************\
**
**  DRIVER INTERFACE DEFINITION
**
\******************************************************************************/

/*-------------------------------------------------------------------------*//**
**  @brief Driver interface
**
**  This structure is a common interface for all sensor drivers.
*/
typedef struct
MdvSensor_t{
        /// A function to initialize the driver (mandatory).
        MdvSensorDriverInterface_Init_t Init;
        /// A function to set calibration parameters (optional).
        MdvSensorDriverInterface_SetCalibrationParams_t SetCalibrationParams;
        /// A function to start calibration sequence (optional).
        MdvSensorDriverInterface_StartCalibration_t StartCalibration;
        /// A function to get calibration state (optional).
        MdvSensorDriverInterface_GetCalibrationState_t GetCalibrationState;
        /// A function to get calibration data (optional).
        MdvSensorDriverInterface_GetCalibrationData_t GetCalibrationData;
        /// A function to set calibration data (optional).
        MdvSensorDriverInterface_SetCalibrationData_t SetCalibrationData;
        /// A function to get the sensor configuration data (optional).
        MdvSensorDriverInterface_GetConfiguration_t GetConfiguration;
        /// A function to set the sensor configuration data (optional).
        MdvSensorDriverInterface_SetConfiguration_t SetConfiguration;
        /// A function to set sensor input data (mandatory).
        MdvSensorDriverInterface_SetInput_t SetInput;
        /// A function to get sensor output data (mandatory).
        MdvSensorDriverInterface_GetOutput_t GetOutput;
        /// A function to turn the sensor on (optional).
        MdvSensorDriverInterface_On_t On;
        /// A function to turn the sensor off (optional).
        MdvSensorDriverInterface_On_t Off;
        /// A function to run the sensor state machine (optional).
        MdvSensorDriverInterface_Run_t Run;
        /// Sensor data sets.
        struct{
                /// Data item count in the input data set.
                uint8_t idc;
                /// Data item count in the output data set.
                uint8_t odc;
                /// Pointer to the input data set.
                MdvVar_t *ids;
                /// Pointer to the output data set.
                MdvSensorOutput_t *ods;
        } ds;
        /// Driver data (don't use manually).
        struct{
                /// Input data callback.
                MdvSensorInputCallback_t icbk;
                /// Output data callback.
                MdvSensorOutputCallback_t ocbk;
                /// Control command callback.
                MdvSensorControlCommandCallback_t ccbk;
                /// Request sensor data.
                bool rreq;
                /// Sensor control.
                MdvSensorControl_t ctrl;
                /// Calibration state.
                bool cal;
                /// Timer system.
                MdvTimerSystem_t *tsys;
                /// User defined data.
                void *ud;
        } dd;
} MdvSensor_t;

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
mdv_sensor_set_output_status(
        MdvSensorOutput_t *output,
        MdvSensorOutputDataStatus_t status
);

/*-------------------------------------------------------------------------*//**
**  @brief Sets output data.
**
**  @param[in] output Output to set.
**  @param[in] data New output data.
*/
void
mdv_sensor_set_output_data(
        MdvSensorOutput_t *output,
        MdvVar_t data
);

/******************************************************************************\
**
**  SENSOR API FUNCTIONS
**
\******************************************************************************/

/*-------------------------------------------------------------------------*//**
**  @brief Initializes a sensor driver.
**
**  @param[in] sensor Sensor to be initialized.
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
mdv_sensor_init(
        MdvSensor_t *sensor,
        MdvSensorDriverInterface_Init_t funcInit,
        MdvSensorDriverInterface_SetCalibrationParams_t funcSetCalibrationParams,
        MdvSensorDriverInterface_StartCalibration_t funcStartCalibration,
        MdvSensorDriverInterface_GetCalibrationState_t funcGetCalibrationState,
        MdvSensorDriverInterface_GetCalibrationData_t funcGetCalibrationData,
        MdvSensorDriverInterface_SetCalibrationData_t funcSetCalibrationData,
        MdvSensorDriverInterface_GetConfiguration_t funcGetConfiguration,
        MdvSensorDriverInterface_SetConfiguration_t funcSetConfiguration,
        MdvSensorDriverInterface_SetInput_t funcSetInput,
        MdvSensorDriverInterface_GetOutput_t funcGetOutput,
        MdvSensorDriverInterface_On_t funcOn,
        MdvSensorDriverInterface_On_t funcOff,
        MdvSensorDriverInterface_Run_t funcRun,
        uint8_t inputDataItemCount,
        uint8_t outputDataItemCount,
        MdvVar_t *inputDataSet,
        MdvSensorOutput_t *outputDataSet,
        MdvSensorInputCallback_t inputCallback,
        MdvSensorOutputCallback_t outputCallback,
        MdvSensorControlCommandCallback_t ctrlCmdCallback,
        MdvTimerSystem_t *timerSys,
        void *userData
);

/*-------------------------------------------------------------------------*//**
**  @brief Gets a handle to a sensor.
**
**  @param[in] sensor A sensor.
**  @param[out] handle Pointer to a sensor handle variable.
**
**  @retval MDV_RESULT_OK Handle successfully got.
**  @retval MDV_SENSORAPI_ERROR_INVALID_POINTER The driver or the handle 
**      pointer points to null.
**  @retval SENSOR_ERROR_NOT_INITIALIZED Driver not initialized.
*/
MdvResult_t
mdv_sensor_get_handle(
        MdvSensor_t *sensor,
        MdvHandle_t *handle
);

/*-------------------------------------------------------------------------*//**
**  @brief Sets calibration parameters.
**
**  @param[in] handle Handle to a sensor.
**  @param[in] calibrationParams Pointer to sensor specific calibration
**      parameters.
**
**  @retval MDV_RESULT_OK Successful.
**  @retval SENSOR_ERROR_INVALID_HANDLE Invalid sensor handle.
**  @retval MDV_SENSORAPI_ERROR_INVALID_POINTER The calibrationParams pointer points
**      to null.
**  @retval SENSOR_ERROR_NOT_IMPLEMENTED Sensor doesn't feature calibration.
*/
MdvResult_t
mdv_sensor_set_calibration_parameters(
        MdvHandle_t handle,
        void *calibrationParams
);

/*-------------------------------------------------------------------------*//**
**  @brief Starts a sensor calibration routine.
**
**  @param[in] handle Handle to a sensor.
**
**  @retval MDV_RESULT_OK Successful.
**  @retval SENSOR_ERROR_INVALID_HANDLE Invalid sensor handle.
**  @retval SENSOR_ERROR_NOT_IMPLEMENTED Sensor doesn't feature calibration.
*/
MdvResult_t
mdv_sensor_start_calibration(
        MdvHandle_t handle
);

/*-------------------------------------------------------------------------*//**
**  @brief Gets sensor calibration state.
**
**  @param[in] handle Handle to a sensor.
**  @param[out] calibrationState Sensor specific calibration state.
**
**  @retval MDV_RESULT_OK Successful.
**  @retval SENSOR_ERROR_INVALID_HANDLE Invalid sensor handle.
**  @retval MDV_SENSORAPI_ERROR_INVALID_POINTER The calibrationState pointer points
**      to null.
**  @retval SENSOR_ERROR_NOT_IMPLEMENTED Sensor doesn't feature calibration.
*/
MdvResult_t
mdv_sensor_get_calibration_state(
        MdvHandle_t handle,
        void *calibrationState
);

/*-------------------------------------------------------------------------*//**
**  @brief Gets sensor calibration data.
**
**  @param[in] handle Handle to a sensor.
**  @param[out] calibrationData Pointer to sensor specific calibration data.
**
**  @retval MDV_RESULT_OK Successful.
**  @retval SENSOR_ERROR_INVALID_HANDLE Invalid sensor handle.
**  @retval MDV_SENSORAPI_ERROR_INVALID_POINTER The calibrationData pointer points
**      to null.
**  @retval SENSOR_ERROR_NOT_IMPLEMENTED Sensor doesn't feature calibration.
*/
MdvResult_t
mdv_sensor_get_calibration_data(
        MdvHandle_t handle,
        void *calibrationData
);

/*-------------------------------------------------------------------------*//**
**  @brief Sets sensor calibration data.
**
**  @param[in] handle Handle to a sensor.
**  @param[in] calibrationData Pointer to sensor specific calibration data.
**
**  @retval MDV_RESULT_OK Successful.
**  @retval SENSOR_ERROR_INVALID_HANDLE Invalid sensor handle.
**  @retval MDV_SENSORAPI_ERROR_INVALID_POINTER The calibrationData pointer points
**      to null.
**  @retval SENSOR_ERROR_INVALID_PARAMETER Calibration data is wrong.
**  @retval SENSOR_ERROR_NOT_IMPLEMENTED Sensor doesn't feature calibration.
**
**  Turns the sensor status to "calibrated".
*/
MdvResult_t
mdv_sensor_set_calibration_data(
        MdvHandle_t handle,
        void *calibrationData
);

/*-------------------------------------------------------------------------*//**
**  @brief Gets the sensor calibration status.
**
**  @param[in] handle Handle to a sensor.
**
**  @retval true Sensor is calibrated.
**  @retval false Sensor is not calibrated, or the handle is invalid.
*/
bool
mdv_sensor_is_calibrated(
        MdvHandle_t handle
);

/*-------------------------------------------------------------------------*//**
**  @brief Gets the sensor configuration data.
**
**  @param[in] handle Handle to a sensor.
**  @param[in] configuration Pointer to sensor specific configuration data.
**
**  @retval MDV_RESULT_OK Successful.
**  @retval SENSOR_ERROR_INVALID_HANDLE Invalid sensor handle.
**  @retval MDV_SENSORAPI_ERROR_INVALID_POINTER The configuration pointer points to
**      null.
**  @retval SENSOR_ERROR_NOT_IMPLEMENTED Sensor doesn't feature configuration.
*/
MdvResult_t
mdv_sensor_get_configuration(
        MdvHandle_t handle,
        void *configuration
);

/*-------------------------------------------------------------------------*//**
**  @brief Sets the sensor configuration data.
**
**  @param[in] handle Handle to a sensor.
**  @param[in] configuration Pointer to sensor specific configuration data.
**
**  @retval MDV_RESULT_OK Successful.
**  @retval SENSOR_ERROR_INVALID_HANDLE Invalid sensor handle.
**  @retval MDV_SENSORAPI_ERROR_INVALID_POINTER The configuration pointer points to
**      null.
**  @retval SENSOR_ERROR_INVALID_PARAMETER Configuration data is wrong.
**  @retval SENSOR_ERROR_NOT_IMPLEMENTED Sensor doesn't feature configuration.
*/
MdvResult_t
mdv_sensor_set_configuration(
        MdvHandle_t handle,
        void *configuration
);

/*-------------------------------------------------------------------------*//**
**  @brief Controls a sensor.
**
**  @param[in] handle Handle to a sensor.
**  @param[in] control Sensor control value.
**
**  @return No return value.
**
**  If the sensor handle is invalid, does nothing.
*/
void
mdv_sensor_control(
        MdvHandle_t handle,
        MdvSensorControl_t control
);

/*-------------------------------------------------------------------------*//**
**  @brief Gets the driver control status.
**
**  @param[in] handle Handle to a sensor.
**
**  @retval Sensor control status. Returns MDV_SENSOR_DISABLE if the handle is
**          invalid.
*/
MdvSensorControl_t
mdv_sensor_status(
        MdvHandle_t handle
);

/*-------------------------------------------------------------------------*//**
**  @brief Runs a sensor.
**
**  @param[in] handle Handle to a sensor.
**
**  @return No return value.
**
**  This function can be used instead of the Sensor_RunDriver if the sensors
**  need to be run in separate threads or processes.
*/
void
mdv_sensor_run(
        MdvHandle_t handle
);

#endif // ifndef mdv_sensor_api_H

/* EOF */