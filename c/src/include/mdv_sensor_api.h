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
#include "mdv_handle.h"
#include "mdv_driver.h"

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
typedef MdvVar_t
(MdvSensorInputCallback_t)(
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
typedef void
(MdvSensorOutputCallback_t)(
        uint8_t index,
        MdvVar_t data
);

/*-------------------------------------------------------------------------*//**
**  @brief Sensor control command callback.
**
**  @param[in] cmdId The ID of the calibration command.
**  @param[in] data Data related to the command.
**
**  @return Caller may require data returned by the callback (see the
**          description of the particular command for details).
**
**  Sensor framework invokes this type of callback when the sensor needs to have
**  control to a certain function in the user application, e.g. to adjust PWM,
**  control I/O etc. The use of this callback depends on the requirements of the
**  driver.
**
**  @remarks This is an optional function which may be implemented in the user
**           application separately for each sensor supporting control commands.
*/
typedef
MdvVar_t
(MdvSensorControlCommandCallback_t)(
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
        /// @brief Turns a sensor off.
        MDV_SENSOR_DISABLE=0,
        /// @brief Turns a sensor on.
        MDV_SENSOR_ENABLE,
        /// @brief Halts a sensor.
        MDV_SENSOR_HALT
} MdvSensorControl_t;

/*-------------------------------------------------------------------------*//**
**  @brief Sensor output data status.
**
**  Sensor output data status indicates the validity of the sensor data.
*/
typedef enum
MdvSensorOutputDataStatus_t{
        /// @brief Sensor data is not valid, for example, sensor initialization
        ///        hasn't been completed yet.
        MDV_SENSOR_OUTPUT_DATA_UNKNOWN,
        /// @brief There is a failure in reading the sensor data, thus the data
        ///        is not valid.
        MDV_SENSOR_OUTPUT_DATA_FAILURE,
        /// @brief Sensor data is valid.
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
        /// @brief Status of the sensor output.
        /// @remarks This value is set by a sensor driver by using the function
        ///          @ref mdv_sensor_set_output_status.
        MdvSensorOutputDataStatus_t st;
        /// @brief Indicator for changes in data and status.
        /// @remarks This value is set by functions
        ///          @ref mdv_sensor_set_output_status and
        ///          @ref mdv_sensor_set_output_data. The value is resetted
        ///          automatically by the API when the change is handled in the
        ///          function @ref mdv_sensor_run. Do not write this value
        ///          manually.
        bool ci;
        /// @brief Output data.
        /// @remarks This value is set by a sensor driver by using the function
        ///          @ref mdv_sensor_set_output_data.
        MdvVar_t data;
        /// @brief Data refresh cycle timer.
        /// @remarks This value is used internally by the API. The value must be
        ///          initialized in the sensor driver's initialization function.
        MdvTimer_t tim;
        /// @brief Data refresh cycle length in milliseconds.
        /// @remarks This value is used internally by the API. The value must be
        ///          initialized in the sensor driver's initialization function.
        uint16_t rc;
} MdvSensorOutput_t;

/******************************************************************************\
**
**  DRIVER API FUNCTION DECLARATIONS
**
\******************************************************************************/

/*-------------------------------------------------------------------------*//**
**  @brief Sets calibration parameters.
**
**  @param[in] calibrationParams Pointer to sensor specific calibration
**             parameters.
**  @param[in] userData User defined data.
**
**  @return No return value.
**
**  This function is implemented in the sensor driver and the pointer to the
**  function is stored into the driver instance.
**
**  @remarks This function is optional. If implemented, it shall copy the
**           calibration parameters to the instance of the driver.
*/
typedef void
(MdvSensorDriverInterfaceSetCalibrationParams_t)(
        void *const calibrationParams,
        void *const userData
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
**           if it supports calibration and requires initializations during
**           calibration startup.
*/
typedef void
(MdvSensorDriverInterfaceStartCalibration_t)(
        void *const userData
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
**           if the calibration is performed in steps. The states are driver
**           specific.
*/
typedef MdvResult_t
(MdvSensorDriverInterfaceGetCalibrationState_t)(
        void *const calibrationState,
        void *const userData
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
**           if it supports calibration.
*/
typedef void
(MdvSensorDriverInterfaceGetCalibrationData_t)(
        void *const calibrationData,
        void *const userData
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
**           if it supports calibration.
*/
typedef void
(MdvSensorDriverInterfaceSetCalibrationData_t)(
        void *const calibrationData,
        void *const userData
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
**           to support configuration. This function can be used to return the
**           current configuration state as well as the default configuration.
*/
typedef void
(MdvSensorDriverInterfaceGetConfiguration_t)(
        void *const configurationData,
        void *const userData
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
**           to support configuration. The function must perform comparison
**           between the new and old configuration data. It must return true if
**           the configuration data has changed or false otherwise.
*/
typedef bool
(MdvSensorDriverInterfaceSetConfiguration_t)(
        void *const configurationData,
        void *const userData
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
**           store input data to the input data set of the driver instance.
*/
typedef void
(MdvSensorDriverInterfaceSetInput_t)(
        uint8_t index,
        MdvVar_t inputData,
        void *const userData
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
**           return the output data from the output data set of the driver
**           instance. The driver may use this function to produce the output
**           data from the input data.
*/
typedef MdvSensorOutput_t *
(MdvSensorDriverInterfaceGetOutput_t)(
        uint8_t index,
        void *const userData
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
        /// @brief Driver common API.
        MdvDriver_t common;
        /// A function to set calibration parameters (optional).
        MdvSensorDriverInterfaceSetCalibrationParams_t setCalibrationParams;
        /// A function to start calibration sequence (optional).
        MdvSensorDriverInterfaceStartCalibration_t startCalibration;
        /// A function to get calibration state (optional).
        MdvSensorDriverInterfaceGetCalibrationState_t getCalibrationState;
        /// A function to get calibration data (optional).
        MdvSensorDriverInterfaceGetCalibrationData_t getCalibrationData;
        /// A function to set calibration data (optional).
        MdvSensorDriverInterfaceSetCalibrationData_t setCalibrationData;
        /// A function to get the sensor configuration data (optional).
        MdvSensorDriverInterfaceGetConfiguration_t getConfiguration;
        /// A function to set the sensor configuration data (optional).
        MdvSensorDriverInterfaceSetConfiguration_t setConfiguration;
        /// A function to set sensor input data (mandatory).
        MdvSensorDriverInterfaceSetInput_t setInput;
        /// A function to get sensor output data (mandatory).
        MdvSensorDriverInterfaceGetOutput_t getOutput;
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

#ifdef __cplusplus
extern "C"{
#endif // ifdef __cplusplus

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
**  @param[in] tsys Timer system to be used for timings.
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
        MdvSensorDriverInterfaceInit_t funcInit,
        MdvSensorDriverInterfaceSetCalibrationParams_t funcSetCalibrationParams,
        MdvSensorDriverInterfaceStartCalibration_t funcStartCalibration,
        MdvSensorDriverInterfaceGetCalibrationState_t funcGetCalibrationState,
        MdvSensorDriverInterfaceGetCalibrationData_t funcGetCalibrationData,
        MdvSensorDriverInterfaceSetCalibrationData_t funcSetCalibrationData,
        MdvSensorDriverInterfaceGetConfiguration_t funcGetConfiguration,
        MdvSensorDriverInterfaceSetConfiguration_t funcSetConfiguration,
        MdvSensorDriverInterfaceSetInput_t funcSetInput,
        MdvSensorDriverInterfaceGetOutput_t funcGetOutput,
        uint8_t inputDataItemCount,
        uint8_t outputDataItemCount,
        MdvVar_t *inputDataSet,
        MdvSensorOutput_t *outputDataSet,
        MdvSensorInputCallback_t inputCallback,
        MdvSensorOutputCallback_t outputCallback,
        MdvSensorControlCommandCallback_t ctrlCmdCallback,
        MdvTimerSystem_t *tsys,
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

#ifdef __cplusplus
}
#endif // ifdef __cplusplus

#endif // ifndef mdv_sensor_api_H

/* EOF */