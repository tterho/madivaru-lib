/***************************************************************************//**
**
**  @file       mdv_sensor_api.c
**  @ingroup    madivaru-lib
**  @brief      Sensor framework implementation
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

#include "mdv_sensor_api.h"

#include <string.h>

/******************************************************************************\
**
**  DRIVER HELPER API FUNCTION DEFINITIONS
**
\******************************************************************************/

void
mdv_sensor_set_output_status(
        MdvSensorOutput_t *output,
        MdvSensorOutputDataStatus_t status
)
{
        if(output->st==status){
                // Status doesn't change.
                return;
        }
        // New status.
        output->st=status;
        // Status changed flag.
        output->ci=true;
}

void
mdv_sensor_set_output_data(
        MdvSensorOutput_t *output,
        MdvVar_t data
)
{
        if(!memcmp(&output->data,&data,sizeof(MdvVar_t))){
                // Data doesn't change.
                return;
        }
        // New data.
        output->data=data;
        // Data changed flag.
        output->ci=true;
}

/******************************************************************************\
**
**  SENSOR API FUNCTION DEFINITIONS
**
\******************************************************************************/

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
        MdvTimerSystem_t *tsys,
        void *userData
)
{
        // Initialize the sensor function interface.
        sensor->Init=funcInit;
        sensor->SetCalibrationParams=funcSetCalibrationParams;
        sensor->StartCalibration=funcStartCalibration;
        sensor->GetCalibrationState=funcGetCalibrationState;
        sensor->GetCalibrationData=funcGetCalibrationData;
        sensor->SetCalibrationData=funcSetCalibrationData;
        sensor->GetConfiguration=funcGetConfiguration;
        sensor->SetConfiguration=funcSetConfiguration;
        sensor->SetInput=funcSetInput;
        sensor->GetOutput=funcGetOutput;
        sensor->On=funcOn;
        sensor->Off=funcOff;
        sensor->Run=funcRun;
        sensor->ds.idc=inputDataItemCount;
        sensor->ds.odc=outputDataItemCount;
        sensor->ds.ids=inputDataSet;
        sensor->ds.ods=outputDataSet;
        // Initialize sensor data.
        sensor->dd.icbk=inputCallback;
        sensor->dd.ocbk=outputCallback;
        sensor->dd.ccbk=ctrlCmdCallback;
        sensor->dd.ctrl=MDV_SENSOR_DISABLE;
        sensor->dd.rreq=false;
        sensor->dd.ud=userData;
        // If the sensor supports calibration, the initial calibration state is
        // false. Otherwise it is true (no calibration required).
        sensor->dd.cal=sensor->SetCalibrationData?false:true;
        // Initialize sensor hardware.
        sensor->Init(sensor->dd.ud);
}

MdvResult_t
mdv_sensor_get_handle(
        MdvSensor_t *sensor,
        MdvHandle_t *handle
)
{
        if(!sensor||!handle){
                return MDV_SENSORAPI_ERROR_INVALID_POINTER;
        }
        *handle=(MdvHandle_t)sensor;
        return MDV_RESULT_OK;
}

MdvResult_t
mdv_sensor_set_calibration_parameters(
        MdvHandle_t handle,
        void *calibrationParams
)
{
        MdvSensor_t *drv;

        if(!handle){
                return MDV_SENSORAPI_ERROR_INVALID_PARAMETER;
        }
        if(!calibrationParams){
                return MDV_SENSORAPI_ERROR_INVALID_POINTER;
        }
        drv=(MdvSensor_t*)handle;
        if(!drv->SetCalibrationParams){
                return MDV_SENSORAPI_ERROR_NOT_IMPLEMENTED;
        }
        drv->SetCalibrationParams(calibrationParams,drv->dd.ud);
        return MDV_RESULT_OK;
}

MdvResult_t
mdv_sensor_start_calibration(
        MdvHandle_t handle
)
{
        MdvSensor_t *drv;

        if(!handle){
                return MDV_SENSORAPI_ERROR_INVALID_PARAMETER;
        }
        drv=(MdvSensor_t*)handle;
        if(!drv->StartCalibration){
                return MDV_SENSORAPI_ERROR_NOT_IMPLEMENTED;
        }
        drv->StartCalibration(drv->dd.ud);
        return MDV_RESULT_OK;
}

MdvResult_t
mdv_sensor_get_calibration_state(
        MdvHandle_t handle,
        void *calibrationState
)
{
        MdvSensor_t *drv;

        if(!handle){
                return MDV_SENSORAPI_ERROR_INVALID_PARAMETER;
        }
        if(!calibrationState){
                return MDV_SENSORAPI_ERROR_INVALID_POINTER;
        }
        drv=(MdvSensor_t*)handle;
        if(!drv->GetCalibrationState){
                return MDV_SENSORAPI_ERROR_NOT_IMPLEMENTED;
        }
        return drv->GetCalibrationState(calibrationState,drv->dd.ud);
}

MdvResult_t
mdv_sensor_get_calibration_data(
        MdvHandle_t handle,
        void *calibrationData
)
{
        MdvSensor_t *drv;

        if(!handle){
                return MDV_SENSORAPI_ERROR_INVALID_PARAMETER;
        }
        if(!calibrationData){
                return MDV_SENSORAPI_ERROR_INVALID_POINTER;
        }
        drv=(MdvSensor_t*)handle;
        if(!drv->GetCalibrationData){
                return MDV_SENSORAPI_ERROR_NOT_IMPLEMENTED;
        }
        drv->GetCalibrationData(calibrationData,drv->dd.ud);
        return MDV_RESULT_OK;
}

MdvResult_t
mdv_sensor_set_calibration_data(
        MdvHandle_t handle,
        void *calibrationData
)
{
        MdvSensor_t *drv;

        if(!handle){
                return MDV_SENSORAPI_ERROR_INVALID_PARAMETER;
        }
        if(!calibrationData){
                return MDV_SENSORAPI_ERROR_INVALID_POINTER;
        }
        drv=(MdvSensor_t*)handle;
        if(!drv->SetCalibrationData){
                return MDV_SENSORAPI_ERROR_NOT_IMPLEMENTED;
        }
        drv->SetCalibrationData(calibrationData,drv->dd.ud);
        drv->dd.cal=true;
        return MDV_RESULT_OK;
}

bool
mdv_sensor_is_calibrated(
        MdvHandle_t handle
)
{
        MdvSensor_t *drv;

        if(!handle){
                return false;
        }
        drv=(MdvSensor_t*)handle;
        return drv->dd.cal;
}

MdvResult_t
mdv_sensor_get_configuration(
        MdvHandle_t handle,
        void *configuration
)
{
        MdvSensor_t *drv;

        if(!handle){
                return MDV_SENSORAPI_ERROR_INVALID_PARAMETER;
        }
        if(!configuration){
                return MDV_SENSORAPI_ERROR_INVALID_POINTER;
        }
        drv=(MdvSensor_t*)handle;
        if(!drv->GetConfiguration){
                return MDV_SENSORAPI_ERROR_NOT_IMPLEMENTED;
        }
        drv->GetConfiguration(configuration,drv->dd.ud);
        return MDV_RESULT_OK;
}

MdvResult_t
mdv_sensor_set_configuration(
        MdvHandle_t handle,
        void *configuration
)
{
        MdvSensor_t *drv;

        if(!handle){
                return MDV_SENSORAPI_ERROR_INVALID_PARAMETER;
        }
        if(!configuration){
                return MDV_SENSORAPI_ERROR_INVALID_POINTER;
        }
        drv=(MdvSensor_t*)handle;
        if(!drv->SetConfiguration){
                return MDV_SENSORAPI_ERROR_NOT_IMPLEMENTED;
        }
        drv->dd.rreq=drv->SetConfiguration(configuration,drv->dd.ud);
        return MDV_RESULT_OK;
}

void
mdv_sensor_control(
        MdvHandle_t handle,
        MdvSensorControl_t control
)
{
        MdvSensor_t *drv;
        uint8_t odi;
        MdvVar_t od={0};

        if(!handle){
                return;
        }
        drv=(MdvSensor_t*)handle;
        if(drv->dd.ctrl==control){
                return;
        }
        drv->dd.ctrl=control;
        switch(control){
        default:
        case MDV_SENSOR_DISABLE:
                // Reset all sensor data in the user application.
                for(odi=0;odi<drv->ds.odc;odi++){
                        drv->dd.ocbk(odi,od);
                }
                break;
        case MDV_SENSOR_ENABLE:
                drv->dd.rreq=true;
                if(drv->On){
                        drv->On(drv->dd.ud);
                }
                break;
        case MDV_SENSOR_HALT:
                if(drv->Off){
                        drv->Off(drv->dd.ud);
                }
                // Reset all sensor data in the user application.
                for(odi=0;odi<drv->ds.odc;odi++){
                        drv->dd.ocbk(odi,od);
                }
                break;
        }
}

MdvSensorControl_t
mdv_sensor_status(
        MdvHandle_t handle
)
{
        MdvSensor_t *drv;

        if(!handle){
                return MDV_SENSOR_HALT;
        }
        drv=(MdvSensor_t*)handle;
        return drv->dd.ctrl;
}

void
mdv_sensor_run(
        MdvHandle_t handle
)
{
        MdvSensor_t *drv;
        MdvVar_t id;
        uint8_t idi;
        MdvSensorOutput_t *od;
        uint8_t odi;
        uint32_t tl;

        if(!handle){
                return;
        }
        drv=(MdvSensor_t*)handle;
        if(!drv->dd.icbk||!drv->dd.ocbk){
                // Driver hasn't been initialized correctly.
                return;
        }
        // If the sensor is halted, do nothing.
        if(drv->dd.ctrl==MDV_SENSOR_HALT){
                return;
        }
        // Go through the input data set and request data for each item
        // in the set.
        for(idi=0;idi<drv->ds.idc;idi++){
                // Request sensor input data from the user application
                // by using a callback.
                id=drv->dd.icbk(idi);
                // Set input data to the sensor.
                drv->SetInput(idi,id,drv->dd.ud);
        }
        // Run the sensor driver.
        if(drv->Run){
                drv->Run(drv->dd.ud);
        }
        // Go through the output data set and send out each item in the
        // set if needed.
        for(odi=0;odi<drv->ds.odc;odi++){
                od=drv->GetOutput(odi,drv->dd.ud);
                if(!od){
                        // Unsupported output id.
                        continue;
                }
                if(od->rc){
                        // Output refresh cycle is in use.
                        // Check the cycle timer.
                        mdv_timer_get_time(
                            drv->dd.tsys,
                            od->tim,
                            MDV_TIME_UNIT_MS,
                            &tl
                        );
                        if(tl<od->rc){
                                // Timer is still running.
                                continue;
                        }
                        // Cycle timer timeout occurred.
                        // Restart the timer.
                        mdv_timer_start(drv->dd.tsys,&od->tim);
                }
                // Check sensor data status.
                if(od->st!=MDV_SENSOR_OUTPUT_DATA_OK){
                        // Data not valid.
                        continue;
                }
                // Check data change status / data request.
                if(!od->ci&&!drv->dd.rreq){
                        // Data not changed and report not requested.
                        continue;
                }
                // Reset the data-changed flag.
                od->ci=false;
                if(drv->dd.ctrl==MDV_SENSOR_ENABLE){
                        // Invoke the output data event callback.
                        drv->dd.ocbk(odi,od->data);
                }
        }
        // Reset the data report request.
        drv->dd.rreq=false;
}

/* EOF */