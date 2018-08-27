/***************************************************************************//**
**
**  @file       sensor_api.c
**  @ingroup    sensor_framework
**  @brief      Sensor framework implementation
**  @copyright  (C) 2012-2018 Tuomas Terho
**
*******************************************************************************/
/*
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

#include "sensor_api.h"
#include <string.h>

/******************************************************************************\
**
**  PRIVATE DATA DECLARATIONS
**
\******************************************************************************/

static SensorDrv_t *sensorDriver[SENSOR_COUNT]={0};

/******************************************************************************\
**
**  PUBLIC FUNCTION DECLARATIONS
**
\******************************************************************************/

/*------------------------------------------------------------------------------
**  Initializes a sensor driver.
*/
void
SensorAPI_InitDriver(
        SensorDrv_t *driver,
        Sensor_t sensor,
        SensorDrv_Init_t funcInit,
        SensorDrv_SetCalibrationParams_t funcSetCalibrationParams,
        SensorDrv_StartCalibration_t funcStartCalibration,
        SensorDrv_GetCalibrationState_t funcGetCalibrationState,
        SensorDrv_GetCalibrationData_t funcGetCalibrationData,
        SensorDrv_SetCalibrationData_t funcSetCalibrationData,
        SensorDrv_Configure_t funcConfigure,
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
        void *instance
)
{
        // Map the driver with the sensor.
        sensorDriver[sensor]=driver;
        // Initialize the driver function interface.
        driver->Init=funcInit;
        driver->SetCalibrationParams=funcSetCalibrationParams;
        driver->StartCalibration=funcStartCalibration;
        driver->GetCalibrationState=funcGetCalibrationState;
        driver->GetCalibrationData=funcGetCalibrationData;
        driver->SetCalibrationData=funcSetCalibrationData;
        driver->Configure=funcConfigure;
        driver->SetInput=funcSetInput;
        driver->GetOutput=funcGetOutput;
        driver->On=funcOn;
        driver->Off=funcOff;
        driver->Run=funcRun;
        driver->ds.idc=inputDataItemCount;
        driver->ds.odc=outputDataItemCount;
        driver->ds.ids=inputDataSet;
        driver->ds.ods=outputDataSet;        
        // Initialize driver data. 
        driver->dd.icbk=inputCallback;
        driver->dd.ocbk=outputCallback;
        driver->dd.ccbk=ctrlCmdCallback;
        driver->dd.ctrl=SENSOR_DISABLE;
        driver->dd.rreq=false;
        driver->dd.inst=instance;
        // If the driver supports calibration, the initial calibration state is
        // false. Otherwise it is true (no calibration required).
        driver->dd.cal=driver->SetCalibrationData?false:true;
        // Initialize driver hardware.
        driver->Init(driver->dd.inst);
}

/*------------------------------------------------------------------------------
**  Gets a handle to a sensor.
*/
Result_t
SensorAPI_GetHandle(
        Sensor_t sensor,
        Handle_t *sensorHndl
)
{
        SensorDrv_t *drv;

        drv=sensorDriver[sensor];
        if(!drv){
                return SENSORAPI_ERROR_NOT_INITIALIZED;
        }
        *sensorHndl=drv;
        return RESULT_OK;
}

/*------------------------------------------------------------------------------
**  Sets calibration parameters.
*/
Result_t
SensorAPI_SetCalibrationParams(
        Handle_t sensorHndl,
        void *calibrationParams
)
{
        SensorDrv_t *drv;

        if(!sensorHndl){
                return SENSORAPI_ERROR_INVALID_HANDLE;
        }
        drv=(SensorDrv_t*)sensorHndl;
        if(!drv->SetCalibrationParams){
                return SENSORAPI_ERROR_NOT_IMPLEMENTED;
        }
        drv->SetCalibrationParams(drv->dd.inst,calibrationParams);
        return RESULT_OK;
}

/*------------------------------------------------------------------------------
**  Starts a sensor calibration routine.
*/
Result_t
SensorAPI_StartCalibration(
        Handle_t sensorHndl
)
{
        SensorDrv_t *drv;

        if(!sensorHndl){
                return SENSORAPI_ERROR_INVALID_HANDLE;
        }
        drv=(SensorDrv_t*)sensorHndl;
        if(!drv->StartCalibration){
                return SENSORAPI_ERROR_NOT_IMPLEMENTED;
        }
        drv->StartCalibration(drv->dd.inst);
        return RESULT_OK;
}

/*------------------------------------------------------------------------------
**  Gets sensor calibration state.
*/
Result_t
SensorAPI_GetCalibrationState(
        Handle_t sensorHndl,
        void *calibrationState
)
{
        SensorDrv_t *drv;

        if(!sensorHndl){
                return SENSORAPI_ERROR_INVALID_HANDLE;
        }
        drv=(SensorDrv_t*)sensorHndl;
        if(!drv->GetCalibrationState){
                return SENSORAPI_ERROR_NOT_IMPLEMENTED;
        }
        return drv->GetCalibrationState(drv->dd.inst,calibrationState);
}

/*------------------------------------------------------------------------------
**  Gets sensor calibration data.
*/
Result_t
SensorAPI_GetCalibrationData(
        Handle_t sensorHndl,
        void *calibrationData
)
{
        SensorDrv_t *drv;

        if(!sensorHndl){
                return SENSORAPI_ERROR_INVALID_HANDLE;
        }
        drv=(SensorDrv_t*)sensorHndl;
        if(!drv->GetCalibrationData){
                return SENSORAPI_ERROR_NOT_IMPLEMENTED;
        }
        drv->GetCalibrationData(drv->dd.inst,calibrationData);
        return RESULT_OK;
}

/*------------------------------------------------------------------------------
**  Sets sensor calibration data for a previously calibrated sensor.
*/
Result_t
SensorAPI_SetCalibrationData(
        Handle_t sensorHndl,
        void *calibrationData
)
{
        SensorDrv_t *drv;

        if(!sensorHndl){
                return SENSORAPI_ERROR_INVALID_HANDLE;
        }
        drv=(SensorDrv_t*)sensorHndl;
        if(!drv->SetCalibrationData){
                return SENSORAPI_ERROR_NOT_IMPLEMENTED;
        }
        drv->SetCalibrationData(drv->dd.inst,calibrationData);
        drv->dd.cal=true;
        return RESULT_OK;
}

/*------------------------------------------------------------------------------
**  Gets the sensor calibration status.
*/
bool
SensorAPI_IsCalibrated(
        Handle_t sensorHndl
)
{
        SensorDrv_t *drv;

        if(!sensorHndl){
                return false;
        }
        drv=(SensorDrv_t*)sensorHndl;
        return drv->dd.cal;
}

/*------------------------------------------------------------------------------
**  Configures a sensor.
*/
Result_t
SensorAPI_Configure(
        Handle_t sensorHndl,
        void *configuration
)
{
        SensorDrv_t *drv;

        if(!sensorHndl){
                return SENSORAPI_ERROR_INVALID_HANDLE;
        }
        drv=(SensorDrv_t*)sensorHndl;
        if(!drv->Configure){
                return SENSORAPI_ERROR_NOT_IMPLEMENTED;
        }
        drv->dd.rreq=drv->Configure(drv->dd.inst,configuration);
        return RESULT_OK;
}

/*------------------------------------------------------------------------------
**  Controls a sensor.
*/
void
SensorAPI_Control(
        Handle_t sensorHndl,
        Sensor_Control_t control
)
{
        SensorDrv_t *drv;
        uint8_t odi;
        var_t od={0};

        if(!sensorHndl){
                return;
        }
        drv=(SensorDrv_t*)sensorHndl;
        if(drv->dd.ctrl==control){
                return;
        }
        drv->dd.ctrl=control;
        if(control==SENSOR_ENABLE){
                drv->dd.rreq=true;
                if(drv->On){
                        drv->On(drv->dd.inst);
                }
        }else{
                if(drv->Off){
                        drv->Off(drv->dd.inst);
                }
                // Reset all sensor data.
                for(odi=0;odi<drv->ds.odc;odi++){
                        drv->dd.ocbk(odi,od);
                }
        }
}

/*------------------------------------------------------------------------------
**  Gets the driver control status.
*/
Sensor_Control_t
SensorAPI_Status(
        Handle_t sensorHndl
)
{
        SensorDrv_t *drv;

        if(!sensorHndl){
                return SENSOR_DISABLE;
        }
        drv=(SensorDrv_t*)sensorHndl;
        return drv->dd.ctrl;
}

/*------------------------------------------------------------------------------
**  Runs a sensor.
*/
void
SensorAPI_Run(
        Handle_t sensorHndl
)
{
        SensorDrv_t *drv;
        var_t id;
        uint8_t idi;
        Sensor_Output_t *od;
        uint8_t odi;

        if(!sensorHndl){
                return;
        }
        drv=(SensorDrv_t*)sensorHndl;
        if(!drv->dd.icbk||!drv->dd.ocbk){
                // Driver hasn't been initialized correctly.
                return;
        }
        // Go through the input data set and request data for each item
        // in the set.
        for(idi=0;idi<drv->ds.idc;idi++){
                // Request sensor input data from the user application 
                // by using a callback.
                id=drv->dd.icbk(idi);
                // Set input data to the sensor.
                drv->SetInput(drv->dd.inst,idi,id);
        }
        // Run the sensor driver.
        if(drv->Run){
                drv->Run(drv->dd.inst);
        }
        // Go through the output data set and send out each item in the
        // set if needed.
        for(odi=0;odi<drv->ds.odc;odi++){
                od=drv->GetOutput(drv->dd.inst,odi);
                if(!od){
                        // Unsupported output id.
                        continue;
                }
                if(od->rc){
                        // Output refresh cycle is in use.
                        // Check the cycle timer.
                        if(Timer_Diff(od->tim)<od->rc){
                                // Timer is still running.
                                continue;
                        }
                        // Cycle timer timeout.
                        od->tim=Timer_Get();
                }
                // Check sensor data status.
                if(od->st!=SENSOR_DATA_OK){
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
                if(drv->dd.ctrl==SENSOR_ENABLE){
                        // Invoke the output data event callback.
                        drv->dd.ocbk(odi,od->data);
                }
        }
        // Reset the data report request.
        drv->dd.rreq=false;
}

/*------------------------------------------------------------------------------
**  Runs all sensors.
*/
void
SensorAPI_RunAll(
        void
)
{
        Sensor_t s;

        for(s=(Sensor_t)0;s<SENSOR_COUNT;s++){
                SensorAPI_Run((Handle_t)sensorDriver[s]);
        }
}

/******************************************************************************\
**
**  DRIVER HELPER FUNCTION DECLARATIONS
**
\******************************************************************************/

/*------------------------------------------------------------------------------
**  Sets output status and maintains the output changed state.
*/
void
SensorDrv_SetOutputStatus(
        Sensor_Output_t *output,
        Sensor_OutputStatus_t status
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

/*------------------------------------------------------------------------------
**  Sets output data and maintains the output changed state.
*/
void
SensorDrv_SetOutputData(
        Sensor_Output_t *output,
        var_t data
)
{
        if(!memcmp(&output->data,&data,sizeof(var_t))){
                // Data doesn't change.
                return;
        }
        // New data.
        output->data=data;
        // Data changed flag.
        output->ci=true;
}

/* EOF */