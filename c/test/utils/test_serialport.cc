#include "gtest/gtest.h"
#include "timer.h"
#include "serialport.h"

/*
SERIAL PORT TESTS
Driver interface setup
OK        - Check mandatory parameters.
OK        - Setup the interface.

Initialization
OK        - Check mandatory parameters.
OK        - Check optional parameters.
OK        - Check prerequisities (driver interface initialized).
OK        - Setup the port.
OK        - Check port initialization status.
OK        - Call driver's Init function.

Getting the current configuration
OK        - Check mandatory parameters.
OK        - Get the current configuration.

Open a port
OK        - Check mandatory parameters.
OK        - Ensure unoccupied handle value.
OK        - Check port initialization status.
OK        - Configure the port.
OK        - Call driver's Open function.
OK        - Set valid handle value.

Close a port
OK        - Check mandatory parameters.
OK        - Cancel asynchronous transfers.
OK        - Call driver's Close function.
OK        - Clear the handle value.

Change configuration
OK        - Check mandatory parameters.
OK        - Cancel possible asynchronous transfers.
OK        - Close the port.
OK        - Change the port configuration.
OK        - Reopen the port.

Read data
OK        - Check mandatory parameters.
OK        - Prevent transfer if a transfer is already in progress.
OK        - Transfer turned on.
OK        - Transfer initialized.
OK        - Zero-length transfer.
OK        - Call driver's Read function.
OK        - Initial asynchronous read.
OK        - Synchronous transfer without timeout.
OK        - Synchronous transfer with timeout.
OK        - Return valid data length.

Write data
OK        - Check mandatory parameters.
OK        - Prevent transfer if a transfer is already in progress.
OK        - Transfer turned on.
OK        - Transfer initialized.
OK        - Zero-length transfer.
OK        - Call driver's Write function.
OK        - Initial asynchronous write.
OK        - Synchronous transfer without timeout.
OK        - Synchronous transfer with timeout.
OK        - Return valid data length.

Run asynchronous transfers
OK        - Check mandatory parameters.
OK        - Call driver's RunDriver function.
OK        - Run asynchronous reception.
OK        - Run asynchronous transmission.

TODO:

- Test GetChar
- Test PutChar
- Implement CancelTransfer

*/

namespace{

/* TEST FIXTURE */

class SERIALPORT : public ::testing::Test
{
        protected:

        void SetUp() override {
                result=RESULT_OK;
                handle=(Handle_t)-1;
                ud=0xDEADBEEF;

                memset(&port,0xFF,sizeof(SP_COMPort_t));
                port.init=bFalse;
                memset(&config,0xFF,sizeof(SP_Config_t));
                port.drvfuncInit=spInit;
                port.drvfuncOpen=spOpen;
                port.drvfuncClose=spClose;
                port.drvfuncRead=spRead;
                port.drvfuncWrite=spWrite;
                port.drvfuncRunDriver=spRunDriver;
                spInitReturnValue=RESULT_OK;
                spOpenReturnValue=RESULT_OK;
                spCloseReturnValue=RESULT_OK;
                spReadReturnValue=RESULT_OK;
                spWriteReturnValue=RESULT_OK;
                spRunDriverReturnValue=RESULT_OK;
                spRxCompletedCbkResult=RESULT_OK;
                spRxCompletedCbkUserData=0;
                spTxCompletedCbkResult=RESULT_OK;
                spTxCompletedCbkUserData=0;
                memset(data,0,128);
                bytesRead=0xFFFFFFFF;
                bytesWritten=0xFFFFFFFF;

                TimerAPI_Init(&tsys,1000,10000000);
        }

        void TearDown() override {

        }

        void SetupDriverInterface(SP_COMPort_t *_port){
                SP_SetupDriverInterface(
                        _port,
                        spInit,
                        spOpen,
                        spClose,
                        spRead,
                        spWrite,
                        spRunDriver
                );
        }

        void InitPort(SP_COMPort_t *_port){
                SP_InitPort(
                        _port,
                        spRxCompletedCbk,
                        spTxCompletedCbk,
                        &tsys,
                        TIMER_TU_MS,
                        &ud
                );
        }

        void OpenPort(Handle_t *_handle){
                *_handle=0;
                SetupDriverInterface(&port);
                InitPort(&port);
                SP_GetCurrentConfig(&port,&config);
                SP_Open(&port,&config,_handle);
        }

        SP_COMPort_t port;
        SP_Config_t config;
        Result_t result;
        Handle_t handle;
        uint32_t ud;
        uint8_t data[128];
        uint32_t bytesRead;
        uint32_t bytesWritten;

        static Result_t spInitReturnValue;
        static Result_t spOpenReturnValue;
        static Result_t spCloseReturnValue;
        static Result_t spReadReturnValue;
        static Result_t spWriteReturnValue;
        static Result_t spRunDriverReturnValue;
        static Result_t spRxCompletedCbkResult;
        static void *spRxCompletedCbkUserData;
        static Result_t spTxCompletedCbkResult;
        static void *spTxCompletedCbkUserData;
        static TimerSys_t tsys;

        // Driver mockup functions.

        static Result_t spInit(
                void
        )
        {
                return spInitReturnValue;
        }

        static Result_t spOpen(
                SP_Config_t *cfg
        )
        {
                return spOpenReturnValue;
        }
 
        static Result_t spClose(
                void
        )
        {
                return spCloseReturnValue;
        }

        static Result_t spRead(
                SP_Transfer_t *rxd
        )
        {
                return spReadReturnValue;
        }

        static Result_t spRead8bytes(
                SP_Transfer_t *rxd
        )
        {
                memset(rxd->data,0xAA,8);
                rxd->data+=8;

                if(rxd->tleft==8){
                        rxd->tleft=0;
                        return RESULT_OK;
                }
                rxd->tleft-=8;
                return SP_ERROR_RX_BUFFER_EMPTY;
        }

        static Result_t spReadTimeout(
                SP_Transfer_t *rxd
        )
        {
                if(rxd->tleft>2){
                        rxd->tleft--;
                        rxd->data++;
                }
                TimerAPI_TimerTick(&tsys,1);
                return SP_ERROR_RX_BUFFER_EMPTY;
        }

        static Result_t spWrite(
                SP_Transfer_t *txd
        )
        {
                return spWriteReturnValue;
        }

        static Result_t spWrite8bytes(
                SP_Transfer_t *txd
        )
        {
                txd->data+=8;

                if(txd->tleft==8){
                        txd->tleft=0;
                        return RESULT_OK;
                }
                txd->tleft-=8;
                return SP_ERROR_TX_BUFFER_FULL;
        }

        static Result_t spWriteTimeout(
                SP_Transfer_t *txd
        )
        {
                if(txd->tleft>2){
                        txd->tleft--;
                        txd->data++;
                }
                TimerAPI_TimerTick(&tsys,1);
                return SP_ERROR_RX_BUFFER_EMPTY;
        }

        static Result_t spRunDriver(
                SP_Transfer_t *rxd,
                SP_Transfer_t *txd
        )
        {
                return spRunDriverReturnValue;
        }

        static void
                spRxCompletedCbk(
                        Result_t result,
                        void *userData
                )
        {
                spRxCompletedCbkResult=result;
                spRxCompletedCbkUserData=userData;
        }

        static void
                spTxCompletedCbk(
                        Result_t result,
                        void *userData
                )
        {
                spTxCompletedCbkResult=result;
                spTxCompletedCbkUserData=userData;
        }

};

Result_t SERIALPORT::spInitReturnValue;
Result_t SERIALPORT::spOpenReturnValue;
Result_t SERIALPORT::spCloseReturnValue;
Result_t SERIALPORT::spReadReturnValue;
Result_t SERIALPORT::spWriteReturnValue;
Result_t SERIALPORT::spRunDriverReturnValue;
Result_t SERIALPORT::spRxCompletedCbkResult;
void *SERIALPORT::spRxCompletedCbkUserData;
Result_t SERIALPORT::spTxCompletedCbkResult;
void *SERIALPORT::spTxCompletedCbkUserData;
TimerSys_t SERIALPORT::tsys;

TEST_F(SERIALPORT,SP_SetupDriverInterface_RequiredParameter_port)
{
        result=SP_SetupDriverInterface(
                0,
                spInit,
                spOpen,
                spClose,
                spRead,
                spWrite,
                spRunDriver
        );

        EXPECT_EQ(SP_ERROR_INVALID_POINTER,result);
}

TEST_F(SERIALPORT,SP_SetupDriverInterface_RequiredParameter_funcInit)
{
        result=SP_SetupDriverInterface(
                &port,
                0,
                spOpen,
                spClose,
                spRead,
                spWrite,
                spRunDriver
        );

        EXPECT_EQ(SP_ERROR_INVALID_PARAMETER,result);
}

TEST_F(SERIALPORT,SP_SetupDriverInterface_RequiredParameter_funcOpen)
{
        result=SP_SetupDriverInterface(
                &port,
                spInit,
                0,
                spClose,
                spRead,
                spWrite,
                spRunDriver
        );

        EXPECT_EQ(SP_ERROR_INVALID_PARAMETER,result);
}

TEST_F(SERIALPORT,SP_SetupDriverInterface_RequiredParameter_funcClose)
{
        result=SP_SetupDriverInterface(
                &port,
                spInit,
                spOpen,
                0,
                spRead,
                spWrite,
                spRunDriver
        );

        EXPECT_EQ(SP_ERROR_INVALID_PARAMETER,result);
}

TEST_F(SERIALPORT,SP_SetupDriverInterface_RequiredParameter_funcRead)
{
        result=SP_SetupDriverInterface(
                &port,
                spInit,
                spOpen,
                spClose,
                0,
                spWrite,
                spRunDriver
        );

        EXPECT_EQ(SP_ERROR_INVALID_PARAMETER,result);
}

TEST_F(SERIALPORT,SP_SetupDriverInterface_RequiredParameter_funcWrite)
{
        result=SP_SetupDriverInterface(
                &port,
                spInit,
                spOpen,
                spClose,
                spRead,
                0,
                spRunDriver
        );

        EXPECT_EQ(SP_ERROR_INVALID_PARAMETER,result);
}

TEST_F(SERIALPORT,SP_SetupDriverInterface_OptionalParameter_funcRunDriver)
{
        result=SP_SetupDriverInterface(
                &port,
                spInit,
                spOpen,
                spClose,
                spRead,
                spWrite,
                0
        );

        EXPECT_EQ(RESULT_OK,result);
}

TEST_F(SERIALPORT,SP_SetupDriverInterface_InterfaceInitialization)
{
        port.drvfuncInit=0;
        port.drvfuncOpen=0;
        port.drvfuncClose=0;
        port.drvfuncRead=0;
        port.drvfuncWrite=0;
        port.drvfuncRunDriver=0;

        SP_SetupDriverInterface(
                &port,
                spInit,
                spOpen,
                spClose,
                spRead,
                spWrite,
                spRunDriver
        );

        EXPECT_EQ((SPDrv_Func_Init_t)spInit,port.drvfuncInit);
        EXPECT_EQ((SPDrv_Func_Open_t)spOpen,port.drvfuncOpen);
        EXPECT_EQ((SPDrv_Func_Close_t)spClose,port.drvfuncClose);
        EXPECT_EQ((SPDrv_Func_Transfer_t)spRead,port.drvfuncRead);
        EXPECT_EQ((SPDrv_Func_Transfer_t)spWrite,port.drvfuncWrite);
        EXPECT_EQ((SPDrv_Func_RunDriver_t)spRunDriver,port.drvfuncRunDriver);
}

TEST_F(SERIALPORT,SP_Init_RequiredParameter_port)
{
        result=SP_InitPort(
                0,
                (SP_TransferCompletedCbk_t)0x1,
                (SP_TransferCompletedCbk_t)0x1,
                (TimerSys_t*)0x1,
                (Timer_TimeUnit_t)0x1,
                (void*)0x1
        );

        EXPECT_EQ(SP_ERROR_INVALID_POINTER,result);
}

TEST_F(SERIALPORT,SP_Init_OptionalParameter_rxCompleted)
{
        result=SP_InitPort(
                &port,
                0,
                (SP_TransferCompletedCbk_t)0x1,
                (TimerSys_t*)0x1,
                (Timer_TimeUnit_t)0x1,
                (void*)0x1
        );

        EXPECT_EQ(RESULT_OK,result);
}

TEST_F(SERIALPORT,SP_Init_OptionalParameter_txCompleted)
{
        result=SP_InitPort(
                &port,
                (SP_TransferCompletedCbk_t)0x1,
                0,
                (TimerSys_t*)0x1,
                (Timer_TimeUnit_t)0x1,
                (void*)0x1
        );

        EXPECT_EQ(RESULT_OK,result);
}

TEST_F(SERIALPORT,SP_Init_OptionalParameter_timerSys)
{
        result=SP_InitPort(
                &port,
                (SP_TransferCompletedCbk_t)0x1,
                (SP_TransferCompletedCbk_t)0x1,
                0,
                (Timer_TimeUnit_t)0x1,
                (void*)0x1
        );

        EXPECT_EQ(RESULT_OK,result);
}

TEST_F(SERIALPORT,SP_Init_OptionalParameter_timeUnit)
{
        result=SP_InitPort(
                &port,
                (SP_TransferCompletedCbk_t)0x1,
                (SP_TransferCompletedCbk_t)0x1,
                (TimerSys_t*)0x1,
                (Timer_TimeUnit_t)0,
                (void*)0x1
        );

        EXPECT_EQ(RESULT_OK,result);
}

TEST_F(SERIALPORT,SP_Init_OptionalParameter_userData)
{
        result=SP_InitPort(
                &port,
                (SP_TransferCompletedCbk_t)0x1,
                (SP_TransferCompletedCbk_t)0x1,
                (TimerSys_t*)0x1,
                (Timer_TimeUnit_t)0x1,
                0
        );

        EXPECT_EQ(RESULT_OK,result);
}

TEST_F(SERIALPORT,SP_Init_PortFunctionPointersUntouched)
{
        SP_InitPort(&port,0,0,0,(Timer_TimeUnit_t)0,0);

        EXPECT_EQ((SPDrv_Func_Init_t)spInit,port.drvfuncInit);
        EXPECT_EQ((SPDrv_Func_Open_t)spOpen,port.drvfuncOpen);
        EXPECT_EQ((SPDrv_Func_Close_t)spClose,port.drvfuncClose);
        EXPECT_EQ((SPDrv_Func_Transfer_t)spRead,port.drvfuncRead);
        EXPECT_EQ((SPDrv_Func_Transfer_t)spWrite,port.drvfuncWrite);
        EXPECT_EQ((SPDrv_Func_RunDriver_t)spRunDriver,port.drvfuncRunDriver);
}

TEST_F(SERIALPORT,SP_Init_DriverInterfaceNotInitialized)
{
        SP_COMPort_t _port={0};

        result=SP_InitPort(&_port,0,0,0,(Timer_TimeUnit_t)0,0);

        EXPECT_EQ(SP_ERROR_NO_DRIVER_INTERFACE,result);
}

TEST_F(SERIALPORT,SP_Init_DriverInterfaceInitialized)
{
        SP_COMPort_t _port={0};
        SetupDriverInterface(&_port);

        result=SP_InitPort(&_port,0,0,0,(Timer_TimeUnit_t)0,0);

        EXPECT_EQ(RESULT_OK,result);
}

TEST_F(SERIALPORT,SP_Init_PortDefaultConfigurationApplied)
{
        SP_InitPort(&port,0,0,0,(Timer_TimeUnit_t)0,0);

        EXPECT_EQ(SP_BR_9600,port.cfg.BaudRate);
        EXPECT_EQ(SP_DB_8,port.cfg.DataBits);
        EXPECT_EQ(SP_PA_NONE,port.cfg.Parity);
        EXPECT_EQ(SP_SB_ONE,port.cfg.StopBits);
        EXPECT_EQ(SP_FC_NONE,port.cfg.FlowControl);
}

TEST_F(SERIALPORT,SP_Init_PortTransferDescriptorsInitialized)
{
        SP_InitPort(
                &port,
                spRxCompletedCbk,
                spTxCompletedCbk,
                &tsys,
                TIMER_TU_MS,
                &ud
        );

        EXPECT_EQ(0,port.rxd.t_on);
        EXPECT_EQ(0,port.rxd.len);
        EXPECT_EQ(0,port.rxd.plen);
        EXPECT_EQ(0,port.rxd.data);
        EXPECT_EQ(0,port.rxd.tleft);
        EXPECT_EQ(0,port.rxd.ptleft);
        EXPECT_EQ(0,port.rxd.tout);
        EXPECT_EQ(0,port.rxd.tmr);
        EXPECT_EQ(&tsys,port.rxd.tsys);
        EXPECT_EQ(TIMER_TU_MS,port.rxd.tu);
        EXPECT_EQ((SP_TransferCompletedCbk_t)spRxCompletedCbk,port.rxd.cbk);
        EXPECT_EQ(&ud,port.rxd.ud);

        EXPECT_EQ(0,port.txd.t_on);
        EXPECT_EQ(0,port.txd.len);
        EXPECT_EQ(0,port.txd.plen);
        EXPECT_EQ(0,port.txd.data);
        EXPECT_EQ(0,port.txd.tleft);
        EXPECT_EQ(0,port.txd.ptleft);
        EXPECT_EQ(0,port.txd.tout);
        EXPECT_EQ(0,port.txd.tmr);
        EXPECT_EQ(&tsys,port.txd.tsys);
        EXPECT_EQ(TIMER_TU_MS,port.txd.tu);
        EXPECT_EQ((SP_TransferCompletedCbk_t)spTxCompletedCbk,port.txd.cbk);
        EXPECT_EQ(&ud,port.txd.ud);
}

TEST_F(SERIALPORT,SP_Init_PortInitialized)
{
        SP_InitPort(
                &port,
                spRxCompletedCbk,
                spTxCompletedCbk,
                &tsys,
                TIMER_TU_MS,
                &ud
        );

        EXPECT_EQ(port.init,bTrue);
}

TEST_F(SERIALPORT,SP_Init_CallDriverFunction_Init)
{
        spInitReturnValue=SP_ERROR_DRIVER_INTERNAL_ERROR;

        result=SP_InitPort(
                &port,
                spRxCompletedCbk,
                spTxCompletedCbk,
                &tsys,
                TIMER_TU_MS,
                &ud
        );

        EXPECT_EQ(SP_ERROR_DRIVER_INTERNAL_ERROR,result);
}

TEST_F(SERIALPORT,SP_GetCurrentConfig_RequiredParameter_port)
{
        result=SP_GetCurrentConfig(0,&config);

        EXPECT_EQ(SP_ERROR_INVALID_POINTER,result);
}

TEST_F(SERIALPORT,SP_GetCurrentConfig_RequiredParameter_config)
{
        result=SP_GetCurrentConfig(&port,0);

        EXPECT_EQ(SP_ERROR_INVALID_POINTER,result);
}

TEST_F(SERIALPORT,SP_GetCurrentConfig_ConfigurationInitialized)
{
        SP_InitPort(
                &port,
                spRxCompletedCbk,
                spTxCompletedCbk,
                &tsys,
                TIMER_TU_MS,
                &ud
        );

        SP_GetCurrentConfig(&port,&config);

        EXPECT_EQ(SP_BR_9600,config.BaudRate);
}

TEST_F(SERIALPORT,SP_Open_RequiredParameter_port)
{
        result=SP_Open(0,&config,&handle);

        EXPECT_EQ(SP_ERROR_INVALID_POINTER,result);
}

TEST_F(SERIALPORT,SP_Open_RequiredParameter_config)
{
        result=SP_Open(&port,0,&handle);

        EXPECT_EQ(SP_ERROR_INVALID_POINTER,result);
}

TEST_F(SERIALPORT,SP_Open_RequiredParameter_handle)
{
        result=SP_Open(&port,&config,0);

        EXPECT_EQ(SP_ERROR_INVALID_POINTER,result);
}

TEST_F(SERIALPORT,SP_Open_HandleAlreadyOccupied)
{
        result=SP_Open(&port,&config,&handle);

        EXPECT_EQ(SP_ERROR_RESOURCE_IN_USE,result);
}

TEST_F(SERIALPORT,SP_Open_PortNotInitialized)
{
        handle=0;

        result=SP_Open(&port,&config,&handle);

        EXPECT_EQ(SP_ERROR_PORT_NOT_INITIALIZED,result);
}

TEST_F(SERIALPORT,SP_Open_ConfigurePort)
{
        handle=0;

        SetupDriverInterface(&port);
        InitPort(&port);
        config.BaudRate=SP_BR_115200;

        SP_Open(&port,&config,&handle);

        EXPECT_EQ(SP_BR_115200,port.cfg.BaudRate);
}

TEST_F(SERIALPORT,SP_Open_CallDriverFunction_Open)
{
        handle=0;
        spOpenReturnValue=SP_ERROR_DRIVER_INTERNAL_ERROR;

        SetupDriverInterface(&port);
        InitPort(&port);

        result=SP_Open(&port,&config,&handle);

        EXPECT_EQ(SP_ERROR_DRIVER_INTERNAL_ERROR,result);
}

TEST_F(SERIALPORT,SP_Open_HandleOccupied)
{
        handle=0;

        SetupDriverInterface(&port);
        InitPort(&port);

        SP_Open(&port,&config,&handle);

        EXPECT_NE((Handle_t)0,handle);
}

TEST_F(SERIALPORT,SP_Close_RequiredParameter_handle)
{
        result=SP_Close(0);

        EXPECT_EQ(SP_ERROR_INVALID_POINTER,result);
}

TEST_F(SERIALPORT,SP_Close_CancelTransfer_AsyncCallback_TransferNotOngoing)
{
        OpenPort(&handle);

        port.rxd.t_on=0;
        port.txd.t_on=0;

        spRxCompletedCbkResult=-100;
        spTxCompletedCbkResult=-100;

        SP_Close(&handle);

        EXPECT_EQ(-100,spRxCompletedCbkResult);
        EXPECT_EQ(-100,spTxCompletedCbkResult);
}

TEST_F(SERIALPORT,SP_Close_CancelTransfer_AsyncCallback_TransferOngoing)
{
        OpenPort(&handle);

        port.rxd.t_on=1;
        port.txd.t_on=1;

        SP_Close(&handle);

        EXPECT_EQ(SP_ERROR_ASYNC_TRANSFER_CANCELLED,spRxCompletedCbkResult);
        EXPECT_EQ((void*)&ud,spRxCompletedCbkUserData);
        EXPECT_EQ(SP_ERROR_ASYNC_TRANSFER_CANCELLED,spTxCompletedCbkResult);
        EXPECT_EQ((void*)&ud,spTxCompletedCbkUserData);
}

TEST_F(SERIALPORT,SP_Close_CancelTransfer_TransferStopped)
{
        OpenPort(&handle);

        port.rxd.t_on=1;
        port.txd.t_on=1;

        SP_Close(&handle);

        EXPECT_EQ(0,port.rxd.t_on);
        EXPECT_EQ(0,port.txd.t_on);
}

TEST_F(SERIALPORT,SP_Close_CallDriverFunction_Close)
{
        OpenPort(&handle);

        spCloseReturnValue=SP_ERROR_DRIVER_INTERNAL_ERROR;

        result=SP_Close(&handle);

        EXPECT_EQ(SP_ERROR_DRIVER_INTERNAL_ERROR,result);
}

TEST_F(SERIALPORT,SP_Close_ResetHandle)
{
        OpenPort(&handle);

        SP_Close(&handle);

        EXPECT_EQ(0,handle);
}

TEST_F(SERIALPORT,SP_ChangeConfig_RequiredParameter_handle)
{
        result=SP_ChangeConfig(0,&config);

        EXPECT_EQ(SP_ERROR_INVALID_PARAMETER,result);
}

TEST_F(SERIALPORT,SP_ChangeConfig_RequiredParameter_config)
{
        result=SP_ChangeConfig(handle,0);

        EXPECT_EQ(SP_ERROR_INVALID_POINTER,result);
}

TEST_F(SERIALPORT,SP_ChangeConfig_CancelAsyncTransfer)
{
        OpenPort(&handle);

        port.rxd.t_on=1;
        port.txd.t_on=1;

        SP_ChangeConfig(handle,&config);

        EXPECT_EQ(0,port.rxd.t_on);
        EXPECT_EQ(0,port.txd.t_on);
}

TEST_F(SERIALPORT,SP_ChangeConfig_CallDriverFunction_Close)
{
        OpenPort(&handle);
        spCloseReturnValue=SP_ERROR_DRIVER_INTERNAL_ERROR;

        result=SP_ChangeConfig(handle,&config);

        EXPECT_EQ(SP_ERROR_DRIVER_INTERNAL_ERROR,result);
}

TEST_F(SERIALPORT,SP_ChangeConfig_ConfigChanged)
{
        OpenPort(&handle);
        config.BaudRate=SP_BR_230400;

        SP_ChangeConfig(handle,&config);

        EXPECT_EQ(SP_BR_230400,port.cfg.BaudRate);
}

TEST_F(SERIALPORT,SP_ChangeConfig_CallDriverFunction_Open)
{
        OpenPort(&handle);
        spOpenReturnValue=SP_ERROR_DRIVER_INTERNAL_ERROR;

        result=SP_ChangeConfig(handle,&config);

        EXPECT_EQ(SP_ERROR_DRIVER_INTERNAL_ERROR,result);
}

TEST_F(SERIALPORT,SP_Read_RequiredParameter_handle)
{
        result=SP_Read(0,0,data,0,0);

        EXPECT_EQ(SP_ERROR_INVALID_PARAMETER,result);
}

TEST_F(SERIALPORT,SP_Read_RequiredParameter_data)
{
        result=SP_Read(handle,0,0,0,0);

        EXPECT_EQ(SP_ERROR_INVALID_POINTER,result);
}

TEST_F(SERIALPORT,SP_Read_TransferAlreadyInProgress)
{
        OpenPort(&handle);
        port.rxd.t_on=1;

        result=SP_Read(handle,0,data,&bytesRead,1000);

        EXPECT_EQ(SP_ERROR_ASYNC_TRANSFER_IN_PROGRESS,result);
}

TEST_F(SERIALPORT,SP_Read_TransferTurnedOn)
{
        OpenPort(&handle);
        port.rxd.t_on=0;

        SP_Read(handle,128,data,&bytesRead,1000);

        EXPECT_EQ(1,port.rxd.t_on);
}

TEST_F(SERIALPORT,SP_Read_TransferInitialized)
{
        OpenPort(&handle);
        port.rxd.ptleft=-1;
        tsys.tck=12345;

        SP_Read(handle,0,data,&bytesRead,1000);

        EXPECT_EQ(0,port.rxd.len);
        EXPECT_EQ((uint8_t*)data,port.rxd.data);
        EXPECT_EQ((uint32_t*)&bytesRead,port.rxd.plen);
        EXPECT_EQ(1000,port.rxd.tout);
        EXPECT_EQ(0,port.rxd.tleft);
        EXPECT_EQ(0,port.rxd.ptleft);
        EXPECT_EQ(0,bytesRead);
        EXPECT_EQ(12345,port.rxd.tmr);
}

TEST_F(SERIALPORT,SP_Read_TransferInitialized_TimeoutNotSpecified)
{
        OpenPort(&handle);
        tsys.tck=12345;

        SP_Read(handle,0,data,&bytesRead,0);

        EXPECT_EQ(0,port.rxd.tmr);
}

TEST_F(SERIALPORT,SP_Read_ZeroLength_TransferCompleted)
{
        OpenPort(&handle);
        spRxCompletedCbkResult=-1;

        result=SP_Read(handle,0,data,&bytesRead,0);

        EXPECT_EQ(RESULT_OK,result);
        EXPECT_EQ(0,port.rxd.t_on);
        EXPECT_EQ(0,bytesRead);
        EXPECT_EQ(RESULT_OK,spRxCompletedCbkResult);
}

TEST_F(SERIALPORT,SP_Read_CallDriverFunction_Read)
{
        OpenPort(&handle);
        spReadReturnValue=SP_ERROR_DRIVER_INTERNAL_ERROR;

        result=SP_Read(handle,128,data,&bytesRead,1000);

        EXPECT_EQ(SP_ERROR_DRIVER_INTERNAL_ERROR,result);
}

TEST_F(SERIALPORT,SP_Read_TransferCancelledOnReadError)
{
        OpenPort(&handle);
        spReadReturnValue=SP_ERROR_DRIVER_INTERNAL_ERROR;

        result=SP_Read(handle,128,data,&bytesRead,1000);

        EXPECT_EQ(0,port.rxd.t_on);
        EXPECT_EQ(SP_ERROR_DRIVER_INTERNAL_ERROR,spRxCompletedCbkResult);
}

TEST_F(SERIALPORT,SP_Read_TransferSuccessfulOnRxBufferEmpty)
{
        OpenPort(&handle);
        spReadReturnValue=SP_ERROR_RX_BUFFER_EMPTY;

        result=SP_Read(handle,128,data,&bytesRead,1000);

        EXPECT_EQ(1,port.rxd.t_on);
        EXPECT_EQ(RESULT_OK,spRxCompletedCbkResult);
}

TEST_F(SERIALPORT,SP_Read_AsyncTransfer_Full8byteTransfer)
{
        OpenPort(&handle);
        port.drvfuncRead=spRead8bytes;

        SP_Read(handle,8,data,&bytesRead,1000);

        EXPECT_EQ(0,port.rxd.t_on);
        EXPECT_EQ(8,bytesRead);
}

TEST_F(SERIALPORT,SP_Read_AsyncTransfer_Initial8bytes)
{
        OpenPort(&handle);
        port.drvfuncRead=spRead8bytes;

        result=SP_Read(handle,128,data,&bytesRead,0);

        EXPECT_EQ(RESULT_OK,result);
        EXPECT_EQ(1,port.rxd.t_on);
}

TEST_F(SERIALPORT,SP_Read_SyncTransfer_WithoutTimeout)
{
        OpenPort(&handle);
        port.rxd.cbk=0;
        port.drvfuncRead=spRead8bytes;

        result=SP_Read(handle,128,data,&bytesRead,0);

        EXPECT_EQ(RESULT_OK,result);
        EXPECT_EQ(128,bytesRead);
}

TEST_F(SERIALPORT,SP_Read_SyncTransfer_WithTimeout_NoTimeoutBeforeAllAvailableDataReceived)
{
        OpenPort(&handle);
        port.rxd.cbk=0;
        port.drvfuncRead=spReadTimeout;

        result=SP_Read(handle,128,data,&bytesRead,10);

        EXPECT_EQ(126,bytesRead);
        EXPECT_EQ(SP_ERROR_TIMEOUT,result);
}

TEST_F(SERIALPORT,SP_Write_RequiredParameter_handle)
{
        result=SP_Write(0,0,data,0,0);

        EXPECT_EQ(SP_ERROR_INVALID_PARAMETER,result);
}

TEST_F(SERIALPORT,SP_Write_RequiredParameter_data)
{
        result=SP_Write(handle,0,0,0,0);

        EXPECT_EQ(SP_ERROR_INVALID_POINTER,result);
}

TEST_F(SERIALPORT,SP_Write_TransferAlreadyInProgress)
{
        OpenPort(&handle);
        port.txd.t_on=1;

        result=SP_Write(handle,0,data,&bytesWritten,1000);

        EXPECT_EQ(SP_ERROR_ASYNC_TRANSFER_IN_PROGRESS,result);
}

TEST_F(SERIALPORT,SP_Write_TransferTurnedOn)
{
        OpenPort(&handle);
        port.txd.t_on=0;

        SP_Write(handle,128,data,&bytesWritten,1000);

        EXPECT_EQ(1,port.txd.t_on);
}

TEST_F(SERIALPORT,SP_Write_TransferInitialized)
{
        OpenPort(&handle);
        port.txd.ptleft=-1;
        tsys.tck=12345;

        SP_Write(handle,0,data,&bytesWritten,1000);

        EXPECT_EQ(0,port.txd.len);
        EXPECT_EQ((uint8_t*)data,port.txd.data);
        EXPECT_EQ((uint32_t*)&bytesWritten,port.txd.plen);
        EXPECT_EQ(1000,port.txd.tout);
        EXPECT_EQ(0,port.txd.tleft);
        EXPECT_EQ(0,port.txd.ptleft);
        EXPECT_EQ(0,bytesWritten);
        EXPECT_EQ(12345,port.txd.tmr);
}

TEST_F(SERIALPORT,SP_Write_TransferInitialized_TimeoutNotSpecified)
{
        OpenPort(&handle);
        tsys.tck=12345;

        SP_Write(handle,0,data,&bytesWritten,0);

        EXPECT_EQ(0,port.txd.tmr);
}

TEST_F(SERIALPORT,SP_Write_ZeroLength_TransferCompleted)
{
        OpenPort(&handle);
        spTxCompletedCbkResult=-1;

        result=SP_Write(handle,0,data,&bytesWritten,0);

        EXPECT_EQ(RESULT_OK,result);
        EXPECT_EQ(0,port.txd.t_on);
        EXPECT_EQ(0,bytesWritten);
        EXPECT_EQ(RESULT_OK,spTxCompletedCbkResult);
}

TEST_F(SERIALPORT,SP_Write_CallDriverFunction_Write)
{
        OpenPort(&handle);
        spWriteReturnValue=SP_ERROR_DRIVER_INTERNAL_ERROR;

        result=SP_Write(handle,128,data,&bytesWritten,1000);

        EXPECT_EQ(SP_ERROR_DRIVER_INTERNAL_ERROR,result);
}

TEST_F(SERIALPORT,SP_Write_TransferCancelledOnWriteError)
{
        OpenPort(&handle);
        spWriteReturnValue=SP_ERROR_DRIVER_INTERNAL_ERROR;

        result=SP_Write(handle,128,data,&bytesWritten,1000);

        EXPECT_EQ(0,port.txd.t_on);
        EXPECT_EQ(SP_ERROR_DRIVER_INTERNAL_ERROR,spTxCompletedCbkResult);
}

TEST_F(SERIALPORT,SP_Write_TransferSuccessfulOnRxBufferEmpty)
{
        OpenPort(&handle);
        spWriteReturnValue=SP_ERROR_RX_BUFFER_EMPTY;

        result=SP_Write(handle,128,data,&bytesWritten,1000);

        EXPECT_EQ(1,port.txd.t_on);
        EXPECT_EQ(RESULT_OK,spTxCompletedCbkResult);
}

TEST_F(SERIALPORT,SP_Write_AsyncTransfer_Full8byteTransfer)
{
        OpenPort(&handle);
        port.drvfuncWrite=spWrite8bytes;

        SP_Write(handle,8,data,&bytesWritten,1000);

        EXPECT_EQ(0,port.rxd.t_on);
        EXPECT_EQ(8,bytesWritten);
}

TEST_F(SERIALPORT,SP_Write_AsyncTransfer_Initial8bytes)
{
        OpenPort(&handle);
        port.drvfuncWrite=spWrite8bytes;

        result=SP_Write(handle,128,data,&bytesWritten,0);

        EXPECT_EQ(RESULT_OK,result);
        EXPECT_EQ(1,port.txd.t_on);
}

TEST_F(SERIALPORT,SP_Write_SyncTransfer_WithoutTimeout)
{
        OpenPort(&handle);
        port.txd.cbk=0;
        port.drvfuncWrite=spWrite8bytes;

        result=SP_Write(handle,128,data,&bytesWritten,0);

        EXPECT_EQ(RESULT_OK,result);
        EXPECT_EQ(128,bytesWritten);
}

TEST_F(SERIALPORT,SP_Write_SyncTransfer_WithTimeout_NoTimeoutBeforeAllAvailableDataReceived)
{
        OpenPort(&handle);
        port.txd.cbk=0;
        port.drvfuncWrite=spWriteTimeout;

        result=SP_Write(handle,128,data,&bytesWritten,10);

        EXPECT_EQ(126,bytesWritten);
        EXPECT_EQ(SP_ERROR_TIMEOUT,result);
}

TEST_F(SERIALPORT,SP_Run_RequiredParameter_handle)
{
        result=SP_Run(0);

        EXPECT_EQ(SP_ERROR_INVALID_PARAMETER,result);
}

TEST_F(SERIALPORT,SP_Run_CallDriverFunction_RunDriver)
{
        OpenPort(&handle);

        spRunDriverReturnValue=SP_ERROR_DRIVER_INTERNAL_ERROR;

        result=SP_Run(handle);

        EXPECT_EQ(SP_ERROR_DRIVER_INTERNAL_ERROR,result);
}

TEST_F(SERIALPORT,SP_Run_DontFallInUndefinedFunction_RunDriver)
{
        OpenPort(&handle);
        port.drvfuncRunDriver=0;

        SP_Run(handle);
}

TEST_F(SERIALPORT,SP_Run_RunAsyncTransfers)
{
        OpenPort(&handle);
        spReadReturnValue=SP_ERROR_RX_BUFFER_EMPTY;
        spWriteReturnValue=SP_ERROR_TX_BUFFER_FULL;

        SP_Read(handle,128,data,&bytesRead,0);
        SP_Write(handle,128,data,&bytesWritten,0);

        spReadReturnValue=SP_ERROR_DRIVER_INTERNAL_ERROR;
        spWriteReturnValue=SP_ERROR_DRIVER_INTERNAL_ERROR;

        SP_Run(handle);

        EXPECT_EQ(SP_ERROR_DRIVER_INTERNAL_ERROR,spRxCompletedCbkResult);
        EXPECT_EQ(SP_ERROR_DRIVER_INTERNAL_ERROR,spTxCompletedCbkResult);
}

TEST_F(SERIALPORT,SP_Run_DontRunAsyncTransfersWhenNotStarted)
{
        OpenPort(&handle);
        spReadReturnValue=SP_ERROR_RX_BUFFER_EMPTY;

        spReadReturnValue=SP_ERROR_DRIVER_INTERNAL_ERROR;

        SP_Run(handle);

        EXPECT_EQ(RESULT_OK,spRxCompletedCbkResult);
}

} // namespace