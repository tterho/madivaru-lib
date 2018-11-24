#include "gtest/gtest.h"
#include "mdv_timer.h"
#include "mdv_serialport.h"

/*

TODO:

- Test GetChar
- Test PutChar
- Implement CancelTransfer

*/

namespace{

/* TEST FIXTURE */

class mdv_serialport : public ::testing::Test
{
        protected:

        void SetUp() override {
                result=MDV_RESULT_OK;
                handle=(MdvHandle_t)-1;
                ud=0xDEADBEEF;

                memset(&port,0xFF,sizeof(MdvSerialPort_t));
                memset(&drv,0,sizeof(MdvSerialPortDriverInterface_t));
                memset(&config,0xFF,sizeof(MdvSerialPortConfig_t));
                port.init=false;
                port.drv=0;
                spInitReturnValue=MDV_RESULT_OK;
                spOpenReturnValue=MDV_RESULT_OK;
                spCloseReturnValue=MDV_RESULT_OK;
                spReadReturnValue=MDV_RESULT_OK;
                spWriteReturnValue=MDV_RESULT_OK;
                spRunDriverReturnValue=MDV_RESULT_OK;
                spRxCompletedCbkResult=MDV_RESULT_OK;
                spRxCompletedCbkUserData=0;
                spTxCompletedCbkResult=MDV_RESULT_OK;
                spTxCompletedCbkUserData=0;
                memset(data,0,128);
                bytesRead=0xFFFFFFFF;
                bytesWritten=0xFFFFFFFF;

                mdv_timer_system_init(&tsys,1000,10000000);
        }

        void TearDown() override {

        }

        void SetupDriverInterface(MdvSerialPortDriverInterface_t *_drv){
                mdv_serialport_setup_driver_interface(
                        _drv,
                        spInit,
                        spOpen,
                        spClose,
                        spRead,
                        spWrite,
                        spRunDriver
                );
        }

        void InitPort(MdvSerialPort_t *_port){
                mdv_serialport_init(
                        _port,
                        &drv,
                        spRxCompletedCbk,
                        spTxCompletedCbk,
                        &tsys,
                        MDV_TIME_UNIT_MS,
                        &ud
                );
        }

        void OpenPort(MdvHandle_t *_handle){
                *_handle=0;
                SetupDriverInterface(&drv);
                InitPort(&port);
                mdv_serialport_get_current_configuration(&port,&config);
                mdv_serialport_open(&port,&config,_handle);
        }

        MdvSerialPort_t port;
        MdvSerialPortDriverInterface_t drv;
        MdvSerialPortConfig_t config;
        MdvResult_t result;
        MdvHandle_t handle;
        uint32_t ud;
        uint8_t data[128];
        uint32_t bytesRead;
        uint32_t bytesWritten;

        static MdvResult_t spInitReturnValue;
        static MdvResult_t spOpenReturnValue;
        static MdvResult_t spCloseReturnValue;
        static MdvResult_t spReadReturnValue;
        static MdvResult_t spWriteReturnValue;
        static MdvResult_t spRunDriverReturnValue;
        static MdvResult_t spRxCompletedCbkResult;
        static void *spRxCompletedCbkUserData;
        static MdvResult_t spTxCompletedCbkResult;
        static void *spTxCompletedCbkUserData;
        static MdvTimerSystem_t tsys;

        // Driver mockup functions.

        static MdvResult_t spInit(
                void
        )
        {
                return spInitReturnValue;
        }

        static MdvResult_t spOpen(
                MdvSerialPortConfig_t *cfg
        )
        {
                return spOpenReturnValue;
        }
 
        static MdvResult_t spClose(
                void
        )
        {
                return spCloseReturnValue;
        }

        static MdvResult_t spRead(
                MdvSerialPortTransfer_t *rxd
        )
        {
                return spReadReturnValue;
        }

        static MdvResult_t spRead8bytes(
                MdvSerialPortTransfer_t *rxd
        )
        {
                memset(rxd->data,0xAA,8);
                rxd->data+=8;

                if(rxd->tleft==8){
                        rxd->tleft=0;
                        return MDV_RESULT_OK;
                }
                rxd->tleft-=8;
                return MDV_SERIALPORT_ERROR_RX_BUFFER_EMPTY;
        }

        static MdvResult_t spReadTimeout(
                MdvSerialPortTransfer_t *rxd
        )
        {
                if(rxd->tleft>2){
                        rxd->tleft--;
                        rxd->data++;
                }
                mdv_timer_system_tick(&tsys,1);
                return MDV_SERIALPORT_ERROR_RX_BUFFER_EMPTY;
        }

        static MdvResult_t spWrite(
                MdvSerialPortTransfer_t *txd
        )
        {
                return spWriteReturnValue;
        }

        static MdvResult_t spWrite8bytes(
                MdvSerialPortTransfer_t *txd
        )
        {
                txd->data+=8;

                if(txd->tleft==8){
                        txd->tleft=0;
                        return MDV_RESULT_OK;
                }
                txd->tleft-=8;
                return MDV_SERIALPORT_ERROR_TX_BUFFER_FULL;
        }

        static MdvResult_t spWriteTimeout(
                MdvSerialPortTransfer_t *txd
        )
        {
                if(txd->tleft>2){
                        txd->tleft--;
                        txd->data++;
                }
                mdv_timer_system_tick(&tsys,1);
                return MDV_SERIALPORT_ERROR_RX_BUFFER_EMPTY;
        }

        static MdvResult_t spRunDriver(
                MdvSerialPortTransfer_t *rxd,
                MdvSerialPortTransfer_t *txd
        )
        {
                return spRunDriverReturnValue;
        }

        static void
                spRxCompletedCbk(
                        MdvResult_t result,
                        void *userData
                )
        {
                spRxCompletedCbkResult=result;
                spRxCompletedCbkUserData=userData;
        }

        static void
                spTxCompletedCbk(
                        MdvResult_t result,
                        void *userData
                )
        {
                spTxCompletedCbkResult=result;
                spTxCompletedCbkUserData=userData;
        }

};

MdvResult_t mdv_serialport::spInitReturnValue;
MdvResult_t mdv_serialport::spOpenReturnValue;
MdvResult_t mdv_serialport::spCloseReturnValue;
MdvResult_t mdv_serialport::spReadReturnValue;
MdvResult_t mdv_serialport::spWriteReturnValue;
MdvResult_t mdv_serialport::spRunDriverReturnValue;
MdvResult_t mdv_serialport::spRxCompletedCbkResult;
void *mdv_serialport::spRxCompletedCbkUserData;
MdvResult_t mdv_serialport::spTxCompletedCbkResult;
void *mdv_serialport::spTxCompletedCbkUserData;
MdvTimerSystem_t mdv_serialport::tsys;

TEST_F(mdv_serialport,_setup_driver_interface__RequiredParameter_driver)
{
        result=mdv_serialport_setup_driver_interface(
                0,
                spInit,
                spOpen,
                spClose,
                spRead,
                spWrite,
                spRunDriver
        );

        EXPECT_EQ(MDV_SERIALPORT_ERROR_INVALID_POINTER,result);
}

TEST_F(mdv_serialport,_setup_driver_interface__RequiredParameter_funcInit)
{
        result=mdv_serialport_setup_driver_interface(
                &drv,
                0,
                spOpen,
                spClose,
                spRead,
                spWrite,
                spRunDriver
        );

        EXPECT_EQ(MDV_SERIALPORT_ERROR_INVALID_PARAMETER,result);
}

TEST_F(mdv_serialport,_setup_driver_interface__RequiredParameter_funcOpen)
{
        result=mdv_serialport_setup_driver_interface(
                &drv,
                spInit,
                0,
                spClose,
                spRead,
                spWrite,
                spRunDriver
        );

        EXPECT_EQ(MDV_SERIALPORT_ERROR_INVALID_PARAMETER,result);
}

TEST_F(mdv_serialport,_setup_driver_interface__RequiredParameter_funcClose)
{
        result=mdv_serialport_setup_driver_interface(
                &drv,
                spInit,
                spOpen,
                0,
                spRead,
                spWrite,
                spRunDriver
        );

        EXPECT_EQ(MDV_SERIALPORT_ERROR_INVALID_PARAMETER,result);
}

TEST_F(mdv_serialport,_setup_driver_interface__RequiredParameter_funcRead)
{
        result=mdv_serialport_setup_driver_interface(
                &drv,
                spInit,
                spOpen,
                spClose,
                0,
                spWrite,
                spRunDriver
        );

        EXPECT_EQ(MDV_SERIALPORT_ERROR_INVALID_PARAMETER,result);
}

TEST_F(mdv_serialport,_setup_driver_interface__RequiredParameter_funcWrite)
{
        result=mdv_serialport_setup_driver_interface(
                &drv,
                spInit,
                spOpen,
                spClose,
                spRead,
                0,
                spRunDriver
        );

        EXPECT_EQ(MDV_SERIALPORT_ERROR_INVALID_PARAMETER,result);
}

TEST_F(mdv_serialport,_setup_driver_interface__OptionalParameter_funcRunDriver)
{
        result=mdv_serialport_setup_driver_interface(
                &drv,
                spInit,
                spOpen,
                spClose,
                spRead,
                spWrite,
                0
        );

        EXPECT_EQ(MDV_RESULT_OK,result);
}

TEST_F(mdv_serialport,_setup_driver_interface__SetupFinished)
{
        mdv_serialport_setup_driver_interface(
                &drv,
                spInit,
                spOpen,
                spClose,
                spRead,
                spWrite,
                spRunDriver
        );

        EXPECT_EQ((MdvSerialPortDriverInterface_Init_t)spInit,drv.funcInit);
        EXPECT_EQ((MdvSerialPortDriverInterface_Open_t)spOpen,drv.funcOpen);
        EXPECT_EQ((MdvSerialPortDriverInterface_Close_t)spClose,drv.funcClose);
        EXPECT_EQ((MdvSerialPortDriverInterface_Transfer_t)spRead,drv.funcRead);
        EXPECT_EQ((MdvSerialPortDriverInterface_Transfer_t)spWrite,drv.funcWrite);
        EXPECT_EQ((MdvSerialPortDriverInterface_Run_t)spRunDriver,drv.funcRun);
        EXPECT_EQ(true,drv.init);
}

TEST_F(mdv_serialport,_init__RequiredParameter_port)
{
        result=mdv_serialport_init(
                0,
                (MdvSerialPortDriverInterface_t*)0x01,
                (MdvSerialPortTransferCompletedCallback_t)0x1,
                (MdvSerialPortTransferCompletedCallback_t)0x1,
                (MdvTimerSystem_t*)0x1,
                (MdvTimeUnit_t)0x1,
                (void*)0x1
        );

        EXPECT_EQ(MDV_SERIALPORT_ERROR_INVALID_POINTER,result);
}

TEST_F(mdv_serialport,_init__RequiredParameter_driver)
{
        result=mdv_serialport_init(
                &port,
                0,
                (MdvSerialPortTransferCompletedCallback_t)0x1,
                (MdvSerialPortTransferCompletedCallback_t)0x1,
                (MdvTimerSystem_t*)0x1,
                (MdvTimeUnit_t)0x1,
                (void*)0x1
        );

        EXPECT_EQ(MDV_SERIALPORT_ERROR_INVALID_POINTER,result);
}

TEST_F(mdv_serialport,_init__DriverNotInitialized)
{
        result=mdv_serialport_init(
                &port,
                &drv,
                (MdvSerialPortTransferCompletedCallback_t)0x1,
                (MdvSerialPortTransferCompletedCallback_t)0x1,
                (MdvTimerSystem_t*)0x1,
                (MdvTimeUnit_t)0x1,
                (void*)0x1
        );

        EXPECT_EQ(MDV_SERIALPORT_ERROR_NO_DRIVER_INTERFACE,result);
}

TEST_F(mdv_serialport,_init__DriverAssociatedWithPort)
{
        SetupDriverInterface(&drv);

        mdv_serialport_init(
                &port,
                &drv,
                (MdvSerialPortTransferCompletedCallback_t)0x1,
                (MdvSerialPortTransferCompletedCallback_t)0x1,
                (MdvTimerSystem_t*)0x1,
                (MdvTimeUnit_t)0x1,
                (void*)0x1
        );

        EXPECT_EQ(port.drv,&drv);
}

TEST_F(mdv_serialport,_init__OptionalParameter_rxCompleted)
{
        SetupDriverInterface(&drv);

        result=mdv_serialport_init(
                &port,
                &drv,
                0,
                (MdvSerialPortTransferCompletedCallback_t)0x1,
                (MdvTimerSystem_t*)0x1,
                (MdvTimeUnit_t)0x1,
                (void*)0x1
        );

        EXPECT_EQ(MDV_RESULT_OK,result);
}

TEST_F(mdv_serialport,_init__OptionalParameter_txCompleted)
{
        SetupDriverInterface(&drv);

        result=mdv_serialport_init(
                &port,
                &drv,
                (MdvSerialPortTransferCompletedCallback_t)0x1,
                0,
                (MdvTimerSystem_t*)0x1,
                (MdvTimeUnit_t)0x1,
                (void*)0x1
        );

        EXPECT_EQ(MDV_RESULT_OK,result);
}

TEST_F(mdv_serialport,_init__OptionalParameter_timerSys)
{
        SetupDriverInterface(&drv);

        result=mdv_serialport_init(
                &port,
                &drv,
                (MdvSerialPortTransferCompletedCallback_t)0x1,
                (MdvSerialPortTransferCompletedCallback_t)0x1,
                0,
                (MdvTimeUnit_t)0x1,
                (void*)0x1
        );

        EXPECT_EQ(MDV_RESULT_OK,result);
}

TEST_F(mdv_serialport,_init__OptionalParameter_timeUnit)
{
        SetupDriverInterface(&drv);

        result=mdv_serialport_init(
                &port,
                &drv,
                (MdvSerialPortTransferCompletedCallback_t)0x1,
                (MdvSerialPortTransferCompletedCallback_t)0x1,
                (MdvTimerSystem_t*)0x1,
                (MdvTimeUnit_t)0,
                (void*)0x1
        );

        EXPECT_EQ(MDV_RESULT_OK,result);
}

TEST_F(mdv_serialport,_init__OptionalParameter_userData)
{
        SetupDriverInterface(&drv);

        result=mdv_serialport_init(
                &port,
                &drv,
                (MdvSerialPortTransferCompletedCallback_t)0x1,
                (MdvSerialPortTransferCompletedCallback_t)0x1,
                (MdvTimerSystem_t*)0x1,
                (MdvTimeUnit_t)0x1,
                0
        );

        EXPECT_EQ(MDV_RESULT_OK,result);
}

TEST_F(mdv_serialport,_init__PortFunctionPointersUntouched)
{
        SetupDriverInterface(&drv);

        mdv_serialport_init(&port,&drv,0,0,0,(MdvTimeUnit_t)0,0);

        EXPECT_EQ((MdvSerialPortDriverInterface_Init_t)spInit,port.drv->funcInit);
        EXPECT_EQ((MdvSerialPortDriverInterface_Open_t)spOpen,port.drv->funcOpen);
        EXPECT_EQ((MdvSerialPortDriverInterface_Close_t)spClose,port.drv->funcClose);
        EXPECT_EQ((MdvSerialPortDriverInterface_Transfer_t)spRead,port.drv->funcRead);
        EXPECT_EQ((MdvSerialPortDriverInterface_Transfer_t)spWrite,port.drv->funcWrite);
        EXPECT_EQ((MdvSerialPortDriverInterface_Run_t)spRunDriver,port.drv->funcRun);
}

TEST_F(mdv_serialport,_init__PortDefaultConfigurationApplied)
{
        SetupDriverInterface(&drv);

        mdv_serialport_init(&port,&drv,0,0,0,(MdvTimeUnit_t)0,0);

        EXPECT_EQ(MDV_SERIALPORT_BAUDRATE_9600,port.cfg.baudRate);
        EXPECT_EQ(MDV_SERIALPORT_DATABITS_8,port.cfg.dataBits);
        EXPECT_EQ(MDV_SERIALPORT_PARITY_NONE,port.cfg.parity);
        EXPECT_EQ(MDV_SERIALPORT_STOPBITS_ONE,port.cfg.stopBits);
        EXPECT_EQ(MDV_SERIALPORT_FLOWCONTROL_NONE,port.cfg.flowControl);
}

TEST_F(mdv_serialport,_init__PortTransferDescriptorsInitialized)
{
        SetupDriverInterface(&drv);

        mdv_serialport_init(
                &port,
                &drv,
                spRxCompletedCbk,
                spTxCompletedCbk,
                &tsys,
                MDV_TIME_UNIT_MS,
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
        EXPECT_EQ(MDV_TIME_UNIT_MS,port.rxd.tu);
        EXPECT_EQ((MdvSerialPortTransferCompletedCallback_t)spRxCompletedCbk,port.rxd.cbk);
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
        EXPECT_EQ(MDV_TIME_UNIT_MS,port.txd.tu);
        EXPECT_EQ((MdvSerialPortTransferCompletedCallback_t)spTxCompletedCbk,port.txd.cbk);
        EXPECT_EQ(&ud,port.txd.ud);
}

TEST_F(mdv_serialport,_init__PortInitialized)
{
        SetupDriverInterface(&drv);

        mdv_serialport_init(
                &port,
                &drv,
                spRxCompletedCbk,
                spTxCompletedCbk,
                &tsys,
                MDV_TIME_UNIT_MS,
                &ud
        );

        EXPECT_EQ(port.init,true);
}

TEST_F(mdv_serialport,_init__CallDriverFunction_Init)
{
        SetupDriverInterface(&drv);

        spInitReturnValue=MDV_SERIALPORT_ERROR_DRIVER_INTERNAL_ERROR;

        result=mdv_serialport_init(
                &port,
                &drv,
                spRxCompletedCbk,
                spTxCompletedCbk,
                &tsys,
                MDV_TIME_UNIT_MS,
                &ud
        );

        EXPECT_EQ(MDV_SERIALPORT_ERROR_DRIVER_INTERNAL_ERROR,result);
}

TEST_F(mdv_serialport,_get_current_configuration__RequiredParameter_port)
{
        result=mdv_serialport_get_current_configuration(0,&config);

        EXPECT_EQ(MDV_SERIALPORT_ERROR_INVALID_POINTER,result);
}

TEST_F(mdv_serialport,_get_current_configuration__RequiredParameter_config)
{
        result=mdv_serialport_get_current_configuration(&port,0);

        EXPECT_EQ(MDV_SERIALPORT_ERROR_INVALID_POINTER,result);
}

TEST_F(mdv_serialport,_get_current_configuration__ConfigurationInitialized)
{
        SetupDriverInterface(&drv);

        mdv_serialport_init(
                &port,
                &drv,
                spRxCompletedCbk,
                spTxCompletedCbk,
                &tsys,
                MDV_TIME_UNIT_MS,
                &ud
        );

        mdv_serialport_get_current_configuration(&port,&config);

        EXPECT_EQ(MDV_SERIALPORT_BAUDRATE_9600,config.baudRate);
}

TEST_F(mdv_serialport,_open__RequiredParameter_port)
{
        result=mdv_serialport_open(0,&config,&handle);

        EXPECT_EQ(MDV_SERIALPORT_ERROR_INVALID_POINTER,result);
}

TEST_F(mdv_serialport,_open__RequiredParameter_config)
{
        result=mdv_serialport_open(&port,0,&handle);

        EXPECT_EQ(MDV_SERIALPORT_ERROR_INVALID_POINTER,result);
}

TEST_F(mdv_serialport,_open__RequiredParameter_handle)
{
        result=mdv_serialport_open(&port,&config,0);

        EXPECT_EQ(MDV_SERIALPORT_ERROR_INVALID_POINTER,result);
}

TEST_F(mdv_serialport,_open__HandleAlreadyOccupied)
{
        result=mdv_serialport_open(&port,&config,&handle);

        EXPECT_EQ(MDV_SERIALPORT_ERROR_RESOURCE_IN_USE,result);
}

TEST_F(mdv_serialport,_open__PortNotInitialized)
{
        handle=0;

        result=mdv_serialport_open(&port,&config,&handle);

        EXPECT_EQ(MDV_SERIALPORT_ERROR_PORT_NOT_INITIALIZED,result);
}

TEST_F(mdv_serialport,_open__ConfigurePort)
{
        handle=0;

        SetupDriverInterface(&drv);
        InitPort(&port);
        config.baudRate=MDV_SERIALPORT_BAUDRATE_115200;

        mdv_serialport_open(&port,&config,&handle);

        EXPECT_EQ(MDV_SERIALPORT_BAUDRATE_115200,port.cfg.baudRate);
}

TEST_F(mdv_serialport,_open__CallDriverFunction_Open)
{
        handle=0;
        spOpenReturnValue=MDV_SERIALPORT_ERROR_DRIVER_INTERNAL_ERROR;

        SetupDriverInterface(&drv);
        InitPort(&port);

        result=mdv_serialport_open(&port,&config,&handle);

        EXPECT_EQ(MDV_SERIALPORT_ERROR_DRIVER_INTERNAL_ERROR,result);
}

TEST_F(mdv_serialport,_open__HandleOccupied)
{
        handle=0;

        SetupDriverInterface(&drv);
        InitPort(&port);

        mdv_serialport_open(&port,&config,&handle);

        EXPECT_NE((MdvHandle_t)0,handle);
}

TEST_F(mdv_serialport,_close__RequiredParameter_handle)
{
        result=mdv_serialport_close(0);

        EXPECT_EQ(MDV_SERIALPORT_ERROR_INVALID_POINTER,result);
}

TEST_F(mdv_serialport,_close__CancelTransfer_AsyncCallback_TransferNotOngoing)
{
        OpenPort(&handle);

        port.rxd.t_on=0;
        port.txd.t_on=0;

        spRxCompletedCbkResult=-100;
        spTxCompletedCbkResult=-100;

        mdv_serialport_close(&handle);

        EXPECT_EQ(-100,spRxCompletedCbkResult);
        EXPECT_EQ(-100,spTxCompletedCbkResult);
}

TEST_F(mdv_serialport,_close__CancelTransfer_AsyncCallback_TransferOngoing)
{
        OpenPort(&handle);

        port.rxd.t_on=1;
        port.txd.t_on=1;

        mdv_serialport_close(&handle);

        EXPECT_EQ(MDV_SERIALPORT_ERROR_ASYNC_TRANSFER_CANCELLED,spRxCompletedCbkResult);
        EXPECT_EQ((void*)&ud,spRxCompletedCbkUserData);
        EXPECT_EQ(MDV_SERIALPORT_ERROR_ASYNC_TRANSFER_CANCELLED,spTxCompletedCbkResult);
        EXPECT_EQ((void*)&ud,spTxCompletedCbkUserData);
}

TEST_F(mdv_serialport,_close__CancelTransfer_TransferStopped)
{
        OpenPort(&handle);

        port.rxd.t_on=1;
        port.txd.t_on=1;

        mdv_serialport_close(&handle);

        EXPECT_EQ(0,port.rxd.t_on);
        EXPECT_EQ(0,port.txd.t_on);
}

TEST_F(mdv_serialport,_close__CallDriverFunction_Close)
{
        OpenPort(&handle);

        spCloseReturnValue=MDV_SERIALPORT_ERROR_DRIVER_INTERNAL_ERROR;

        result=mdv_serialport_close(&handle);

        EXPECT_EQ(MDV_SERIALPORT_ERROR_DRIVER_INTERNAL_ERROR,result);
}

TEST_F(mdv_serialport,_close__ResetHandle)
{
        OpenPort(&handle);

        mdv_serialport_close(&handle);

        EXPECT_EQ(0,handle);
}

TEST_F(mdv_serialport,_change_configuration__RequiredParameter_handle)
{
        result=mdv_serialport_change_configuration(0,&config);

        EXPECT_EQ(MDV_SERIALPORT_ERROR_INVALID_PARAMETER,result);
}

TEST_F(mdv_serialport,_change_configuration__RequiredParameter_config)
{
        result=mdv_serialport_change_configuration(handle,0);

        EXPECT_EQ(MDV_SERIALPORT_ERROR_INVALID_POINTER,result);
}

TEST_F(mdv_serialport,_change_configuration__CancelAsyncTransfer)
{
        OpenPort(&handle);

        port.rxd.t_on=1;
        port.txd.t_on=1;

        mdv_serialport_change_configuration(handle,&config);

        EXPECT_EQ(0,port.rxd.t_on);
        EXPECT_EQ(0,port.txd.t_on);
}

TEST_F(mdv_serialport,_change_configuration__CallDriverFunction_Close)
{
        OpenPort(&handle);
        spCloseReturnValue=MDV_SERIALPORT_ERROR_DRIVER_INTERNAL_ERROR;

        result=mdv_serialport_change_configuration(handle,&config);

        EXPECT_EQ(MDV_SERIALPORT_ERROR_DRIVER_INTERNAL_ERROR,result);
}

TEST_F(mdv_serialport,_change_configuration__ConfigChanged)
{
        OpenPort(&handle);
        config.baudRate=MDV_SERIALPORT_BAUDRATE_230400;

        mdv_serialport_change_configuration(handle,&config);

        EXPECT_EQ(MDV_SERIALPORT_BAUDRATE_230400,port.cfg.baudRate);
}

TEST_F(mdv_serialport,_change_configuration__CallDriverFunction_Open)
{
        OpenPort(&handle);
        spOpenReturnValue=MDV_SERIALPORT_ERROR_DRIVER_INTERNAL_ERROR;

        result=mdv_serialport_change_configuration(handle,&config);

        EXPECT_EQ(MDV_SERIALPORT_ERROR_DRIVER_INTERNAL_ERROR,result);
}

TEST_F(mdv_serialport,_read__RequiredParameter_handle)
{
        result=mdv_serialport_read(0,0,data,0,0);

        EXPECT_EQ(MDV_SERIALPORT_ERROR_INVALID_PARAMETER,result);
}

TEST_F(mdv_serialport,_read__RequiredParameter_data)
{
        result=mdv_serialport_read(handle,0,0,0,0);

        EXPECT_EQ(MDV_SERIALPORT_ERROR_INVALID_POINTER,result);
}

TEST_F(mdv_serialport,_read__TransferAlreadyInProgress)
{
        OpenPort(&handle);
        port.rxd.t_on=1;

        result=mdv_serialport_read(handle,0,data,&bytesRead,1000);

        EXPECT_EQ(MDV_SERIALPORT_ERROR_ASYNC_TRANSFER_IN_PROGRESS,result);
}

TEST_F(mdv_serialport,_read__TransferTurnedOn)
{
        OpenPort(&handle);
        port.rxd.t_on=0;

        mdv_serialport_read(handle,128,data,&bytesRead,1000);

        EXPECT_EQ(1,port.rxd.t_on);
}

TEST_F(mdv_serialport,_read__TransferInitialized)
{
        OpenPort(&handle);
        port.rxd.ptleft=-1;
        tsys.tck=12345;

        mdv_serialport_read(handle,0,data,&bytesRead,1000);

        EXPECT_EQ(0,port.rxd.len);
        EXPECT_EQ((uint8_t*)data,port.rxd.data);
        EXPECT_EQ((uint32_t*)&bytesRead,port.rxd.plen);
        EXPECT_EQ(1000,port.rxd.tout);
        EXPECT_EQ(0,port.rxd.tleft);
        EXPECT_EQ(0,port.rxd.ptleft);
        EXPECT_EQ(0,bytesRead);
        EXPECT_EQ(12345,port.rxd.tmr);
}

TEST_F(mdv_serialport,_read__TransferInitialized_TimeoutNotSpecified)
{
        OpenPort(&handle);
        tsys.tck=12345;

        mdv_serialport_read(handle,0,data,&bytesRead,0);

        EXPECT_EQ(0,port.rxd.tmr);
}

TEST_F(mdv_serialport,_read__ZeroLength_TransferCompleted)
{
        OpenPort(&handle);
        spRxCompletedCbkResult=-1;

        result=mdv_serialport_read(handle,0,data,&bytesRead,0);

        EXPECT_EQ(MDV_RESULT_OK,result);
        EXPECT_EQ(0,port.rxd.t_on);
        EXPECT_EQ(0,bytesRead);
        EXPECT_EQ(MDV_RESULT_OK,spRxCompletedCbkResult);
}

TEST_F(mdv_serialport,_read__CallDriverFunction_Read)
{
        OpenPort(&handle);
        spReadReturnValue=MDV_SERIALPORT_ERROR_DRIVER_INTERNAL_ERROR;

        result=mdv_serialport_read(handle,128,data,&bytesRead,1000);

        EXPECT_EQ(MDV_SERIALPORT_ERROR_DRIVER_INTERNAL_ERROR,result);
}

TEST_F(mdv_serialport,_read__TransferCancelledOnReadError)
{
        OpenPort(&handle);
        spReadReturnValue=MDV_SERIALPORT_ERROR_DRIVER_INTERNAL_ERROR;

        result=mdv_serialport_read(handle,128,data,&bytesRead,1000);

        EXPECT_EQ(0,port.rxd.t_on);
        EXPECT_EQ(MDV_SERIALPORT_ERROR_DRIVER_INTERNAL_ERROR,spRxCompletedCbkResult);
}

TEST_F(mdv_serialport,_read__TransferSuccessfulOnRxBufferEmpty)
{
        OpenPort(&handle);
        spReadReturnValue=MDV_SERIALPORT_ERROR_RX_BUFFER_EMPTY;

        result=mdv_serialport_read(handle,128,data,&bytesRead,1000);

        EXPECT_EQ(1,port.rxd.t_on);
        EXPECT_EQ(MDV_RESULT_OK,spRxCompletedCbkResult);
}

TEST_F(mdv_serialport,_read__AsyncTransfer_Full8byteTransfer)
{
        OpenPort(&handle);
        port.drv->funcRead=spRead8bytes;

        mdv_serialport_read(handle,8,data,&bytesRead,1000);

        EXPECT_EQ(0,port.rxd.t_on);
        EXPECT_EQ(8,bytesRead);
}

TEST_F(mdv_serialport,_read__AsyncTransfer_Initial8bytes)
{
        OpenPort(&handle);
        port.drv->funcRead=spRead8bytes;

        result=mdv_serialport_read(handle,128,data,&bytesRead,0);

        EXPECT_EQ(MDV_RESULT_OK,result);
        EXPECT_EQ(1,port.rxd.t_on);
}

TEST_F(mdv_serialport,_read__SyncTransfer_WithoutTimeout)
{
        OpenPort(&handle);
        port.rxd.cbk=0;
        port.drv->funcRead=spRead8bytes;

        result=mdv_serialport_read(handle,128,data,&bytesRead,0);

        EXPECT_EQ(MDV_RESULT_OK,result);
        EXPECT_EQ(128,bytesRead);
}

TEST_F(mdv_serialport,_read__SyncTransfer_WithTimeout_NoTimeoutBeforeAllAvailableDataReceived)
{
        OpenPort(&handle);
        port.rxd.cbk=0;
        port.drv->funcRead=spReadTimeout;

        result=mdv_serialport_read(handle,128,data,&bytesRead,10);

        EXPECT_EQ(126,bytesRead);
        EXPECT_EQ(MDV_SERIALPORT_ERROR_TIMEOUT,result);
}

TEST_F(mdv_serialport,_write__RequiredParameter_handle)
{
        result=mdv_serialport_write(0,0,data,0,0);

        EXPECT_EQ(MDV_SERIALPORT_ERROR_INVALID_PARAMETER,result);
}

TEST_F(mdv_serialport,_write__RequiredParameter_data)
{
        result=mdv_serialport_write(handle,0,0,0,0);

        EXPECT_EQ(MDV_SERIALPORT_ERROR_INVALID_POINTER,result);
}

TEST_F(mdv_serialport,_write__TransferAlreadyInProgress)
{
        OpenPort(&handle);
        port.txd.t_on=1;

        result=mdv_serialport_write(handle,0,data,&bytesWritten,1000);

        EXPECT_EQ(MDV_SERIALPORT_ERROR_ASYNC_TRANSFER_IN_PROGRESS,result);
}

TEST_F(mdv_serialport,_write__TransferTurnedOn)
{
        OpenPort(&handle);
        port.txd.t_on=0;

        mdv_serialport_write(handle,128,data,&bytesWritten,1000);

        EXPECT_EQ(1,port.txd.t_on);
}

TEST_F(mdv_serialport,_write__TransferInitialized)
{
        OpenPort(&handle);
        port.txd.ptleft=-1;
        tsys.tck=12345;

        mdv_serialport_write(handle,0,data,&bytesWritten,1000);

        EXPECT_EQ(0,port.txd.len);
        EXPECT_EQ((uint8_t*)data,port.txd.data);
        EXPECT_EQ((uint32_t*)&bytesWritten,port.txd.plen);
        EXPECT_EQ(1000,port.txd.tout);
        EXPECT_EQ(0,port.txd.tleft);
        EXPECT_EQ(0,port.txd.ptleft);
        EXPECT_EQ(0,bytesWritten);
        EXPECT_EQ(12345,port.txd.tmr);
}

TEST_F(mdv_serialport,_write__TransferInitialized_TimeoutNotSpecified)
{
        OpenPort(&handle);
        tsys.tck=12345;

        mdv_serialport_write(handle,0,data,&bytesWritten,0);

        EXPECT_EQ(0,port.txd.tmr);
}

TEST_F(mdv_serialport,_write__ZeroLength_TransferCompleted)
{
        OpenPort(&handle);
        spTxCompletedCbkResult=-1;

        result=mdv_serialport_write(handle,0,data,&bytesWritten,0);

        EXPECT_EQ(MDV_RESULT_OK,result);
        EXPECT_EQ(0,port.txd.t_on);
        EXPECT_EQ(0,bytesWritten);
        EXPECT_EQ(MDV_RESULT_OK,spTxCompletedCbkResult);
}

TEST_F(mdv_serialport,_write__CallDriverFunction_Write)
{
        OpenPort(&handle);
        spWriteReturnValue=MDV_SERIALPORT_ERROR_DRIVER_INTERNAL_ERROR;

        result=mdv_serialport_write(handle,128,data,&bytesWritten,1000);

        EXPECT_EQ(MDV_SERIALPORT_ERROR_DRIVER_INTERNAL_ERROR,result);
}

TEST_F(mdv_serialport,_write__TransferCancelledOnWriteError)
{
        OpenPort(&handle);
        spWriteReturnValue=MDV_SERIALPORT_ERROR_DRIVER_INTERNAL_ERROR;

        result=mdv_serialport_write(handle,128,data,&bytesWritten,1000);

        EXPECT_EQ(0,port.txd.t_on);
        EXPECT_EQ(MDV_SERIALPORT_ERROR_DRIVER_INTERNAL_ERROR,spTxCompletedCbkResult);
}

TEST_F(mdv_serialport,_write__TransferSuccessfulOnRxBufferEmpty)
{
        OpenPort(&handle);
        spWriteReturnValue=MDV_SERIALPORT_ERROR_RX_BUFFER_EMPTY;

        result=mdv_serialport_write(handle,128,data,&bytesWritten,1000);

        EXPECT_EQ(1,port.txd.t_on);
        EXPECT_EQ(MDV_RESULT_OK,spTxCompletedCbkResult);
}

TEST_F(mdv_serialport,_write__AsyncTransfer_Full8byteTransfer)
{
        OpenPort(&handle);
        port.drv->funcWrite=spWrite8bytes;

        mdv_serialport_write(handle,8,data,&bytesWritten,1000);

        EXPECT_EQ(0,port.rxd.t_on);
        EXPECT_EQ(8,bytesWritten);
}

TEST_F(mdv_serialport,_write__AsyncTransfer_Initial8bytes)
{
        OpenPort(&handle);
        port.drv->funcWrite=spWrite8bytes;

        result=mdv_serialport_write(handle,128,data,&bytesWritten,0);

        EXPECT_EQ(MDV_RESULT_OK,result);
        EXPECT_EQ(1,port.txd.t_on);
}

TEST_F(mdv_serialport,_write__SyncTransfer_WithoutTimeout)
{
        OpenPort(&handle);
        port.txd.cbk=0;
        port.drv->funcWrite=spWrite8bytes;

        result=mdv_serialport_write(handle,128,data,&bytesWritten,0);

        EXPECT_EQ(MDV_RESULT_OK,result);
        EXPECT_EQ(128,bytesWritten);
}

TEST_F(mdv_serialport,_write__SyncTransfer_WithTimeout_NoTimeoutBeforeAllAvailableDataReceived)
{
        OpenPort(&handle);
        port.txd.cbk=0;
        port.drv->funcWrite=spWriteTimeout;

        result=mdv_serialport_write(handle,128,data,&bytesWritten,10);

        EXPECT_EQ(126,bytesWritten);
        EXPECT_EQ(MDV_SERIALPORT_ERROR_TIMEOUT,result);
}

TEST_F(mdv_serialport,_runtime_process__RequiredParameter_handle)
{
        result=mdv_serialport_runtime_process(0);

        EXPECT_EQ(MDV_SERIALPORT_ERROR_INVALID_PARAMETER,result);
}

TEST_F(mdv_serialport,_runtime_process__CallDriverFunction_RunDriver)
{
        OpenPort(&handle);

        spRunDriverReturnValue=MDV_SERIALPORT_ERROR_DRIVER_INTERNAL_ERROR;

        result=mdv_serialport_runtime_process(handle);

        EXPECT_EQ(MDV_SERIALPORT_ERROR_DRIVER_INTERNAL_ERROR,result);
}

TEST_F(mdv_serialport,_runtime_process__DontFallInUndefinedFunction_RunDriver)
{
        OpenPort(&handle);
        port.drv->funcRun=0;

        mdv_serialport_runtime_process(handle);
}

TEST_F(mdv_serialport,_runtime_process__RunAsyncTransfers)
{
        OpenPort(&handle);
        spReadReturnValue=MDV_SERIALPORT_ERROR_RX_BUFFER_EMPTY;
        spWriteReturnValue=MDV_SERIALPORT_ERROR_TX_BUFFER_FULL;

        mdv_serialport_read(handle,128,data,&bytesRead,0);
        mdv_serialport_write(handle,128,data,&bytesWritten,0);

        spReadReturnValue=MDV_SERIALPORT_ERROR_DRIVER_INTERNAL_ERROR;
        spWriteReturnValue=MDV_SERIALPORT_ERROR_DRIVER_INTERNAL_ERROR;

        mdv_serialport_runtime_process(handle);

        EXPECT_EQ(MDV_SERIALPORT_ERROR_DRIVER_INTERNAL_ERROR,spRxCompletedCbkResult);
        EXPECT_EQ(MDV_SERIALPORT_ERROR_DRIVER_INTERNAL_ERROR,spTxCompletedCbkResult);
}

TEST_F(mdv_serialport,_runtime_process__DontRunAsyncTransfersWhenNotStarted)
{
        OpenPort(&handle);
        spReadReturnValue=MDV_SERIALPORT_ERROR_RX_BUFFER_EMPTY;

        spReadReturnValue=MDV_SERIALPORT_ERROR_DRIVER_INTERNAL_ERROR;

        mdv_serialport_runtime_process(handle);

        EXPECT_EQ(MDV_RESULT_OK,spRxCompletedCbkResult);
}

} // namespace