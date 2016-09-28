/*******************************************************************************
  Filename:       simpleBLEPeripheral.c
  Revised:        $Date: 2016-01-07 16:59:59 -0800 (Thu, 07 Jan 2016) $
  Revision:       $Revision: 44594 $

  Description:    This file contains the Simple BLE Peripheral sample 
                  application for use with the CC2650 Bluetooth Low Energy 
                  Protocol Stack.

  Copyright 2013 - 2015 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
*******************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <string.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>

#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/family/arm/m3/Hwi.h>
#include <ti/sysbios/family/arm/cc26xx/Power.h>
#include <ti/sysbios/BIOS.h>
#include <ti/drivers/PIN/PINCC26XX.h>
#include <ti/drivers/UART.h>

#include <driverlib/aux_adc.h>
#include <driverlib/aux_wuc.h>
#include <inc/hw_aux_evctl.h>

#include "hci_tl.h"
#include "gatt.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"
#include "simpleGATTprofile.h"

#if defined(SENSORTAG_HW)
#include "bsp_spi.h"
#endif // SENSORTAG_HW

#if defined(FEATURE_OAD) || defined(IMAGE_INVALIDATE)
#include "oad_target.h"
#include "oad.h"
#endif //FEATURE_OAD || IMAGE_INVALIDATE

#include "peripheral.h"
#include "gapbondmgr.h"

#include "osal_snv.h"
#include "ICallBleAPIMSG.h"

#include "util.h"
#include "board_lcd.h"
#include "board_key.h"
#include "Board.h"

#include "simpleBLEPeripheral.h"

#include <ti/drivers/lcd/LCDDogm1286.h>

#if defined (NPI_USE_UART) || defined (NPI_USE_SPI) 
#include "tl.h"
#endif //TL

/*********************************************************************
 * CONSTANTS
 */
// Advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          160

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

#ifndef FEATURE_OAD
// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     8

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     8
#else
// Minimum connection interval (units of 1.25ms, 8=10ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     8

// Maximum connection interval (units of 1.25ms, 8=10ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     8
#endif // FEATURE_OAD

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter
// update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          1000

// Whether to enable automatic parameter update request when a connection is
// formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         TRUE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         6

// How often to perform periodic event (in msec)
#define SBP_PERIODIC_EVT_PERIOD               1000
#define SBP_PERIODIC_EVT_PERIOD_BAT           30000



#ifdef FEATURE_OAD
// The size of an OAD packet.
#define OAD_PACKET_SIZE                       ((OAD_BLOCK_SIZE) + 2)
#endif // FEATURE_OAD

// Task configuration
#define SBP_TASK_PRIORITY                     1

#ifndef SBP_TASK_STACK_SIZE
#define SBP_TASK_STACK_SIZE                   644
#endif

// Internal Events for RTOS application
#define SBP_STATE_CHANGE_EVT                  0x0001
#define SBP_CHAR_CHANGE_EVT                   0x0002
#define SBP_PERIODIC_EVT                      0x0004
#define SBP_CONN_EVT_END_EVT                  0x0008

#define SBP_PERIODIC_EVT_BAT                  0x0010


#define LIPOBATTERY_649kBy560k
#ifdef LIPOBATTERY_649kBy560k
#define ADC_BAT_LEVELFULL     0x0A46 /*4.2v @ Ref 3.023v*/
#define ADC_BAT_LEVELMEDIUM   0x0950 /*3.8v @ Ref 3.023v*/
#define ADC_BAT_LEVELCRITICAL 0x08D2 /*3.6v @ Ref 3.023v*/
#define ADC_BAT_LEVELSHUTDOWN 0x0893 /*3.5v @ Ref 3.023v*/
#endif

typedef enum{
  BAT_INDICATEFULL = 0,
  BAT_INDICATEMEDIUM,
  BAT_INDICATECRITICAL,
  BAT_INDICATESHUTDOWN, //3
  BAT_TOTALBATLEVEL /* 4 */
} BATTERY_LEVEL;


#if defined (NPI_USE_UART) || defined (NPI_USE_SPI) 
//events that TL will use to control the driver
#define MRDY_EVENT                            0x0010
#define TRANSPORT_RX_EVENT                    0x0020
#define TRANSPORT_TX_DONE_EVENT               0x0040
#endif //TL

#if defined (NPI_USE_UART) || defined (NPI_USE_SPI)
#define APP_TL_BUFF_SIZE                      150
#endif //TL

/*********************************************************************
 * TYPEDEFS
 */

// App event passed from profiles.
typedef struct
{
  appEvtHdr_t hdr;  // event header.
} sbpEvt_t;

/*********************************************************************
 * LOCAL VARIABLES
 */
#if defined (NPI_USE_UART) || defined (NPI_USE_SPI) 
//used to store data read from transport layer
static uint8_t appRxBuf[APP_TL_BUFF_SIZE];
#endif //TL

static uint8 txbuf[] = {0xFF, 0xFE, 0x01, 0x00, 0x00};
static uint8_t formatCounter=0;
static uint8_t ppgTxBuf[20];
static uint8_t spo2TxBuf[10];
static uint16_t SN=0;
static uint8_t TurnOnPpg=0;
static uint8_t TurnOffPpg=0;
static uint8_t TurnOnDebugSPO2=0;
static uint8_t TurnOffDebugSPO2=0;
static uint8_t ppgReq=5;
static uint8_t debugSPO2Req=5;
uint8_t snvBuf[12];
static uint8_t snvFlag=0;
uint8_t ret=0;
static uint8 ppgFormat = 0;
static uint16_t deviceID;

//adc //HJ ******
Semaphore_Struct semAdc;
Semaphore_Handle hSemAdc;

Hwi_Struct hwi;

#define SAMPLECOUNT 20
#define SAMPLETYPE uint16_t
#define SAMPLESIZE sizeof(SAMPLETYPE)

SAMPLETYPE adcSamples[SAMPLECOUNT];
SAMPLETYPE singleSample;

PIN_Handle hSbpPins_ADC;
PIN_State  sbpPins_ADC;

const PIN_Config SBP_ADC_configTablePins[] = {
  SENS_BAT      | PIN_INPUT_DIS | PIN_GPIO_OUTPUT_DIS ,
  PIN_TERMINATE
};
//adc //HJ ******

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Semaphore globally used to post events to the application thread
static ICall_Semaphore sem;

// Clock instances for internal periodic events.
static Clock_Struct periodicClock;
static Clock_Struct periodicClockBat;

// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

#if defined(FEATURE_OAD)
// Event data from OAD profile.
static Queue_Struct oadQ;
static Queue_Handle hOadQ;
#endif //FEATURE_OAD

// events flag for internal application events.
static uint16_t events;

// Task configuration
Task_Struct sbpTask;
Char sbpTaskStack[SBP_TASK_STACK_SIZE];

// Profile state and parameters
//static gaprole_States_t gapProfileState = GAPROLE_INIT;

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8_t scanRspData[] =
{
  // complete name
  0x0A,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  0x50,   // 'P'
  0x50,   // 'P'
  0x47,   // 'G'
  0x5F,   // '_'
  0x30,   // '0'
  0x30,   // '0'
  0x30,   // '0'
  0x30,   // '0'
  0x30,   // '0'
  // connection interval range
  0x05,   // length of this data
  GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
  LO_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),   // 100ms
  HI_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),
  LO_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),   // 1s
  HI_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),

  // Tx power level
  0x02,   // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0       // 0dBm
};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8_t advertData[] =
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // service UUID, to notify central devices what services are included
  // in this peripheral
  0x03,   // length of this data
  GAP_ADTYPE_16BIT_MORE,      // some of the UUID's, but not all
#ifdef FEATURE_OAD
  LO_UINT16(OAD_SERVICE_UUID),
  HI_UINT16(OAD_SERVICE_UUID)
#else
  LO_UINT16(SIMPLEPROFILE_SERV_UUID),
  HI_UINT16(SIMPLEPROFILE_SERV_UUID),
#endif //!FEATURE_OAD
};

// GAP GATT Attributes
static uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "Simple BLE Peripheral";

// Globals used for ATT Response retransmission
static gattMsgEvent_t *pAttRsp = NULL;
static uint8_t rspTxRetry = 0;







/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void SimpleBLEPeripheral_init( void );
static void SimpleBLEPeripheral_taskFxn(UArg a0, UArg a1);

static uint8_t SimpleBLEPeripheral_processStackMsg(ICall_Hdr *pMsg);
static uint8_t SimpleBLEPeripheral_processGATTMsg(gattMsgEvent_t *pMsg);
static void SimpleBLEPeripheral_processAppMsg(sbpEvt_t *pMsg);
static void SimpleBLEPeripheral_processStateChangeEvt(gaprole_States_t newState);
static void SimpleBLEPeripheral_processCharValueChangeEvt(uint8_t paramID);
static void SimpleBLEPeripheral_performPeriodicTask(void);

static void SimpleBLEPeripheral_sendAttRsp(void);
static void SimpleBLEPeripheral_freeAttRsp(uint8_t status);

static void SimpleBLEPeripheral_stateChangeCB(gaprole_States_t newState);
#ifndef FEATURE_OAD
static void SimpleBLEPeripheral_charValueChangeCB(uint8_t paramID);
#endif //!FEATURE_OAD
static void SimpleBLEPeripheral_enqueueMsg(uint8_t event, uint8_t state);

#ifdef FEATURE_OAD
void SimpleBLEPeripheral_processOadWriteCB(uint8_t event, uint16_t connHandle,
                                           uint8_t *pData);
#endif //FEATURE_OAD

static void SimpleBLEPeripheral_clockHandler(UArg arg);
static void SimpleBLEPeripheral_clockHandler_BAT(UArg arg);

#if defined (NPI_USE_UART) || defined (NPI_USE_SPI) 
//TL packet parser
static void SimpleBLEPeripheral_TLpacketParser(void);
#endif //TL 

static void ppgConnect(void);
static void ppgDisconnect(void);
static void setDeviceId(void);

static void actDebugSPO2(void);
static void deactDebugSPO2(void);

//ADC Start //HJ*********************
void taskFxn(UArg a0, UArg a1);
void adcIsr(UArg a0);


uint16_t ADC_getBatteryLevel(uint16 u16_adcVal) {
  uint16_t u16_ret;

//#define DEBUG_PRINTRAW
#ifdef DEBUG_PRINTRAW
  u16_ret = u16_adcVal;
#else
  if (u16_adcVal > ADC_BAT_LEVELMEDIUM) {
    u16_ret = BAT_INDICATEFULL;
  } else if (u16_adcVal > ADC_BAT_LEVELCRITICAL) {
    u16_ret = BAT_INDICATEMEDIUM;
  } else if (u16_adcVal > ADC_BAT_LEVELSHUTDOWN) {
    u16_ret = BAT_INDICATECRITICAL;
  } else { /* SHUTDOWN */
    u16_ret = BAT_INDICATESHUTDOWN;
  }
#endif

  return u16_ret;
}

void initSemaphoreADC(void) {
  // Construct semaphore used for pending in task
  Semaphore_Params sParams;
  Semaphore_Params_init(&sParams);
  sParams.mode = Semaphore_Mode_BINARY;

  Semaphore_construct(&semAdc, 0, &sParams);
  hSemAdc = Semaphore_handle(&semAdc);
}

void initHwiADC(void) {
  Hwi_Params hwiParams;
  Hwi_Params_init(&hwiParams);
  hwiParams.enableInt = true;

  Hwi_construct(&hwi, INT_AUX_ADC, adcIsr, &hwiParams, NULL);
}

void initAdcTemp(void) {
  // Set up pins
  hSbpPins_ADC = PIN_open(&sbpPins_ADC, SBP_ADC_configTablePins);

  // Enable clock for ADC digital and analog interface (not currently enabled in driver)
  AUXWUCClockEnable(AUX_WUC_SOC_CLOCK/*AUX_WUC_MODCLKEN0_SOC_M*/|AUX_WUC_MODCLKEN0_AUX_ADI4_M);
  // Connect AUX IO5 (DI10) as analog input. Light sensor on SmartRF06EB
  AUXADCSelectInput(ADC_COMPB_IN_AUXIO5);

  // Set up ADC
  AUXADCEnableSync(AUXADC_REF_VDDA_REL, AUXADC_SAMPLE_TIME_2P7_US, AUXADC_TRIGGER_MANUAL);
  
  //disable input scaling for external reference //HJ
  //AUXADCDisableInputScaling();
}

void initAdcBat(void) {
  // Set up pins
  hSbpPins_ADC = PIN_open(&sbpPins_ADC, SBP_ADC_configTablePins);

  // Enable clock for ADC digital and analog interface (not currently enabled in driver)
  AUXWUCClockEnable(AUX_WUC_SOC_CLOCK/*AUX_WUC_MODCLKEN0_SOC_M*/|AUX_WUC_MODCLKEN0_AUX_ADI4_M);
  // Connect AUX IO5 (DI10) as analog input. Light sensor on SmartRF06EB
  AUXADCSelectInput(ADC_COMPB_IN_AUXIO5);

  // Set up ADC
  AUXADCEnableSync(AUXADC_REF_VDDA_REL, AUXADC_SAMPLE_TIME_2P7_US, AUXADC_TRIGGER_MANUAL);
  
  //disable input scaling for external reference //HJ
  //AUXADCDisableInputScaling();
}


void triggerAdcTemp(void){
  // Disallow STANDBY mode while using the ADC.
  Power_setConstraint(Power_SB_DISALLOW);
  //Sleep 100ms in IDLE mode
  //Task_sleep(100 * 1000 / Clock_tickPeriod);

  // Trigger ADC sampling
  AUXADCGenManualTrigger();
  // Wait in IDLE until done
//  Semaphore_pend(hSemAdc, BIOS_WAIT_FOREVER );
}

uint16_t blockingADCInterrupt(void){
  // Disallow STANDBY mode while using the ADC.
  Power_setConstraint(Power_SB_DISALLOW);
  //Sleep 100ms in IDLE mode
  //Task_sleep(100 * 1000 / Clock_tickPeriod);

  // Trigger ADC sampling
  AUXADCGenManualTrigger();
  // Wait in IDLE until done
//  Semaphore_pend(hSemAdc, BIOS_WAIT_FOREVER );
  // Allow STANDBY mode again
  Power_releaseConstraint(Power_SB_DISALLOW);
  return singleSample;
}

void adcIsr(UArg a0) {

  // Pop sample from FIFO to allow clearing ADC_IRQ event
  singleSample = AUXADCReadFifo();
  // Clear ADC_IRQ flag. Note: Missing driver for this.
  HWREGBITW(AUX_EVCTL_BASE + AUX_EVCTL_O_EVTOMCUFLAGSCLR, AUX_EVCTL_EVTOMCUFLAGSCLR_ADC_IRQ_BITN) = 1;

  // Post semaphore to wakeup task
  Semaphore_post(hSemAdc);

}

//ADC End//HJ*********************

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t SimpleBLEPeripheral_gapRoleCBs =
{
  SimpleBLEPeripheral_stateChangeCB     // Profile State Change Callbacks
};

// GAP Bond Manager Callbacks
static gapBondCBs_t simpleBLEPeripheral_BondMgrCBs =
{
  NULL, // Passcode callback (not used by application)
  NULL  // Pairing / Bonding state Callback (not used by application)
};

// Simple GATT Profile Callbacks
#ifndef FEATURE_OAD
static simpleProfileCBs_t SimpleBLEPeripheral_simpleProfileCBs =
{
  SimpleBLEPeripheral_charValueChangeCB // Characteristic value change callback
};
#endif //!FEATURE_OAD

#ifdef FEATURE_OAD
static oadTargetCBs_t simpleBLEPeripheral_oadCBs =
{
  SimpleBLEPeripheral_processOadWriteCB // Write Callback.
};
#endif //FEATURE_OAD



#if defined (NPI_USE_UART) || defined (NPI_USE_SPI) 
static TLCBs_t SimpleBLEPeripheral_TLCBs =
{
  SimpleBLEPeripheral_TLpacketParser // parse data read from transport layer
};
#endif

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SimpleBLEPeripheral_createTask
 *
 * @brief   Task creation function for the Simple BLE Peripheral.
 *
 * @param   None.
 *
 * @return  None.
 */
void SimpleBLEPeripheral_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = sbpTaskStack;
  taskParams.stackSize = SBP_TASK_STACK_SIZE;
  taskParams.priority = SBP_TASK_PRIORITY;

  Task_construct(&sbpTask, SimpleBLEPeripheral_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_init
 *
 * @brief   Called during initialization and contains application
 *          specific initialization (ie. hardware initialization/setup,
 *          table initialization, power up notification, etc), and
 *          profile initialization/setup.
 *
 * @param   None.
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_init(void)
{
  // ******************************************************************
  // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &sem);

  // Hard code the BD Address till CC2650 board gets its own IEEE address
  //uint8 bdAddress[B_ADDR_LEN] = { 0xAD, 0xD0, 0x0A, 0xAD, 0xD0, 0x0A };
  //HCI_EXT_SetBDADDRCmd(bdAddress);

  // Set device's Sleep Clock Accuracy
  //HCI_EXT_SetSCACmd(500);
  
  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueue = Util_constructQueue(&appMsg);

  // Create one-shot clocks for internal periodic events.
  Util_constructClock(&periodicClock, SimpleBLEPeripheral_clockHandler,
                      SBP_PERIODIC_EVT_PERIOD, 0, false, SBP_PERIODIC_EVT);
  Util_constructClock(&periodicClockBat, SimpleBLEPeripheral_clockHandler_BAT,
                      SBP_PERIODIC_EVT_PERIOD_BAT, 0, false, SBP_PERIODIC_EVT_BAT);
  
  
  
#ifndef SENSORTAG_HW
  Board_openLCD();
#endif //SENSORTAG_HW
  
#if SENSORTAG_HW
  // Setup SPI bus for serial flash and Devpack interface
  bspSpiOpen();
#endif //SENSORTAG_HW
  
  // Setup the GAP
  GAP_SetParamValue(TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL);

  // Setup the GAP Peripheral Role Profile
  {
    // For all hardware platforms, device starts advertising upon initialization
    uint8_t initialAdvertEnable = TRUE;

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16_t advertOffTime = 0;

    uint8_t enableUpdateRequest = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16_t desiredMinInterval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16_t desiredMaxInterval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16_t desiredSlaveLatency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16_t desiredConnTimeout = DEFAULT_DESIRED_CONN_TIMEOUT;

    // Set the GAP Role Parameters
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                         &initialAdvertEnable);
    GAPRole_SetParameter(GAPROLE_ADVERT_OFF_TIME, sizeof(uint16_t),
                         &advertOffTime);
    
    
    //HJ
    osal_snv_read(0x80, SIMPLEPROFILE_CHAR9_LEN, snvBuf);
    setDeviceId();
    //GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData), scanRspData);
    GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData);

    GAPRole_SetParameter(GAPROLE_PARAM_UPDATE_ENABLE, sizeof(uint8_t),
                         &enableUpdateRequest);
    GAPRole_SetParameter(GAPROLE_MIN_CONN_INTERVAL, sizeof(uint16_t),
                         &desiredMinInterval);
    GAPRole_SetParameter(GAPROLE_MAX_CONN_INTERVAL, sizeof(uint16_t),
                         &desiredMaxInterval);
    GAPRole_SetParameter(GAPROLE_SLAVE_LATENCY, sizeof(uint16_t),
                         &desiredSlaveLatency);
    GAPRole_SetParameter(GAPROLE_TIMEOUT_MULTIPLIER, sizeof(uint16_t),
                         &desiredConnTimeout);
  }

  // Set the GAP Characteristics
  GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);

  // Set advertising interval
  {
    uint16_t advInt = DEFAULT_ADVERTISING_INTERVAL;

    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, advInt);
  }

  // Setup the GAP Bond Manager
  {
    uint32_t passkey = 0; // passkey "000000"
    uint8_t pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    uint8_t mitm = TRUE;
    uint8_t ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
    uint8_t bonding = TRUE;

    GAPBondMgr_SetParameter(GAPBOND_DEFAULT_PASSCODE, sizeof(uint32_t),
                            &passkey);
    GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t), &pairMode);
    GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);
    GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
    GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t), &bonding);
  }

   // Initialize GATT attributes
  GGS_AddService(GATT_ALL_SERVICES);           // GAP
  GATTServApp_AddService(GATT_ALL_SERVICES);   // GATT attributes
  DevInfo_AddService();                        // Device Information Service

#ifndef FEATURE_OAD
  SimpleProfile_AddService(GATT_ALL_SERVICES); // Simple GATT Profile
#endif //!FEATURE_OAD

#ifdef FEATURE_OAD
  VOID OAD_addService();                 // OAD Profile
  OAD_register((oadTargetCBs_t *)&simpleBLEPeripheral_oadCBs);
  hOadQ = Util_constructQueue(&oadQ);
#endif

#ifdef IMAGE_INVALIDATE
  Reset_addService();
#endif //IMAGE_INVALIDATE
  
  
#ifndef FEATURE_OAD
  // Setup the SimpleProfile Characteristic Values
  {
    uint8_t charValue1 = 1;
    uint8_t charValue2 = 2;
    uint8_t charValue3 = 3;
    uint8_t charValue4[SIMPLEPROFILE_CHAR4_LEN] = { 0 };
    uint8_t charValue5[SIMPLEPROFILE_CHAR5_LEN] = { 0 };
    uint8_t charValue6[SIMPLEPROFILE_CHAR6_LEN] = { 0 };
    uint8_t charValue7[SIMPLEPROFILE_CHAR7_LEN] = { 0 };
    uint8_t charValue8[SIMPLEPROFILE_CHAR8_LEN] = { 0 };
    uint8_t charValue9[SIMPLEPROFILE_CHAR9_LEN] = { 0 };

    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR1, sizeof(uint8_t),
                               &charValue1);
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR2, sizeof(uint8_t),
                               &charValue2);
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR3, sizeof(uint8_t),
                               &charValue3);
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4, SIMPLEPROFILE_CHAR4_LEN,
                               charValue4);
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR5, SIMPLEPROFILE_CHAR5_LEN,
                               charValue5);
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR6, SIMPLEPROFILE_CHAR6_LEN,
                               charValue6);
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR7, SIMPLEPROFILE_CHAR7_LEN,
                               charValue7);
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR8, SIMPLEPROFILE_CHAR8_LEN,
                               charValue8);
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR9, SIMPLEPROFILE_CHAR9_LEN,
                               charValue9);
  }

  // Register callback with SimpleGATTprofile
  SimpleProfile_RegisterAppCBs(&SimpleBLEPeripheral_simpleProfileCBs);
#endif //!FEATURE_OAD
  
  // Start the Device
  VOID GAPRole_StartDevice(&SimpleBLEPeripheral_gapRoleCBs);
  
  // Start Bond Manager
  VOID GAPBondMgr_Register(&simpleBLEPeripheral_BondMgrCBs);

  // Register with GAP for HCI/Host messages
  GAP_RegisterForMsgs(selfEntity);
  
  // Register for GATT local events and ATT Responses pending for transmission
  GATT_RegisterForMsgs(selfEntity);
  
#if defined FEATURE_OAD
#if defined (HAL_IMAGE_A)
  LCD_WRITE_STRING("BLE Peripheral A", LCD_PAGE0);
#else
  LCD_WRITE_STRING("BLE Peripheral B", LCD_PAGE0);
#endif // HAL_IMAGE_A
#else
  LCD_WRITE_STRING("BLE Peripheral", LCD_PAGE0);
#endif // FEATURE_OAD
  
  
#if defined (NPI_USE_UART) || defined (NPI_USE_SPI)
  //initialize and pass information to TL
  TLinit(&sem, &SimpleBLEPeripheral_TLCBs, TRANSPORT_TX_DONE_EVENT,
         TRANSPORT_RX_EVENT, MRDY_EVENT);
#endif //TL
}       

/*********************************************************************
 * @fn      SimpleBLEPeripheral_taskFxn
 *
 * @brief   Application task entry point for the Simple BLE Peripheral.
 *
 * @param   a0, a1 - not used.
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_taskFxn(UArg a0, UArg a1)
{
  // Initialize application
  SimpleBLEPeripheral_init();

  // Application main loop
  for (;;)
  {
    // Waits for a signal to the semaphore associated with the calling thread.
    // Note that the semaphore associated with a thread is signaled when a
    // message is queued to the message receive queue of the thread or when
    // ICall_signal() function is called onto the semaphore.
    ICall_Errno errno = ICall_wait(ICALL_TIMEOUT_FOREVER);

    #if defined (NPI_USE_UART) || defined (NPI_USE_SPI)     
      //TL handles driver events. this must be done first
      TL_handleISRevent();
    #endif //TL

    if (errno == ICALL_ERRNO_SUCCESS)
    {
      ICall_EntityID dest;
      ICall_ServiceEnum src;
      ICall_HciExtEvt *pMsg = NULL;

      if (ICall_fetchServiceMsg(&src, &dest,
                                (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
      {
        uint8 safeToDealloc = TRUE;
        
        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
        {
          ICall_Event *pEvt = (ICall_Event *)pMsg;
          
          // Check for BLE stack events first
          if (pEvt->signature == 0xffff)
          {
            if (pEvt->event_flag & SBP_CONN_EVT_END_EVT)
            {
              // Try to retransmit pending ATT Response (if any)
              SimpleBLEPeripheral_sendAttRsp();
            }
          }
          else
          {
            // Process inter-task message
            safeToDealloc = SimpleBLEPeripheral_processStackMsg(
                                                             (ICall_Hdr *)pMsg);
          }
        }

        if (pMsg && safeToDealloc)
        {
          ICall_freeMsg(pMsg);
        }
      }

      // If RTOS queue is not empty, process app message.
      while (!Queue_empty(appMsgQueue))
      {
        sbpEvt_t *pMsg = (sbpEvt_t *)Util_dequeueMsg(appMsgQueue);
        if (pMsg)
        {
          // Process message.
          SimpleBLEPeripheral_processAppMsg(pMsg);

          // Free the space from the message.
          ICall_free(pMsg);
        }
      }
    }

    if (events & SBP_PERIODIC_EVT)
    {
      events &= ~SBP_PERIODIC_EVT;

      //Util_startClock(&periodicClock);

      // Perform periodic application task
      SimpleBLEPeripheral_performPeriodicTask();
    }
    
    
    if (events & SBP_PERIODIC_EVT_BAT)
    {
      events &= ~SBP_PERIODIC_EVT_BAT; //clear event
      Util_startClock(&periodicClockBat);
      {
        static uint16_t u16_adcVal = 0x0000;
        
        initAdcBat();
        triggerAdcTemp();
        while (HWREGBITW(AUX_EVCTL_BASE + AUX_EVCTL_O_EVTOMCUFLAGSCLR, AUX_EVCTL_EVTOMCUFLAGSCLR_ADC_IRQ_BITN == 0)){};
        u16_adcVal = AUXADCReadFifo();
        HWREGBITW(AUX_EVCTL_BASE + AUX_EVCTL_O_EVTOMCUFLAGSCLR, AUX_EVCTL_EVTOMCUFLAGSCLR_ADC_IRQ_BITN) = 1;
        // Allow STANDBY mode again
        Power_releaseConstraint(Power_SB_DISALLOW);

        // Disable ADC
        AUXADCDisable();

        // Restore pins to values in BoardGpioTable
        PIN_close(hSbpPins_ADC);
        u16_adcVal = ADC_getBatteryLevel(u16_adcVal);
        SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR5, sizeof ( uint8 )*2, &u16_adcVal ); //Battery
      }
    }
    
    
    
#ifdef FEATURE_OAD
    while (!Queue_empty(hOadQ))
    {
      oadTargetWrite_t *oadWriteEvt = Queue_dequeue(hOadQ);

      // Identify new image.
      if (oadWriteEvt->event == OAD_WRITE_IDENTIFY_REQ)
      {
        OAD_imgIdentifyWrite(oadWriteEvt->connHandle, oadWriteEvt->pData);
      }
      // Write a next block request.
      else if (oadWriteEvt->event == OAD_WRITE_BLOCK_REQ)
      {
        OAD_imgBlockWrite(oadWriteEvt->connHandle, oadWriteEvt->pData);
      }

      // Free buffer.
      ICall_free(oadWriteEvt);
    }
#endif //FEATURE_OAD
  }
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_processStackMsg
 *
 * @brief   Process an incoming stack message.
 *
 * @param   pMsg - message to process
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t SimpleBLEPeripheral_processStackMsg(ICall_Hdr *pMsg)
{
  uint8_t safeToDealloc = TRUE;
    
  switch (pMsg->event)
  {
    case GATT_MSG_EVENT:
      // Process GATT message
      safeToDealloc = SimpleBLEPeripheral_processGATTMsg(
                                                        (gattMsgEvent_t *)pMsg);
      break;

    case HCI_GAP_EVENT_EVENT:
      {
        // Process HCI message
        switch(pMsg->status)
        {
          case HCI_COMMAND_COMPLETE_EVENT_CODE:
            // Process HCI Command Complete Event
            break;
            
          default:
            break;
        }
      }
      break;
      
    default:
      // do nothing
      break;
  }
  
  return (safeToDealloc);
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_processGATTMsg
 *
 * @brief   Process GATT messages and events.
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t SimpleBLEPeripheral_processGATTMsg(gattMsgEvent_t *pMsg)
{
  // See if GATT server was unable to transmit an ATT response
  if (pMsg->hdr.status == blePending)
  {
    // No HCI buffer was available. Let's try to retransmit the response
    // on the next connection event.
    if (HCI_EXT_ConnEventNoticeCmd(pMsg->connHandle, selfEntity,
                                   SBP_CONN_EVT_END_EVT) == SUCCESS)
    {
      // First free any pending response
      SimpleBLEPeripheral_freeAttRsp(FAILURE);
      
      // Hold on to the response message for retransmission
      pAttRsp = pMsg;
      
      // Don't free the response message yet
      return (FALSE);
    }
  }
  else if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT)
  {
    // ATT request-response or indication-confirmation flow control is
    // violated. All subsequent ATT requests or indications will be dropped.
    // The app is informed in case it wants to drop the connection.
    
    // Display the opcode of the message that caused the violation.
    LCD_WRITE_STRING_VALUE("FC Violated:", pMsg->msg.flowCtrlEvt.opcode,
                           10, LCD_PAGE5);
  }    
  else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
  {
    // MTU size updated
    LCD_WRITE_STRING_VALUE("MTU Size:", pMsg->msg.mtuEvt.MTU, 10, LCD_PAGE5);
  }
  
  // Free message payload. Needed only for ATT Protocol messages
  GATT_bm_free(&pMsg->msg, pMsg->method);
  
  // It's safe to free the incoming message
  return (TRUE);
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_sendAttRsp
 *
 * @brief   Send a pending ATT response message.
 *
 * @param   none
 *
 * @return  none
 */
static void SimpleBLEPeripheral_sendAttRsp(void)
{
  // See if there's a pending ATT Response to be transmitted
  if (pAttRsp != NULL)
  {
    uint8_t status;
    
    // Increment retransmission count
    rspTxRetry++;
    
    // Try to retransmit ATT response till either we're successful or
    // the ATT Client times out (after 30s) and drops the connection.
    status = GATT_SendRsp(pAttRsp->connHandle, pAttRsp->method, 
                          &(pAttRsp->msg));
    if ((status != blePending) && (status != MSG_BUFFER_NOT_AVAIL))
    {
      // Disable connection event end notice
      HCI_EXT_ConnEventNoticeCmd(pAttRsp->connHandle, selfEntity, 0);
      
      // We're done with the response message
      SimpleBLEPeripheral_freeAttRsp(status);
    }
    else
    {
      // Continue retrying
      LCD_WRITE_STRING_VALUE("Rsp send retry:", rspTxRetry, 10, LCD_PAGE5);
    }
  }
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_freeAttRsp
 *
 * @brief   Free ATT response message.
 *
 * @param   status - response transmit status
 *
 * @return  none
 */
static void SimpleBLEPeripheral_freeAttRsp(uint8_t status)
{
  // See if there's a pending ATT response message
  if (pAttRsp != NULL)
  {
    // See if the response was sent out successfully
    if (status == SUCCESS)
    {
      LCD_WRITE_STRING_VALUE("Rsp sent, retry:", rspTxRetry, 10, LCD_PAGE5);
    }
    else
    {
      // Free response payload
      GATT_bm_free(&pAttRsp->msg, pAttRsp->method);
      
      LCD_WRITE_STRING_VALUE("Rsp retry failed:", rspTxRetry, 10, LCD_PAGE5);
    }
    
    // Free response message
    ICall_freeMsg(pAttRsp);
    
    // Reset our globals
    pAttRsp = NULL;
    rspTxRetry = 0;
  }
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_processAppMsg
 *
 * @brief   Process an incoming callback from a profile.
 *
 * @param   pMsg - message to process
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_processAppMsg(sbpEvt_t *pMsg)
{
  switch (pMsg->hdr.event)
  {
    case SBP_STATE_CHANGE_EVT:
      SimpleBLEPeripheral_processStateChangeEvt((gaprole_States_t)pMsg->
                                                hdr.state);
      break;

    case SBP_CHAR_CHANGE_EVT:
      SimpleBLEPeripheral_processCharValueChangeEvt(pMsg->hdr.state);
      break;

    default:
      // Do nothing.
      break;
  }
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_stateChangeCB
 *
 * @brief   Callback from GAP Role indicating a role state change.
 *
 * @param   newState - new state
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_stateChangeCB(gaprole_States_t newState)
{
  SimpleBLEPeripheral_enqueueMsg(SBP_STATE_CHANGE_EVT, newState);
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_processStateChangeEvt
 *
 * @brief   Process a pending GAP Role state change event.
 *
 * @param   newState - new state
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_processStateChangeEvt(gaprole_States_t newState)
{
#ifdef PLUS_BROADCASTER
  static bool firstConnFlag = false;
#endif // PLUS_BROADCASTER

  switch ( newState )
  {
    case GAPROLE_STARTED:
      {
        uint8_t ownAddress[B_ADDR_LEN];
        uint8_t systemId[DEVINFO_SYSTEM_ID_LEN];

        GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);

        // use 6 bytes of device address for 8 bytes of system ID value
        systemId[0] = ownAddress[0];
        systemId[1] = ownAddress[1];
        systemId[2] = ownAddress[2];

        // set middle bytes to zero
        systemId[4] = 0x00;
        systemId[3] = 0x00;

        // shift three bytes up
        systemId[7] = ownAddress[5];
        systemId[6] = ownAddress[4];
        systemId[5] = ownAddress[3];

        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, 
                             systemId);

        // Display device address
        LCD_WRITE_STRING(Util_convertBdAddr2Str(ownAddress), LCD_PAGE1);
        LCD_WRITE_STRING("Initialized", LCD_PAGE2);
        
        //Battery timer //HJ
        Util_startClock(&periodicClockBat);

      }
      break;

    case GAPROLE_ADVERTISING:
      LCD_WRITE_STRING("Advertising", LCD_PAGE2);
      ppgDisconnect();
      break;

#ifdef PLUS_BROADCASTER   
    /* After a connection is dropped a device in PLUS_BROADCASTER will continue
     * sending non-connectable advertisements and shall sending this change of 
     * state to the application.  These are then disabled here so that sending 
     * connectable advertisements can resume.
     */
    case GAPROLE_ADVERTISING_NONCONN:
      {
        uint8_t advertEnabled = FALSE;
      
        // Disable non-connectable advertising.
        GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8_t),
                           &advertEnabled);
      
        advertEnabled = TRUE;
      
        // Enabled connectable advertising.
        GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                             &advertEnabled);
        
        // Reset flag for next connection.
        firstConnFlag = false;
        
        SimpleBLEPeripheral_freeAttRsp(bleNotConnected);
        ppgDisconnect();
      }
      break;
#endif //PLUS_BROADCASTER   

    case GAPROLE_CONNECTED:
      {
        uint8_t peerAddress[B_ADDR_LEN];

        GAPRole_GetParameter(GAPROLE_CONN_BD_ADDR, peerAddress);

        //Util_startClock(&periodicClock);
        //Battery Timer //HJ
        Util_startClock(&periodicClockBat);
        
        LCD_WRITE_STRING("Connected", LCD_PAGE2);
        LCD_WRITE_STRING(Util_convertBdAddr2Str(peerAddress), LCD_PAGE3);

        #ifdef PLUS_BROADCASTER
          // Only turn advertising on for this state when we first connect
          // otherwise, when we go from connected_advertising back to this state
          // we will be turning advertising back on.
          if (firstConnFlag == false)
          {
            uint8_t advertEnabled = FALSE; // Turn on Advertising

            // Disable connectable advertising.
            GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                                 &advertEnabled);
            
            // Set to true for non-connectabel advertising.
            advertEnabled = TRUE;

            // Enable non-connectable advertising.
            GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8_t),
                                 &advertEnabled);
            firstConnFlag = true;
          }
        #endif // PLUS_BROADCASTER
          ppgConnect();
      }
      break;

    case GAPROLE_CONNECTED_ADV:
      LCD_WRITE_STRING("Connected Advertising", LCD_PAGE2);
      break;

    case GAPROLE_WAITING:
      Util_stopClock(&periodicClock);
        //Battery Timer //HJ
      Util_stopClock(&periodicClockBat);
      
      SimpleBLEPeripheral_freeAttRsp(bleNotConnected);

      LCD_WRITE_STRING("Disconnected", LCD_PAGE2);
      
      // Clear remaining lines
      LCD_WRITE_STRING("", LCD_PAGE3);
      LCD_WRITE_STRING("", LCD_PAGE4);
      LCD_WRITE_STRING("", LCD_PAGE5);
      ppgDisconnect();

      break;

    case GAPROLE_WAITING_AFTER_TIMEOUT:
      SimpleBLEPeripheral_freeAttRsp(bleNotConnected);
      
      LCD_WRITE_STRING("Timed Out", LCD_PAGE2);
      
      // Clear remaining lines
      LCD_WRITE_STRING("", LCD_PAGE3);
      LCD_WRITE_STRING("", LCD_PAGE4);
      LCD_WRITE_STRING("", LCD_PAGE5);

      #ifdef PLUS_BROADCASTER
        // Reset flag for next connection.
        firstConnFlag = false;
      #endif //#ifdef (PLUS_BROADCASTER)
          ppgDisconnect();
      break;

    case GAPROLE_ERROR:
      LCD_WRITE_STRING("Error", LCD_PAGE2);
      ppgDisconnect();
      break;

    default:
      LCD_WRITE_STRING("", LCD_PAGE2);
      break;
  }

  // Update the state
  //gapProfileState = newState;
}

#ifndef FEATURE_OAD
/*********************************************************************
 * @fn      SimpleBLEPeripheral_charValueChangeCB
 *
 * @brief   Callback from Simple Profile indicating a characteristic
 *          value change.
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_charValueChangeCB(uint8_t paramID)
{
  SimpleBLEPeripheral_enqueueMsg(SBP_CHAR_CHANGE_EVT, paramID);
}
#endif //!FEATURE_OAD

/*********************************************************************
 * @fn      SimpleBLEPeripheral_processCharValueChangeEvt
 *
 * @brief   Process a pending Simple Profile characteristic value change
 *          event.
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_processCharValueChangeEvt(uint8_t paramID)
{
#ifndef FEATURE_OAD
  uint8_t newValue;
  
  switch(paramID)
  {
    case SIMPLEPROFILE_CHAR1:
      SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR1, &newValue);
      
      switch(newValue) {
        case 0x00:
          HCI_EXT_SetTxPowerCmd(LL_EXT_TX_POWER_MINUS_21_DBM);
          break;
        
        case 0x01:
          HCI_EXT_SetTxPowerCmd(LL_EXT_TX_POWER_MINUS_6_DBM);
          break;
          
        case 0x02:
          HCI_EXT_SetTxPowerCmd(LL_EXT_TX_POWER_0_DBM);
          break;
          
        case 0x03:
          HCI_EXT_SetTxPowerCmd(LL_EXT_TX_POWER_5_DBM);
          break;
          
        case 0x04:
          //HCI_EXT_SetRxGainCmd(HCI_EXT_RX_GAIN_STD);
          break;
          
        case 0x05:
          //HCI_EXT_SetRxGainCmd(HCI_EXT_RX_GAIN_HIGH);
          break;
          
        case 0x64: //75Hz
          break;
          
        case 0x65: //150Hz
          break;
          
        case 0x66: //250Hz
          break;
        
        case 0x70: //Activate Filtered
          break;

        case 0x71: //Activate Raw
          break;

          
        case 0x80: //128
          ppgConnect();
          break;
          
        case 0x81:  //129
          ppgDisconnect();
          break;
          
        case 0x85:  
          actDebugSPO2();
          break;

        case 0x86: 
          deactDebugSPO2();
          break;          

      case 0x48:  //notify char8 //130 
        snvFlag = 3 ;
        break;
          
      case 0x49:  //notify char9 //131
        snvFlag = 4 ;
        break;
        
      default:
        break;  
        
      }
            
      if (snvFlag == 3) {
        ret=osal_snv_read(0x8A, SIMPLEPROFILE_CHAR8_LEN, snvBuf);
        SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR8, SIMPLEPROFILE_CHAR8_LEN, snvBuf);
        snvFlag = 0;
      }
      
      if (snvFlag == 4) {
        ret=osal_snv_read(0x80, SIMPLEPROFILE_CHAR9_LEN, snvBuf);
        SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR9, SIMPLEPROFILE_CHAR9_LEN, snvBuf);
        snvFlag = 0;
      }

      break;        
      
    case SIMPLEPROFILE_CHAR2:
      SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR2, &newValue);
      switch(newValue) {
        case 0x00: //SpO2 OK
          break;
          
        case 0x01: //SpO2 Warning
          break;
          
        case 0x02: //SpO2 Critical
          break;

      default:
        break;  
      }
    break;
    
    
    case SIMPLEPROFILE_CHAR3:
      SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR3, &newValue);
      switch(newValue) {
        case 0x00: //Saw Wave
          ppgFormat = 0;
          for (int i=0; i<20; i++) {
            ppgTxBuf[i] = i; }
          break;
          
        case 0x01: //4x 16-bit PPG1 [XHz] PPG2 [XHz]
          ppgFormat = 1;
          break;
          
        case 0x02: //9x 16-bit PPG1
          ppgFormat = 2;
          break;
        
        case 0x03: //9x 16-bit PPG2
          ppgFormat = 3;
          break;
          
      default:
        break;  
      }
      break; 
      
    case SIMPLEPROFILE_CHAR8:
      SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR8, snvBuf);
      ret=osal_snv_write(0x8A, SIMPLEPROFILE_CHAR8_LEN, snvBuf);
      snvFlag = 0;
    break;

    case SIMPLEPROFILE_CHAR9:
      SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR9, snvBuf);
      ret=osal_snv_write(0x80, SIMPLEPROFILE_CHAR9_LEN, snvBuf);
      snvFlag = 0; 
      
    setDeviceId();
    //GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData), scanRspData);
    break;
  
      default:
      // should not reach here!
      break;
  }
#endif //!FEATURE_OAD
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_performPeriodicTask
 *
 * @brief   Perform a periodic application task. This function gets called
 *          every five seconds (SBP_PERIODIC_EVT_PERIOD). In this example,
 *          the value of the third characteristic in the SimpleGATTProfile
 *          service is retrieved from the profile, and then copied into the
 *          value of the the fourth characteristic.
 *
 * @param   None.
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_performPeriodicTask(void)
{
      if ((TurnOnPpg==0x01) && (ppgReq>0)) {
        txbuf[0] = 0xFF;
        txbuf[1] = 0xFE;
        txbuf[2] = 0x01;
        txbuf[3] = 0x00;
        txbuf[4] = 0x00;
        TLwrite(txbuf, 5); 
        Util_startClock(&periodicClock);
        ppgReq--;
      }
      if ((TurnOffPpg==0x01) && (ppgReq>0)) {
        txbuf[0] = 0xFF;
        txbuf[1] = 0xFE;
        txbuf[2] = 0x01;
        txbuf[3] = 0x00;
        txbuf[4] = 0x01;
        TLwrite(txbuf, 5); 
        Util_startClock(&periodicClock);   
        ppgReq--;
      }
      
      //did not receive activate debugSPO2 ack
      if ((TurnOnDebugSPO2==0x01) && (debugSPO2Req>0)) {
        txbuf[0] = 0xFF;
        txbuf[1] = 0xFE;
        txbuf[2] = 0x01;
        txbuf[3] = 0x00;
        txbuf[4] = 0x0D;
        //txbuf[5] = 0x01;
        TLwrite(txbuf, 5); 
        Util_startClock(&periodicClock);
        debugSPO2Req--;
      }
      
      //did not receive disable debugSPO2 ack
      if ((TurnOffDebugSPO2==0x01) && (debugSPO2Req>0)) {
        txbuf[0] = 0xFF;
        txbuf[1] = 0xFE;
        txbuf[2] = 0x01;
        txbuf[3] = 0x00;
        txbuf[4] = 0x0E;
        //txbuf[5] = 0x00;
        TLwrite(txbuf, 5); 
        Util_startClock(&periodicClock);   
        debugSPO2Req--;
      }
}


#if defined(FEATURE_OAD)
/*********************************************************************
 * @fn      SimpleBLEPeripheral_processOadWriteCB
 *
 * @brief   Process a write request to the OAD profile.
 *
 * @param   event      - event type:
 *                       OAD_WRITE_IDENTIFY_REQ
 *                       OAD_WRITE_BLOCK_REQ
 * @param   connHandle - the connection Handle this request is from.
 * @param   pData      - pointer to data for processing and/or storing.
 *
 * @return  None.
 */
void SimpleBLEPeripheral_processOadWriteCB(uint8_t event, uint16_t connHandle,
                                           uint8_t *pData)
{
  oadTargetWrite_t *oadWriteEvt = ICall_malloc( sizeof(oadTargetWrite_t) + \
                                             sizeof(uint8_t) * OAD_PACKET_SIZE);
  
  if ( oadWriteEvt != NULL )
  {
    oadWriteEvt->event = event;
    oadWriteEvt->connHandle = connHandle;
    
    oadWriteEvt->pData = (uint8_t *)(&oadWriteEvt->pData + 1);
    memcpy(oadWriteEvt->pData, pData, OAD_PACKET_SIZE);

    Queue_enqueue(hOadQ, (Queue_Elem *)oadWriteEvt);
    
    // Post the application's semaphore.
    Semaphore_post(sem);
  }
  else
  {
    // Fail silently.
  }
}
#endif //FEATURE_OAD

/*********************************************************************
 * @fn      SimpleBLEPeripheral_clockHandler
 *
 * @brief   Handler function for clock timeouts.
 *
 * @param   arg - event type
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_clockHandler(UArg arg)
{
  // Store the event.
  events |= arg;

  // Wake up the application.
  Semaphore_post(sem);
}

static void SimpleBLEPeripheral_clockHandler_BAT(UArg arg)
{
  // Store the event.
  events |= arg;

  // Wake up the application.
  Semaphore_post(sem);
}
/*********************************************************************
 * @fn      SimpleBLEPeripheral_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_enqueueMsg(uint8_t event, uint8_t state)
{
  sbpEvt_t *pMsg;

  // Create dynamic pointer to message.
  if ((pMsg = ICall_malloc(sizeof(sbpEvt_t))))
  {
    pMsg->hdr.event = event;
    pMsg->hdr.state = state;

    // Enqueue the message.
    Util_enqueueMsg(appMsgQueue, sem, (uint8*)pMsg);
  }
}


#if defined (NPI_USE_UART) || defined (NPI_USE_SPI) 
static void SimpleBLEPeripheral_TLpacketParser(void)
{
  //read available bytes
  uint8_t len = TLgetRxBufLen();
  if (len >= APP_TL_BUFF_SIZE)
  {
    len = APP_TL_BUFF_SIZE;
  }
  TLread(appRxBuf, len);
    //ppgConnect ack
    if ((len==5) && (appRxBuf[0]==0xFF) && (appRxBuf[1]==0xFE) && (appRxBuf[2]==0x01) && (appRxBuf[3]==0x02) && (appRxBuf[4]==0x04)) {
      TurnOnPpg=0x00;
      ppgReq=5;
    }
    //ppgDisconnect ack
    if ((len==5) && (appRxBuf[0]==0xFF) && (appRxBuf[1]==0xFE) && (appRxBuf[2]==0x01) && (appRxBuf[3]==0x02) && (appRxBuf[4]==0x05)) {
      TurnOffPpg=0x00;
      ppgReq=5;
    }
       
    //actDebugSPO2 ack
    if ((len==5) && (appRxBuf[0]==0xFF) && (appRxBuf[1]==0xFE) && (appRxBuf[2]==0x01) && (appRxBuf[3]==0x02) && (appRxBuf[4]==0x0D)) {
      TurnOnDebugSPO2=0x00;
      debugSPO2Req=5;
    }
    
    //deactDebugSPO2 ack
    if ((len==5) && (appRxBuf[0]==0xFF) && (appRxBuf[1]==0xFE) && (appRxBuf[2]==0x01) && (appRxBuf[3]==0x02) && (appRxBuf[4]==0x0D)) {
      TurnOffDebugSPO2=0x00;
      debugSPO2Req=5;
    }
    
      switch(appRxBuf[3])
  {
    case 0x06:
    if (appRxBuf[0]==0xFF) {
      ppgTxBuf[1+formatCounter*2] = appRxBuf[5];
      ppgTxBuf[2+formatCounter*2] = appRxBuf[4];
      ppgTxBuf[5+formatCounter*2] = appRxBuf[7];
      ppgTxBuf[6+formatCounter*2] = appRxBuf[6];
      formatCounter++;
        if (formatCounter>=2) {
        SN++;
        ppgTxBuf[0]=SN;
        SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4, SIMPLEPROFILE_CHAR4_LEN, ppgTxBuf);
        formatCounter=0;
        }
    }
    else {
    formatCounter=0;}
      break;

    case 0x07:
    if (appRxBuf[0]==0xFF) {
      
      switch (ppgFormat) {
        case 0x00:
          for (int i=0; i<20; i++) {
            ppgTxBuf[i]++; 
          }
          SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4, SIMPLEPROFILE_CHAR4_LEN, ppgTxBuf);
          break;
          
        case 0x01:
          ppgTxBuf[2+(formatCounter*2)] = appRxBuf[21];
          ppgTxBuf[3+(formatCounter*2)] = appRxBuf[20];
          ppgTxBuf[10+(formatCounter*2)] = appRxBuf[23];
          ppgTxBuf[11+(formatCounter*2)] = appRxBuf[22];
          formatCounter++;
          if (formatCounter>=4) {
            ppgTxBuf[18] = 0x14;
            ppgTxBuf[19] = 0x14;
            SN++;
            ppgTxBuf[1]=SN & 0xFF;
            ppgTxBuf[0]=(SN >> 8);
            //if ((SN % 20) == 0 ) {
              SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4, SIMPLEPROFILE_CHAR4_LEN, ppgTxBuf);//}
            formatCounter=0;
          }
          break;
          
        case 0x02:
          ppgTxBuf[2+formatCounter*2] = appRxBuf[21];
          ppgTxBuf[3+formatCounter*2] = appRxBuf[20];
          formatCounter++;
            if (formatCounter>=9) {
            SN++;
            ppgTxBuf[1]=SN & 0xFF;
            ppgTxBuf[0]=(SN >> 8);
            //if ((SN % 20) == 0 ) {
              SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4, SIMPLEPROFILE_CHAR4_LEN, ppgTxBuf);//}
            formatCounter=0;
            }
          break;            

        case 0x03:
          ppgTxBuf[2+formatCounter*2] = appRxBuf[23];
          ppgTxBuf[3+formatCounter*2] = appRxBuf[22];
          formatCounter++;
            if (formatCounter>=9) {
            SN++;
            ppgTxBuf[1]=SN & 0xFF;
            ppgTxBuf[0]=(SN >> 8);
            //if ((SN % 5) == 0 ) {
              SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4, SIMPLEPROFILE_CHAR4_LEN, ppgTxBuf);//}
            formatCounter=0;
            }
          break;
         
        default:
      // should not reach here!
          break;
      }
    }
    else {
      formatCounter=0;
    }
    break;
    
    case 0x08:
      if (appRxBuf[0]==0xFF) {
        spo2TxBuf[0] = appRxBuf[13];
        spo2TxBuf[1] = appRxBuf[12];
        spo2TxBuf[2] = appRxBuf[15];
        spo2TxBuf[3] = appRxBuf[14];
        spo2TxBuf[4] = appRxBuf[17];
        spo2TxBuf[5] = appRxBuf[16];
        spo2TxBuf[6] = appRxBuf[19];
        spo2TxBuf[7] = appRxBuf[18];
        spo2TxBuf[8] = appRxBuf[21];
        spo2TxBuf[9] = appRxBuf[20];
        SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR7, SIMPLEPROFILE_CHAR7_LEN, spo2TxBuf);
      }
      break;

    default:
      // should not reach here!
      break;
  }

  // ADD PACKET PARSER HERE
  // for now we just echo...
}
#endif //TL

static void ppgConnect(void) {         
    txbuf[0] = 0xFF;
    txbuf[1] = 0xFE;
    txbuf[2] = 0x01;
    txbuf[3] = 0x00;
    txbuf[4] = 0x00;
    
    for (int i = 0; i < 5; i++) {
    TLwrite(&(txbuf[i]), 1); 
    }

    Util_startClock(&periodicClock);
    TurnOnPpg=0x01;
    ppgReq=5;
}

static void ppgDisconnect(void) {         
    txbuf[0] = 0xFF;
    txbuf[1] = 0xFE;
    txbuf[2] = 0x01;
    txbuf[3] = 0x00;
    txbuf[4] = 0x01;

    for (int i = 0; i < 5; i++) {
      TLwrite(&(txbuf[i]), 1); 
    }

    Util_startClock(&periodicClock);
    TurnOffPpg=0x01; 
    ppgReq=5;
}

static void setDeviceId(void) {
      deviceID = snvBuf[8] * 0x100 + snvBuf[9];
      scanRspData[10] = deviceID % 10 + 0x30;
      deviceID = deviceID / 10;
      scanRspData[9] = deviceID % 10 + 0x30;
      deviceID = deviceID / 10;
      scanRspData[8] = deviceID % 10 + 0x30;
      deviceID = deviceID / 10;
      scanRspData[7] = deviceID % 10 + 0x30;
      deviceID = deviceID / 10;
      scanRspData[6] = deviceID % 10 + 0x30;
      GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData), scanRspData);
}

static void actDebugSPO2(void) {         

    txbuf[0] = 0xFF;
    txbuf[1] = 0xFE;
    txbuf[2] = 0x01;
    txbuf[3] = 0x00;
    txbuf[4] = 0x0D;
    //txbuf[5] = 0x01;
    
    for (int i = 0; i < 5; i++) {
    TLwrite(&(txbuf[i]), 1); 
    }

    Util_startClock(&periodicClock);
    TurnOnDebugSPO2=0x01;
    debugSPO2Req=5;
}

static void deactDebugSPO2(void) {

    txbuf[0] = 0xFF;
    txbuf[1] = 0xFE;
    txbuf[2] = 0x01;
    txbuf[3] = 0x00;
    txbuf[4] = 0x0E;
    //txbuf[5] = 0x00;
    
    for (int i = 0; i < 5; i++) {
    TLwrite(&(txbuf[i]), 1); 
    }

    Util_startClock(&periodicClock);
    TurnOffDebugSPO2=0x01;
    debugSPO2Req=5;
}

/*********************************************************************
*********************************************************************/