//==============================================================================
//
//  Cellular.c
//
//  Copyright (C) 2014 by Industrial Scientific.
//
//  This document and all  contained within are confidential and
//  proprietary property of Industrial Scientific Corporation. All rights 
//  reserved. It is not to be reproduced or reused without the prior approval 
//  of Industrial Scientific Corporation.
//
//==============================================================================
//  FILE 
//==============================================================================
//
//  Source:        Cellular.c
//
//  Project:       Frey
//
//  Author:        Dilawar Ali
//
//  Date:          2018/05/17
//
//  Revision:      1.0
//
//==============================================================================
//  FILE DESCRIPTION
//==============================================================================
//
//! \file
//! This file contains the prototypes of global functions and declaration of global 
//! data used in External Communication module. This module provides support for external
//! communication e.g json creation and response parsing.
//


//==============================================================================
//  INCLUDES
//==============================================================================
#include "Cellular.h"

#include <stdio.h>
#include <dmadrv.h>
#include <em_cmu.h>

#include "main.h"
#include "Timer.h"
//==============================================================================
//  CONSTANTS, TYPEDEFS AND MACROS 
//==============================================================================
const uint8_t configureI2CPortCmd[] = {0xB5,0x62, 0x06, 0x00, 0x14,0x00, 0x00,0x00, 0x3D,0x08, 0x84,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, 0x03,0x00, 0x03,0x00, 0x00,0x00, 0x00,0x00, 0xE9, 0x84};

uint8_t resetGNSS[] = {0xB5,0x62, 0x06, 0x04, 0x04,0x00, 0x00,0x00, 0x00, 0x00, 0x0E,0x64};
uint8_t CFG_MSG[]  = {0xB5,0x62, 0x06, 0x01, 0x08,0x00, 0xF0,0x00, 0x02,0x00,0x00,0x00,0x00,0x00, 0x01,0x2F};
uint8_t CFG_MSG_GET[]  = {0xB5,0x62, 0x06, 0x01, 0x02,0x00, 0xF0,0x00, 0xF9,0x11};

uint8_t const CMD_GPS_CONFIG_GPS[] = {0xB5,0x62,0x06,0x3E,0x0C,0x00,0x00,0x20,0x20,0x01,0x00,0x08,0x10,0x00,0x30,0x5A,0x01,0x01,0x35,0xBC};
//==============================================================================
//  LOCAL DATA DECLARATIONS
//==============================================================================

#define PORT_CELL_V_INT    gpioPortC
#define PIN_CELL_V_INT     13u

#define PORT_CELL_NETWORK  gpioPortB
#define PIN_CELL_NETWORK     13u 

#define PORT_CELL_RESET_N  gpioPortB
#define PIN_CELL_RESET_N   14u

#define PORT_CELL_POWER_ON  gpioPortB
#define PIN_CELL_POWER_ON    11u

////////////  GPS pins //////////////////

#define PORT_GPS_ENABLE    gpioPortB
#define PIN_GPS_ENABLE     8u 

#define PORT_GPS_RESET_N    gpioPortB
#define PIN_GPS_RESET_N     7u 

#define PORT_GPS_SOFTRESET_N    gpioPortC
#define PIN_GPS_SOFTRESET_N     1u 

#define PORT_GPS_TIME_PULSE    gpioPortC
#define PIN_GPS_TIME_PULSE     0u 

#define PORT_GPS_EXT_INT    gpioPortA
#define PIN_GPS_EXT_INT     2u 


UARTDRV_HandleData_t cellUARTHandleData = NULL;
UARTDRV_Handle_t cellUART = &cellUARTHandleData;

static BOOLEAN isGPSinit = false;

//==============================================================================
//  LOCAL FUNCTION PROTOTYPES
//==============================================================================
static void CellularUARTOpen(void);
//static void EnableCellularModule(void);
static void UARTReadParser(  uint8_t Response[], size_t count );
static bool CellularATCMECheck(uint8_t *ResponseBuffer, int32_t length );
static uint32_t UARTReadBlocking(UARTDRV_Handle_t uart,  uint8_t buffer[], uint32_t readSize );
static int32_t WarmupCellularModule(void);
static int32_t ConfigureCertificate(void);
static int32_t PostDataToiNet(void);
static int32_t PerformCellularRecovery(int32_t errorCode);
static int32_t CellularDeviceWrite( ATCOMMAND_INDEX_ENUM at_idx );
static void WaitForCellularGetReady(void);

static int32_t GPSConfigure(void);
static void ClearRxBuffer(void);
static void ConfigureCellularPins(void);
static void CellularPowerUp(void);
static void CellularModuleReset(void);
static int32_t CellularInit(void);

//==============================================================================
//  GLOBAL DATA DECLARATIONS
//==============================================================================
CellularDriver_t gCellularDriver;
ReceivedDataInfo_t cellHttpsReceiving;

uint8_t cellDataBuffer[CELLULAR_DATA_BUFFER_SIZE];
uint8_t httpUrlBuffer[CELLULAR_URL_BUFFER_SIZE];
uint8_t tokenBuffer[MAX_JSON_TOKEN_STRING_SIZE];
uint8_t cellHeaderBuffer[CELLULAR_HEADER_BUFFER_SIZE];

uint8_t cellular_initialized=0;
uint8_t gps_initialized=0;

//==============================================================================
//  LOCAL FUNCTIONS IMPLEMENTATION
//==============================================================================

//------------------------------------------------------------------------------
//  int32_t CellularUARTOpen(void)
//
//   Author:  Dilawar Ali
//   Date:    2018/05/17
//
//!  This function Open Cellular UART
//
//------------------------------------------------------------------------------
static void CellularUARTOpen(void)
{
    static BOOLEAN isUARTOpen = false;
    DEFINE_BUF_QUEUE(EMDRV_UARTDRV_MAX_CONCURRENT_RX_BUFS, rxBufferQueue);
    DEFINE_BUF_QUEUE(EMDRV_UARTDRV_MAX_CONCURRENT_TX_BUFS, txBufferQueue);
    
    // Initialize driver handle
    UARTDRV_InitUart_t initData =   {
        UART0,
        115200,
        _UART_ROUTELOC0_TXLOC_LOC3,
        _UART_ROUTELOC0_RXLOC_LOC3,
        usartStopbits1,
        usartNoParity,
        usartOVS16,
        false,
        uartdrvFlowControlNone,
        NULL,
        NULL,
        NULL,
        NULL,
        (UARTDRV_Buffer_FifoQueue_t *)&rxBufferQueue,
        (UARTDRV_Buffer_FifoQueue_t *)&txBufferQueue,
        NULL,
        NULL
    };
    if(isUARTOpen == true)
    {
        UARTDRV_DeInit(cellUART);
    }
    if(UARTDRV_InitUart(cellUART, &initData) == 0)
    {
        isUARTOpen = true;
    }
}

static void ClearRxBuffer(void)
{
    memset(gCellularDriver.UARTRxBuffer,0,CELLULAR_UART_RX_BUFFER_SIZE);
}

/*
//------------------------------------------------------------------------------
//  void EnableCellularModule(void)
//
//   Author:  Dilawar Ali
//   Date:    2018/05/29
//
//!  This function Enable cellular Power and reset
//
//------------------------------------------------------------------------------
static void EnableCellularModule(void)
{
    RTOS_ERR  err;
    
    // Reset the Cellular Module
    GPIO_PinModeSet(gpioPortB, 11, gpioModePushPull, 1);
    OSTimeDly(1000, OS_OPT_TIME_DLY, &err);
    GPIO_PinModeSet(gpioPortB, 14, gpioModePushPull, 0);
    OSTimeDly(12000, OS_OPT_TIME_DLY, &err);
    GPIO_PinModeSet(gpioPortB, 14, gpioModePushPull, 1);
    OSTimeDly(500, OS_OPT_TIME_DLY, &err);
    
    // Trigger Cell internal module to init
    GPIO_PinModeSet(gpioPortB, 11, gpioModePushPull, 1);
    OSTimeDly(1000, OS_OPT_TIME_DLY, &err);
    GPIO_PinModeSet(gpioPortB, 11, gpioModePushPull, 0);
    OSTimeDly(3000, OS_OPT_TIME_DLY, &err);
    GPIO_PinModeSet(gpioPortB, 11, gpioModePushPull, 1);
    OSTimeDly(1000, OS_OPT_TIME_DLY, &err);
}
*/
//------------------------------------------------------------------------------
//  static void ConfigureCellularPins(void)
//
//   Author:  Abdul Basit
//   Date:    2018/07/09
//
//!  
//
//------------------------------------------------------------------------------
static void ConfigureCellularPins(void)
{
    // CELL_V_INT  PC13 "input"
    GPIO_PinModeSet(PORT_CELL_V_INT, PIN_CELL_V_INT, gpioModeInputPull, 0);
    
    //CELL_RESET_N PB14 "output" default '1'
    GPIO_PinModeSet(PORT_CELL_RESET_N, PIN_CELL_RESET_N, gpioModePushPull, 1);
    
    //CELL_POWER_ON PB11 "ouput" default '1'
    GPIO_PinModeSet(PORT_CELL_POWER_ON, PIN_CELL_POWER_ON, gpioModePushPull, 1);
    
    //CELL_NETWORK  PB13  "input"
    GPIO_PinModeSet(PORT_CELL_NETWORK, PIN_CELL_NETWORK, gpioModeInputPull, 0);
    
    //gps enable pin
    GPIO_PinModeSet(PORT_GPS_ENABLE, PIN_GPS_ENABLE, gpioModePushPull, 0);
    //gps reset pin
    GPIO_PinModeSet(PORT_GPS_RESET_N, PIN_GPS_RESET_N, gpioModePushPull, 1);
    //gps soft reset pin
    GPIO_PinModeSet(PORT_GPS_SOFTRESET_N, PIN_GPS_SOFTRESET_N, gpioModePushPull, 1);
    //gps time pulse input
    GPIO_PinModeSet(PORT_GPS_TIME_PULSE, PIN_GPS_TIME_PULSE, gpioModeInputPull, 0);
    
    //gps interrupt
    GPIO_PinModeSet(PORT_GPS_EXT_INT, PIN_GPS_EXT_INT, gpioModeInputPull, 0);
    
}
//------------------------------------------------------------------------------
//  static void CellularPowerUp(void)
//
//   Author:  Abdul Basit
//   Date:    2018/07/09
//
//!  This function trigger the cellular to init its internal modules
//
//------------------------------------------------------------------------------
static void CellularPowerUp(void)
{
    RTOS_ERR  err;
    // power up GSM module
    GPIO_PinModeSet(PORT_CELL_POWER_ON, PIN_CELL_POWER_ON, gpioModePushPull, 1);
    OSTimeDly(3000, OS_OPT_TIME_DLY, &err);
    GPIO_PinModeSet(PORT_CELL_POWER_ON, PIN_CELL_POWER_ON, gpioModePushPull, 0);
    OSTimeDly(3000, OS_OPT_TIME_DLY, &err);
    
}
//------------------------------------------------------------------------------
//  static void CellularModuleReset(void)
//
//   Author:  Abdul Basit
//   Date:    2018/07/09
//
//!  This function reset the cellular MCU and its Radio
//
//------------------------------------------------------------------------------
static void CellularModuleReset(void)
{
    RTOS_ERR  err;
    // Reset the Cellular Module
    GPIO_PinModeSet(PORT_CELL_RESET_N, PIN_CELL_RESET_N, gpioModePushPull, 0);
//    OSTimeDly(12000, OS_OPT_TIME_DLY, &err);
    OSTimeDly(3000, OS_OPT_TIME_DLY, &err);
    OSTimeDly(3000, OS_OPT_TIME_DLY, &err);
    OSTimeDly(3000, OS_OPT_TIME_DLY, &err);
    OSTimeDly(3000, OS_OPT_TIME_DLY, &err);
    GPIO_PinModeSet(PORT_CELL_RESET_N, PIN_CELL_RESET_N, gpioModePushPull, 1);
    OSTimeDly(500, OS_OPT_TIME_DLY, &err);
}

//------------------------------------------------------------------------------
//  static int CellularGetGPSData(void)
//
//   Author:  Abdul Basit
//   Date:    2018/07/10
//
//!  
//
//------------------------------------------------------------------------------
static int CellularGetGPSData(void)
{
    int status=0;
    RTOS_ERR  err;
   
    if(isGPSinit == false)
    {
        CellularDeviceWrite(ATC_GNSS_ON);
        isGPSinit = true;
        printf("GPS ON\r\n");
    }

    if(status>=0)
    {
        status = CellularDeviceWrite(ATC_UGGGA_DATA);
    }
    //printf("GGA: %s\r\n",gCellularDriver.UARTRxBuffer);
    OSTimeDly(2000, OS_OPT_TIME_DLY, &err); // delay 
    if(status>=0)
    {
//        status = CellularDeviceWrite(ATC_UGGSV_ENABLE);
    }
//    OSTimeDly(2000, OS_OPT_TIME_DLY, &err); // delay 
    if(status>=0)
    {
        status = CellularDeviceWrite(ATC_UGGSV_DATA);
    }
    
    return status;
    
}


//------------------------------------------------------------------------------
//  int32_t CellularDeviceWrite( ATCOMMAND_INDEX_ENUM at_idx )
//
//   Author:  Dilawar Ali
//   Date:    2018/05/29
//
//!  This function Write AT command to cellular UART and read corresponding response
//
//------------------------------------------------------------------------------
int32_t dataSize = 0;
static int32_t CellularDeviceWrite( ATCOMMAND_INDEX_ENUM at_idx )
{
    RTOS_ERR  err;
    int32_t ret = 0;
    uint32_t referenceTime = 0, currentTime = 0;;
    int32_t remainingSize = 0, writeSize = 0;
    uint8_t *inputPtr = NULL;
    // See UART is Open Or not
    if(gCellularDriver.cellUART != NULL)
    {
        ClearRxBuffer();
        cellHttpsReceiving.receivedBytes = 0;
        cellHttpsReceiving.UARTWaitCounter = 0;
        gCellularDriver.currentATIndex = at_idx;
        
        inputPtr = CelluarATCommands[at_idx].cmd;
        remainingSize = strlen((char const*)CelluarATCommands[at_idx].cmd);
        
//        printf("Tx Data: %s\r\n", inputPtr);
        
        if(remainingSize > 0)
        {
            // MAX transaction size supported by DMA is 1024
            // Data greater than 1KB will be written in Chunks
            do
            {
                // Get 1024 bytes or all if less than 1024
                dataSize = FIND_MIN(CELLULAR_UART_TX_BUFFER_SIZE, remainingSize);
                
                memcpy(gCellularDriver.UARTTxBuffer, &inputPtr[writeSize], dataSize);
                gCellularDriver.UARTTxBuffer[dataSize] = 0;
                
                // write the AT command or data to the cellular modem
                ret = UARTDRV_TransmitB(gCellularDriver.cellUART, gCellularDriver.UARTTxBuffer, dataSize);
                remainingSize -= dataSize;
                writeSize += dataSize;
            }
            while(remainingSize > 0);
        }
        if(CelluarATCommands[at_idx].timeout > (uint32_t) 0)
        {
            // if Module will take some time to prepare response
            if(CelluarATCommands[at_idx].responseDelayTime > 0)
            {
                //Sleep task for some required time
                OSTimeDly(CelluarATCommands[at_idx].responseDelayTime, OS_OPT_TIME_DLY, &err);
            }
            
            if(CelluarATCommands[at_idx].cmp == NULL)
            {
                //Command parser function is not defined
                ret = ERR_AT_CMD_PARSER_FUCN_UNDEFINED;
            }
            else
            {
                // Get the start time as reference
                referenceTime = GetRTCTicks();
                cellHttpsReceiving.endReading = 0;
                // Keep reaceiving data untill error or data is received
                while(cellHttpsReceiving.endReading == 0)
                {
                    // read UART in blocking mode
                    ret = UARTReadBlocking(gCellularDriver.cellUART, &gCellularDriver.UARTRxBuffer[cellHttpsReceiving.receivedBytes], (CelluarATCommands[at_idx].responseBytes /*- cellHttpsReceiving.receivedBytes*/));
                    
                    if(ret > 0)
                    {
                        // Parse the received data
                        if (( at_idx == ATC_USOWR) || (at_idx == ATC_ATE))
                        {
                            UARTReadParser(&gCellularDriver.UARTRxBuffer[0], ret);
                        }
                        else
                        {
                            // Skip \r\n
                            UARTReadParser(&gCellularDriver.UARTRxBuffer[2], (ret - 2));
                        }
                    }
                    if(cellHttpsReceiving.endReading == 0)
                    {
                        currentTime = GetRTCTicks();
                        // If UART read timeout
                        if( currentTime >= (referenceTime + CelluarATCommands[at_idx].timeout) )
                        {
                            // UART Reading Timeout
                            cellHttpsReceiving.endReading = ERR_UART_RX_TIMEOUT;
                            break;
                        }
                    }   
                }
                ret = cellHttpsReceiving.endReading;
            }
        }
        else
        {
            // Command has no response
            ret = 0;
        }
    }
    else
    {
        // What you doing here?
        // 8 UART is not Open
        ret = ERR_UART_NOT_OPEN;
    }
    
    return ret;
}


//==============================================================================
//
//  void UARTReadParser(uint8_t Response[], size_t count)
//
//  Author:     Dilawar Ali
//  Date:       2018/05/30
//
//!  This function Parse the data received from cellular UART after writing
//!  AT command to the Module
//=============================================================================
static void UARTReadParser(  uint8_t Response[], size_t count )
{
    if(count >= 1u)
    {
//            printf("Rx Data: %s\r\n", Response);
        // Check whether the response is a CME message
        if(CellularATCMECheck(Response, (int32_t)count) == false)
        {
            // Check whether the correct response is returned
            if(CelluarATCommands[gCellularDriver.currentATIndex].cmp(Response, (int32_t)count) == 0)
            {
                // Stop the read callback function
                cellHttpsReceiving.endReading = 1;
            }
        }
    }
}

//==============================================================================
//
//  bool CellularATCMECheck(uint8_t *ResponseBuffer, int32_t length)
//
//  Author:     Dilawar Ali
//  Date:       2018/05/30
//
//!  Check wheter the response string from cellular modem is a ME Error Result Code.
//
//! \return true: there is an error result code;  \n
//! \return false: no error code.
//=============================================================================
static bool CellularATCMECheck(uint8_t *ResponseBuffer, int32_t length )
{
    bool ret = false;
    if(strncmp((char const*)ResponseBuffer, "+CME ERROR:", 11u) ==  0)
    {
        // CME Error
        cellHttpsReceiving.endReading = ERR_CELLULAR_CME_ERROR;
        ret = true;
    }
    
    return ret;
}

//==============================================================================
//
//  static uint32_t UARTReadBlocking(UARTDRV_Handle_t uart,  uint8_t buffer[], uint32_t readSize )
//
//  Author:     Dilawar Ali
//  Date:       2018/05/30
//
//!  This function Parse the data received from cellular UART after writing
//!  AT command to the Module
//=============================================================================
static uint32_t UARTReadBlocking(UARTDRV_Handle_t uart,  uint8_t buffer[], uint32_t readSize )
{
    int32_t ret = ERR_UNKNOWN_ERROR;
    cellHttpsReceiving.isUARTReadStarted = true;
    cellHttpsReceiving.readyBytes = 0;
    ret = UARTDRV_ReceiveB(uart, buffer, readSize);
    if(ret == 0)
    {
        cellHttpsReceiving.isUARTReadStarted = false;
        cellHttpsReceiving.readyBytes = readSize;
        cellHttpsReceiving.UARTWaitCounter = 0;
    }
    cellHttpsReceiving.receivedBytes += cellHttpsReceiving.readyBytes;
    buffer[cellHttpsReceiving.receivedBytes] = 0u;
    return cellHttpsReceiving.receivedBytes;
}


//==============================================================================
//
//  int32_t WarmupCellularModule(void)
//
//  Author:     Dilawar Ali
//  Date:       2018/05/30
//
//!  This function will warmup the cellular module and get configure the module
//=============================================================================
static int32_t WarmupCellularModule(void)
{
    int32_t ret = -1;
    uint32_t loopCounter = 0;
    uint8_t retryCount= 0;
    // Get necessary Cellular Data data
    for(loopCounter = 0; loopCounter < 3; loopCounter++)
    {
        // test Module is it responding?
        ret = CellularDeviceWrite(ATC_ATE);
        if( ret >= 0)
        {
            //      printf("Module Test Passed\r\n");
            break;
        }
    }
    if(ret < 0)
    {
        // If module didn't responded declare it faulty
        if(ret != ERR_CELLULAR_CME_ERROR)
        {
            ret = ERR_MODULE_NOT_RESPONDING;
            gCellularDriver.errorCode = ERR_MODULE_NOT_RESPONDING;
        }
    }
    else
    {
        // Set the module verbose error 
        ret = CellularDeviceWrite(ATC_CMEE);
        if(ret >= 0)
        {
            //      printf("Verbose Error Enabled\r\n");
        }
        // Check the SIM card status
        for(loopCounter = 0; loopCounter < 3; loopCounter++)
        {
            ret = CellularDeviceWrite(ATC_CPIN_Q);
            if(ret >= 0)
            {
                //        printf("Sim Card is connected\r\n");
                break;
            }
        }
        
        if(ret >= 0)
        {
            //Get module IMEI number
            ret = CellularDeviceWrite(ATC_CGSN);
            if(ret >= 0)
            {
                //        printf("Module IMEI: %s\r\n", gCellularDriver.imei);
            }
            // Get SIM Card ICCID number
            ret = CellularDeviceWrite(ATC_ICCID);
            if(ret >= 0)
            {
                //        printf("Sim Card ICCID: %s\r\n", gCellularDriver.iccid);
            }
            
            CellularDeviceWrite(ATC_URAT);
            // Write Sim Card APN
            ret = CellularDeviceWrite(ATC_CGDCONT);
            
            do
            {
                // Check the Radio signal strength
                ret = CellularDeviceWrite(ATC_CSQ);
                if(ret >= 0)
                {
                    retryCount = 0;
                    //          printf("Cellular Radio Signal Strength: %d\r\n", gCellularDriver.signalStrength);
                }
            }while ((ret < 0) && (++retryCount < 255));

            do
            {
                // Check sim card network registration status
                ret = CellularDeviceWrite(ATC_CREG_Q);
                if(ret >= 0)
                {
                    //            printf("Sim card is Registered on Network Operator\r\n");
                }
            }while ((ret < 0) && (++retryCount < 25));
            if(ret < 0)
            {
                //        printf("Unable to Register Sim Card to Network\r\n");
                ret = ERR_SIM_CARD_REGISTRATION_FAILED;
                gCellularDriver.errorCode = ERR_SIM_CARD_REGISTRATION_FAILED;
            }
            else
            {
                //        printf("APn is Set\r\n");
                // Get time from cellular tower
                ret = CellularDeviceWrite(ATC_CCLK_Q);
                //        printf("Received Time from Cell Tower\r\n"); 
                ret = CellularDeviceWrite(ATC_COPS_Q);
                // Attach GPRS
                ret = CellularDeviceWrite(ATC_CGATT);
                for(loopCounter = 0; loopCounter < 5; loopCounter++)
                {
                    // Open PDP contexct
                    ret = CellularDeviceWrite(ATC_CGACT);
                    if(ret >= 0)
                    {
                        //            printf("Sim card is Registered on Network Operator\r\n");
                        break;
                    }
                }
                if(ret >= 0)
                {
                    // Get Network IP Address
                    ret = CellularDeviceWrite(ATC_CGPADDR);
                    //          printf("PDP context is activated\r\n");
                }
            }
        }
        else
        {
            //      printf("Unable to detect Sim Card\r\n");
            ret = ERR_SIM_CARD_NOT_FOUND;
            gCellularDriver.errorCode = ERR_SIM_CARD_NOT_FOUND;
        }
    }
    return ret;
}


//------------------------------------------------------------------------------
//  int32_t ConfigureCertificate(void)
//
//   Author:  Dilawar Ali
//   Date:    2018/05/18
//
//!  This function server configure certificates for SSL communication
//
//------------------------------------------------------------------------------
static int32_t ConfigureCertificate(void)
{
    int32_t ret = 0;
    // Write certificate to cellular module NVM
    ret = CellularDeviceWrite(ATC_USECMNG);
    if(ret >= 0)
    {
        ret = CellularDeviceWrite(ATC_CERTWRITE);
        if(ret >= 0)
        {
            // Write configuration for certificate e.g encryption type, root or client certificate etc
            ret = CellularDeviceWrite(ATC_USECPRF_1);
            ret = CellularDeviceWrite(ATC_USECPRF_2);
            ret = CellularDeviceWrite(ATC_USECPRF_3);
            ret = CellularDeviceWrite(ATC_USECPRF_4);
        }
    }
    else
    {
        // Error writing Certificate
        ret = ERR_CERTIFICATE_INVALID;
        gCellularDriver.errorCode = ERR_CERTIFICATE_INVALID;
    }
    return ret;
}

//------------------------------------------------------------------------------
//  static int32_t PostDataToiNet(void)
//
//   Author:  Dilawar Ali
//   Date:    2018/06/01
//
//!  This function Get the event from the Queue and send the corresponding event
//!  data to iNet cloud server
//
//------------------------------------------------------------------------------
static int32_t PostDataToiNet(void)
{
    PTR_COMM_EVT_t    commEvent = NULL;
    OS_MSG_SIZE       eventMsgSize = 0;
    RTOS_ERR          err;
    void              *p_msg;
    CPU_TS            ts;
    int32_t           ret = 0;
    uint32_t          size = 0;
    uint8_t           failCounter = 0;
    // Totatl number of events in QUEUE
    uint32_t numberOfEvents = eventMessagesQueue.MsgQ.NbrEntries;
    static uint32_t eventSent = 0;
    if(numberOfEvents > 0)
    {
        gCellularDriver.cellularState = CELLULAR_READY;
        
        // Disable the Power saving
        CellularDeviceWrite(ATC_CFUN_1);
        WaitForCellularGetReady();
        
        if(ret >= 0)
        {
            // Send All events in the Queue
            while( (gCellularDriver.cellularState == CELLULAR_READY) &&
                  ((p_msg = OSQPend(&eventMessagesQueue,100, OS_OPT_PEND_BLOCKING, &eventMsgSize, &ts, &err)) != NULL) )
            {
                commEvent = (PTR_COMM_EVT_t) p_msg;
                cellHttpsReceiving.isEventSent = false;
                // Try again if event is failed to upload or token expires
                while((cellHttpsReceiving.isEventSent == false) && (ret >= 0))
                {
                    
                    //Configure Cellular TCP Socket
                    ret = CellularDeviceWrite(ATC_USOCR);
                    ret = CreateUARTTXdata(ATC_UDCONF, cellDataBuffer, CELLULAR_DATA_BUFFER_SIZE);
                    ret = CellularDeviceWrite(ATC_UDCONF);
                    ret = CreateUARTTXdata(ATC_USOSEC, cellDataBuffer, CELLULAR_DATA_BUFFER_SIZE);
                    ret = CellularDeviceWrite(ATC_USOSEC);
                    
                    ret = CellularDeviceWrite(ATC_USOCLCFG);
                    
                    if(ret >= 0)
                    {
                        // Open TCP Socket
                        ret = CreateUARTTXdata(ATC_USOCO, cellDataBuffer, CELLULAR_DATA_BUFFER_SIZE);
                        ret = CellularDeviceWrite(ATC_USOCO);
                        if(ret >= 0)
                        {
                            // Open Direct Link TCP Socket
                            ret = CreateUARTTXdata(AT_USODL, cellDataBuffer, CELLULAR_DATA_BUFFER_SIZE);
                            ret = CellularDeviceWrite(AT_USODL);
                            if(ret >= 0)
                            {
                                // Change Cellular State  From Ready to Busy
                                gCellularDriver.cellularState = CELLULAR_BUSY;
                                
                                // If No Valid Token is available Get Token
                                if(cellHttpsReceiving.isTokenValid == false)
                                {
                                    // Create JSON data
                                    size = jsonCreatorAndParser[GET_INET_TOKEN].jCreator(httpUrlBuffer, CELLULAR_URL_BUFFER_SIZE, cellDataBuffer, CELLULAR_DATA_BUFFER_SIZE, commEvent);
                                    // Create HTTP Header
                                    size = CreateHttpHeader(GET_INET_TOKEN, cellHeaderBuffer, CELLULAR_HEADER_BUFFER_SIZE, (size-1));
                                }
                                else
                                {
                                    // Create JSON data
                                    size = jsonCreatorAndParser[commEvent->commEvtType].jCreator(httpUrlBuffer, CELLULAR_URL_BUFFER_SIZE, cellDataBuffer, CELLULAR_DATA_BUFFER_SIZE, commEvent);
                                    // Create HTTP Header
                                    size = CreateHttpHeader(commEvent->commEvtType, cellHeaderBuffer, CELLULAR_HEADER_BUFFER_SIZE, (size-1));
                                }
                                
                                
                                if(size > 0)
                                {
                                    // Write HTTP Header to TCP socket
                                    ret = CellularDeviceWrite(ATC_WRITEHEADER);
                                    if(ret >= 0)
                                    {
                                        // Write HTTP Body and Read Response from server
                                        ret = CellularDeviceWrite(ATC_USOWR);
                                        if(ret >= 0)
                                        {
                                            eventSent++;
                                            size = strlen((char const*)cellHttpsReceiving.jsonHeader);
                                            if(cellHttpsReceiving.isTokenValid == false)
                                            {
                                                ret = jsonCreatorAndParser[GET_INET_TOKEN].jParser(cellHttpsReceiving.jsonHeader, size, tokenBuffer, MAX_JSON_TOKEN_STRING_SIZE ,commEvent);
                                                if(ret >= 0)
                                                {
                                                    cellHttpsReceiving.isTokenValid = true;
                                                }
                                                else
                                                {
                                                    cellHttpsReceiving.isTokenValid = true;
                                                }
                                                cellHttpsReceiving.isEventSent = false;
                                            }
                                            else
                                            {
                                                ret = jsonCreatorAndParser[commEvent->commEvtType].jParser(cellHttpsReceiving.jsonHeader, size, NULL, NULL,commEvent);
                                            }
                                        }
                                        else
                                        {
                                            if(failCounter < 3)
                                            {
                                                failCounter++;
                                                ret = 0;
                                            }
                                            else
                                            {
                                                failCounter = 0;
                                                ret = ERR_SERVER_RESPONSE_PARSING_ERROR;
                                            }
                                            
                                        }
                                    }
                                    else
                                    {
                                        ret = ERR_TCP_SOCKET_WRITE_FAILED;
                                        gCellularDriver.errorCode = ERR_TCP_SOCKET_WRITE_FAILED;
                                    }
                                }
                                else
                                {
                                    ret = ERR_JSON_CREATE_FAILED;
                                    gCellularDriver.errorCode = ERR_JSON_CREATE_FAILED;
                                }
                            }
                            
                            else
                            {
                                ret = ERR_UNABLE_TO_OPEN_DIRECT_LINK;
                                gCellularDriver.errorCode = ERR_UNABLE_TO_OPEN_DIRECT_LINK;
                            }
                        }
                        else
                        {
                            ret = ERR_UNABLE_TO_OPEN_TCP_SOCK;
                            gCellularDriver.errorCode = ERR_UNABLE_TO_OPEN_TCP_SOCK;
                        }
                    }
                    else
                    {
                        ret = ERR_TCP_SOCKET_ERROR;
                        gCellularDriver.errorCode = ERR_TCP_SOCKET_ERROR;
                    }
                    if(ret >= 0)
                    {
                        CellularDeviceWrite(ATC_USODL_CLOSE);
                        //                    OSTimeDly(5000, OS_OPT_TIME_DLY, &err);
                        CreateUARTTXdata(ATC_USORD, cellDataBuffer, CELLULAR_DATA_BUFFER_SIZE);
                        CellularDeviceWrite(ATC_USORD);
                        
                        //                        CreateUARTTXdata(ATC_USOCL, cellDataBuffer, CELLULAR_DATA_BUFFER_SIZE);
                        //                        CellularDeviceWrite(ATC_USOCL);
                    }
                    
                }
                // Send Data to cloud and receive corresponding response
                if(ret >= 0)
                {
                    //return the event memory back to memory pool
                    ReturnEventMessageToPool(commEvent, true);
                    //                    OSTimeDly(5000, OS_OPT_TIME_DLY, &err);
                    
                }
                else
                {
                    ReturnEventMessageToPool(commEvent, false);
                    break;
                }
            }
            
        }
    }
    return ret;
}


//------------------------------------------------------------------------------
//   static int32_t PerformCellularRecovery(int32_t errorCode)
//
//   Author:  Dilawar Ali
//   Date:    2018/06/01
//
//!  This function Will perform cellular recovery sequence depending upon the error code
//
//------------------------------------------------------------------------------
static int32_t PerformCellularRecovery(int32_t errorCode)
{
    int32_t ret = 0;
    
    switch(errorCode)
    {
    case ERR_UNKNOWN_ERROR:
    case ERR_MODULE_NOT_RESPONDING:
    case ERR_UART_RX_TIMEOUT:
    case ERR_INVALID_SIGNAL_STRENGTH:
    case ERR_UART_NOT_OPEN:
    case ERR_SIM_CARD_NOT_FOUND:
    case ERR_SIM_CARD_REGISTRATION_FAILED:
    case ERR_CERTIFICATE_INVALID:
        
    case ERR_TCP_SOCKET_ERROR:
    case ERR_UNABLE_TO_OPEN_TCP_SOCK:
    case ERR_UNABLE_TO_OPEN_DIRECT_LINK:
        // Re-init the module
        CellularInit();
        break;
        
    case ERR_TCP_SOCKET_WRITE_FAILED:
    case ERR_JSON_CREATE_FAILED:
        
    case ERR_AT_CMD_PARSER_FUCN_UNDEFINED:
    case ERR_CELLULAR_CME_ERROR:
    case ERR_UART_RX_DATA_NULL:
    case ERR_UART_TX_DATA_NULL:
    case ERR_INCOMPLETE_DATA_RECEIVED:
    case ERR_SERVER_RESPONSE_PARSING_ERROR:
        gCellularDriver.cellularState = CELLULAR_READY;
        break;
        
    default:
        
        gCellularDriver.cellularState = CELLULAR_READY;
        break;
    }
    return ret;
}

//==============================================================================
//  GPS functions
//
//==============================================================================
static int32_t GPSConfigure(void)
{
    int32_t status = 0;
    uint32_t loopCounter = 0;
    RTOS_ERR  err;
    uint32_t messageSize = 0;
    // enble GNSS Power
    for(loopCounter = 0; loopCounter < 5; loopCounter++)
    {
        status = CellularDeviceWrite(ATC_GNNS_SUPPLY_EN);
        if( status >= 0)
        {
            printf("GNSS power supply enabled\r\n");
            break;
        }
    }
    if(status<0)
    {
        printf("unable to power up GNSS\r\n");
    }
    
    if(status>=0)
    {
        for(loopCounter = 0; loopCounter < 5; loopCounter++)
        {
            status = CellularDeviceWrite(ATC_GNNS_DATA_READY);
            if( status >= 0)
            {
                printf("GNSS data ready configured\r\n");
                break;
            }
        }
        if(status<0)
        {
            printf("unable to configure data ready pin\r\n");
        }
    }
    
    // open i2c port  ATC_UI2CO for gps
    if(status>=0)
    {
        for(loopCounter = 0; loopCounter < 5; loopCounter++)
        {
            status = CellularDeviceWrite(ATC_UI2CO);
            if( status >= 0)
            {

                printf("GPS i2c configured\r\n");
                break;
            }
        }
        if(status<0)
        {
            printf("unable to configure gps i2c\r\n");
        }
        
    }
   
    
    //TURN on GPS
    status=0;
    if(status>=0)
    {
        {
//            if(CellularDeviceWrite(ATC_UGPRF) >= 0)
//            {
//                printf("GPS profile Set\r\n");
//            }
//            if(CellularDeviceWrite(ATC_UGIND) >= 0)
//            {
//                printf("Indication configured\r\n");
//            }
//
//            if(CellularDeviceWrite(ATC_ULOCGNSS) >= 0)
//            {
//            }
            status = CellularDeviceWrite(ATC_GNSS_ON);
            if( status >= 0)
            {
                printf("GPS on\r\n");
                isGPSinit = true;
//                CellularDeviceWrite(ATC_UGGGA);
//                CellularDeviceWrite(ATC_UGGSV_ENABLE);
//                OSTimeDly(5000, OS_OPT_TIME_DLY, &err);
                
                
                
//                uint32_t bufferSize = (sizeof(configureI2CPortCmd) / sizeof(uint8_t ));
//                messageSize = snprintf(cellDataBuffer, (CELLULAR_DATA_BUFFER_SIZE - messageSize), "AT+UGUBX=\"");
//                for(uint32_t i=0; i<bufferSize; i++)
//                {
//                    messageSize += snprintf(&cellDataBuffer[messageSize], (CELLULAR_DATA_BUFFER_SIZE - messageSize), "%.2x", configureI2CPortCmd[i]);
//                }
//                messageSize += snprintf(&cellDataBuffer[messageSize], (CELLULAR_DATA_BUFFER_SIZE - messageSize), "\"\r\n");
//                
//                if(CellularDeviceWrite(ATC_UI2CW) >= 0)
//                {
//                    printf("Ready pin configured\r\n");
//                }
//               
//                
//                bufferSize = (sizeof(CFG_MSG) / sizeof(uint8_t ));
//                messageSize = snprintf(cellDataBuffer, (CELLULAR_DATA_BUFFER_SIZE - messageSize), "AT+UGUBX=\"");
//                for(uint32_t i=0; i<bufferSize; i++)
//                {
//                    messageSize += snprintf(&cellDataBuffer[messageSize], (CELLULAR_DATA_BUFFER_SIZE - messageSize), "%.2x", CFG_MSG[i]);
//                }
//                messageSize += snprintf(&cellDataBuffer[messageSize], (CELLULAR_DATA_BUFFER_SIZE - messageSize), "\"\r\n");
//                
//                CellularDeviceWrite(ATC_UI2CW);
//                
//                
//                bufferSize = (sizeof(CFG_MSG_GET) / sizeof(uint8_t ));
//                messageSize = snprintf(cellDataBuffer, (CELLULAR_DATA_BUFFER_SIZE - messageSize), "AT+UGUBX=\"");
//                for(uint32_t i=0; i<bufferSize; i++)
//                {
//                    messageSize += snprintf(&cellDataBuffer[messageSize], (CELLULAR_DATA_BUFFER_SIZE - messageSize), "%.2x", CFG_MSG_GET[i]);
//                }
//                messageSize += snprintf(&cellDataBuffer[messageSize], (CELLULAR_DATA_BUFFER_SIZE - messageSize), "\"\r\n");
//                
//                CellularDeviceWrite(ATC_UI2CW);
                                //                CellularDeviceWrite(ATC_ULOC);
                printf("GPS ON\r\n");
                gps_initialized=1;
            }
        }
        if(status<0)
        {
            printf("unable turn ON GPS\r\n");
            gps_initialized=0;
        }
        
    }
    OSTimeDly(2000, OS_OPT_TIME_DLY, &err); // delay 
    return status;  
}

//------------------------------------------------------------------------------
//  int32_t CellularInit(void)
//
//   Author:  Dilawar Ali
//   Date:    2018/05/17
//
//!  This function Initialize Cellular UART
//
//------------------------------------------------------------------------------

static int32_t CellularInit(void)
{
    RTOS_ERR        err;
    int32_t ret = 0;
    ConfigureCellularPins();
    ClearWatchDogCounter();
    CellularPowerUp();
    ClearWatchDogCounter();
    CellularModuleReset();
    ClearWatchDogCounter();
    
    //  EnableCellularModule();
    CellularUARTOpen();
    OSTimeDly(4000, OS_OPT_TIME_DLY, &err);
    ClearWatchDogCounter();
    gCellularDriver.cellularState = CELLULAR_IDLE;
    gCellularDriver.cellUART = cellUART;
    
    ret = WarmupCellularModule();
    ClearWatchDogCounter();
    //    printf("Cellular Warmup status: %d\r\n", ret);
    
    if(ret < 0)
    {
        gCellularDriver.cellularState = CELLULAR_ERROR;
    }
    else
    {
        ret = ConfigureCertificate();
        if(ret >= 0)
        {
            gCellularDriver.cellularState = CELLULAR_READY;
        }
    }
    return ret;
}

//------------------------------------------------------------------------------
//  static void WaitForCellularGetReady(void)
//
//   Author:  Dilawar Ali
//   Date:    2018/10/24
//
//!  This function wait for cellular to get register to network
//
//------------------------------------------------------------------------------
static void WaitForCellularGetReady(void)
{
    while(CellularDeviceWrite(ATC_CREG_Q) < 0);
}

//==============================================================================
//  GLOBAL FUNCTIONS IMPLEMENTATION
//==============================================================================

//------------------------------------------------------------------------------
//  void CellularTask(void *arg)
//
//   Author:  Dilawar Ali
//   Date:    2018/05/18
//
//!  This function Handles Cellular Task and its all functionalities
//
//------------------------------------------------------------------------------
void CellularTask(void *arg)
{
    RTOS_ERR        err;
    void         *p_msg;
    OS_MSG_SIZE   msg_size;
    CPU_TS        ts;
    CellMsg_t *msg;
    
    
//    uint32_t bufferSize = 0,messageSize = 0;
    
    while(1)
    {
        isCellularReadyToSleep = true;
        p_msg =  OSTaskQPend(WAIT_FOREVER, OS_OPT_PEND_BLOCKING, &msg_size, &ts, &err);
        msg = (CellMsg_t*)p_msg;
        isCellularReadyToSleep = false;
        
        switch(msg->msgId)
        {
        case CELL_INIT:
            ClearWatchDogCounter();
            CellularInit();
            ClearWatchDogCounter();
            break;
            
        case CELL_GPS_CONFIGURE:
//            GPSConfigure();
            ClearWatchDogCounter();
            break;
            
        case CELL_GPS_OFF:
//            CellularDeviceWrite(ATC_GNSS_OFF);
            ClearWatchDogCounter();
            
            
//            bufferSize = (sizeof(resetGNSS) / sizeof(uint8_t ));
//            messageSize = snprintf(cellDataBuffer, (CELLULAR_DATA_BUFFER_SIZE - messageSize), "AT+UGUBX=\"");
//            for(uint32_t i=0; i<bufferSize; i++)
//            {
//                messageSize += snprintf(&cellDataBuffer[messageSize], (CELLULAR_DATA_BUFFER_SIZE - messageSize), "%.2x", resetGNSS[i]);
//            }
//            messageSize += snprintf(&cellDataBuffer[messageSize], (CELLULAR_DATA_BUFFER_SIZE - messageSize), "\"\r\n");
//            
//            CellularDeviceWrite(ATC_UI2CW);
            
//            printf("GPS Off\r\n");
            isGPSinit = false;
            break;
            
        case CELL_GET_GPS_COORDINATES: 
//            CellularGetGPSData();
            ClearWatchDogCounter();
            break;
            
        case CELL_CONNECTION_TEST:
            break;
            
        case CELL_SEND_EVENT_TO_INET:
            //WakeUpCellularModuleFromPowerSaving();
            if(PostDataToiNet() < 0)
            {
                gCellularDriver.cellularState = CELLULAR_ERROR;
                PerformCellularRecovery(gCellularDriver.errorCode);
            }
            else
            {
                gCellularDriver.cellularState = CELLULAR_READY;
            }
            // Enable the Power Saving Mode
            CellularDeviceWrite(ATC_CFUN_0);
            // SetCellularToPowerSavingMode();
            break;
            
        case CELL_SEND_SMS:
            break;
            
        default:
            break;
        }
        
        ReturnTaskMessageToPool((SysMsg_t*)msg);
    }
}