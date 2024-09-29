//==============================================================================
//
//  SysTask.c
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
//  Source:        SysTask.c
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

#include <stdio.h>
#include "main.h"
#include "SPI_Comm.h"
#include "SysTask.h"
#include "Timer.h"
#include "Cellular.h"
#include "FileCommit.h"

//==============================================================================
//  CONSTANTS, TYPEDEFS AND MACROS 
//==============================================================================
#define GPS_DATA_LENGTH           256u
#define MAILBOX_PERIODIC_INTERVAL 60u
#define GPS_PERIODIC_INTERVAL     15u
//==============================================================================
//  LOCAL DATA STRUCTURE DEFINITION
//==============================================================================


//==============================================================================
//  GLOBAL DATA DECLARATIONS
//==============================================================================
Firmware_File_t fwFile;
FILE_DESCRIPTOR_STRUCT firmwareFileDescriptor;
Device_Parameters_t deviceParams;
//==============================================================================
//  LOCAL DATA DECLARATIONS
//==============================================================================

//==============================================================================
//  LOCAL FUNCTION PROTOTYPES
//==============================================================================
inline static void SendPeriodicGPSMessage(void);
inline static void SendPriodicCommMessage(void);

//==============================================================================
//  LOCAL FUNCTIONS IMPLEMENTATION
//==============================================================================

//------------------------------------------------------------------------------
//  static void SendPriodicCommMessage(void)
//
//   Author:  Dilawar Ali
//   Date:    2018/08/18
//
//!  This function Periodically Send Comm event to comm task if there are events
//!  are events need to send in Queue
//
//------------------------------------------------------------------------------
static void SendPriodicCommMessage(void)
{
    static uint16_t mailBoxPeriodicTimeout = MAILBOX_PERIODIC_INTERVAL;
    CellMsg_t *msg = NULL;
    RTOS_ERR  err;
    if(mailBoxPeriodicTimeout == 0)
    {
        mailBoxPeriodicTimeout = 15;
        if(eventMessagesQueue.MsgQ.NbrEntries > 0)
        {
            msg = (CellMsg_t*)GetTaskMessageFromPool();
            if(msg != NULL)
            {
                msg->msgId = CELL_SEND_EVENT_TO_INET;
                msg->msgInfo = 1;
                msg->ptrData = NULL;
                OSTaskQPost(&CellTaskTCB, (void *)msg, sizeof(SysMsg_t), OS_OPT_POST_FIFO, &err);
            }
        }
    }
    else
    {
        mailBoxPeriodicTimeout--;
    }
}

//------------------------------------------------------------------------------
//  static void SendPeriodicGPSMessage(void)
//
//   Author:  Dilawar Ali
//   Date:    2018/08/18
//
//!  This function Periodically Send GPS Message to task to get GPS data from GNSS
//
//------------------------------------------------------------------------------
static void SendPeriodicGPSMessage(void)
{
    static uint16_t GPSPeriodicTimeout = GPS_PERIODIC_INTERVAL;
    static uint8_t counter = 0;
    CellMsg_t *msg = NULL;
    RTOS_ERR  err;
    
    if(GPSPeriodicTimeout == 0)
    {
        
        GPSPeriodicTimeout = GPS_PERIODIC_INTERVAL;
        msg = (CellMsg_t*)GetTaskMessageFromPool();
        if(counter < 12)
        {
            counter++;
        if(msg != NULL)
        {
            msg->msgId = CELL_GET_GPS_COORDINATES;
            msg->msgInfo = 1;
            msg->ptrData = NULL;
            OSTaskQPost(&CellTaskTCB, (void *)msg, sizeof(SysMsg_t), OS_OPT_POST_FIFO, &err);
        }
    }
    else
    {
            counter = 0;
            msg->msgId = CELL_GPS_OFF;
            msg->msgInfo = 1;
            msg->ptrData = NULL;
            OSTaskQPost(&CellTaskTCB, (void *)msg, sizeof(SysMsg_t), OS_OPT_POST_FIFO, &err);
        }
    }
    else
    {
        GPSPeriodicTimeout--;
    }
}

int32_t DataFlashTestCode(void)
{
    int32_t ret = 0, i = 0;
//    uint8_t page = 6;
//    uint8_t DataFlashTxData[DATAFLASH_BYTES_PER_PAGE] = {'B'};
//    for(int32_t i = 0; i < DATAFLASH_BYTES_PER_PAGE; i++)
//    {
//        DataFlashTxData[i] = 'B';
//    }
//    uint8_t DataFlashRxData[DATAFLASH_BYTES_PER_PAGE] = {0};
    ret = DataFlashInit();
    DataFlashEnablePowerSaving();
    
    for(i=0; i<50; i++)
    {
        __asm("nop");
    }
    
    DataFlashDisablePowerSaving();
    
//    ret = DataFlashErasePage(page);
//    
//    ret = DataFlashWriteBuffer(0, DataFlashTxData, 128);
//    ret = DataFlashWriteBufferToPage(page);
//    ret = DataFlashReadPage(page, DataFlashRxData, DATAFLASH_BYTES_PER_PAGE);
    ret = GetDeviceParamsFromFlash(&deviceParams);
    if(ret < 0)
    {
        deviceParams.numberOfParameters = 6;
        sprintf((char *)deviceParams.simApn, "11583.mcs");
        
        ret = SaveCurrentDeviceParameters(&deviceParams);
    }
    
    sprintf((char *)InstrumentInfo.SerialNumber, "%s", deviceParams.instrumentSerialNumber);
    sprintf((char *)InstrumentInfo.ManufacturingDate, "%s", deviceParams.mfgDate);
    sprintf((char *)InstrumentInfo.JobNumber, "%s", deviceParams.jobNumber);
    sprintf((char *)InstrumentInfo.PartNumber, "%s", deviceParams.partNumber);
    sprintf((char *)gCellularDriver.APN, "%s", deviceParams.simApn);
    sprintf((char *)InstrumentInfo.TechniciansInitials, "%s", deviceParams.techInitials);
    
    return ret;
}
//==============================================================================
//  GLOBAL FUNCTIONS IMPLEMENTATION
//==============================================================================

//------------------------------------------------------------------------------
//  void SysTask(void *arg)
//
//   Author:  Dilawar Ali
//   Date:    2018/08/18
//
//!  This function Handle All system asigned task
//
//------------------------------------------------------------------------------
void SysTask(void *arg)
{
    RTOS_ERR        err;
    void         *p_msg;
    OS_MSG_SIZE   msg_size;
    CPU_TS        ts;
    
    SysMsg_t *msg;

    
    QueuesInit();
    // Post initialize message to cellular
    msg = GetTaskMessageFromPool();
    ((CellMsg_t*)msg)->msgId = CELL_INIT;
    OSTaskQPost(&CellTaskTCB, (void *)msg, sizeof(SysMsg_t), OS_OPT_POST_FIFO, &err);
    
    msg = GetTaskMessageFromPool();
    ((CellMsg_t*)msg)->msgId = CELL_GPS_CONFIGURE;
    OSTaskQPost(&CellTaskTCB, (void *)msg, sizeof(SysMsg_t), OS_OPT_POST_FIFO, &err);
    
    (void)SPISlaveConfigure();
    DataFlashTestCode();
   
    while(1)
    {
        ClearWatchDogCounter();
        p_msg =  OSTaskQPend(WAIT_FOREVER, OS_OPT_PEND_BLOCKING, &msg_size, &ts, &err);
        msg = (SysMsg_t*)p_msg;
        
        switch(msg->msgId)
        {
            
        case EVENT_MANAGEMENT_EVENT_RECEVIED:
            SendPriodicCommMessage();
            SendPeriodicGPSMessage();
            break;
            
        case POWER_MANAGEMENT_EVENT_RECEVIED:
            break;
            
        case WRITE_DATA_TO_FLASH:
            fwFile.remainingDataPackets--;
            FileCommitParseReceivedFwFilePacket(&firmwareFileDescriptor, &fwFile);

            break;
            
        case DEVICE_SHUTDOWN_MSG:
            DataFlashDisablePowerSaving();
            
            deviceParams.numberOfParameters = 6;
            sprintf((char *)deviceParams.instrumentSerialNumber, "%s", InstrumentInfo.SerialNumber);
            sprintf((char *)deviceParams.mfgDate, "%s", InstrumentInfo.ManufacturingDate);
            sprintf((char *)deviceParams.jobNumber, "%s", InstrumentInfo.JobNumber);
            sprintf((char *)deviceParams.partNumber,"%s", InstrumentInfo.PartNumber);
            sprintf((char *)deviceParams.simApn, "%s", gCellularDriver.APN);
            sprintf((char *)deviceParams.techInitials, "%s", InstrumentInfo.TechniciansInitials);
            SaveCurrentDeviceParameters(&deviceParams);
            break;
            
        default:
            break;
        }
        
        ReturnTaskMessageToPool(msg);
    }
}

//==============================================================================
//  End Of File
//==============================================================================