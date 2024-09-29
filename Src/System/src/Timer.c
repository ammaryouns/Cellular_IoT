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
#include "Timer.h"
#include <stddef.h>
#include <time.h>
#include <stdlib.h>

#include "Cellular.h"
#include "Main.h"
//==============================================================================
//  CONSTANTS, TYPEDEFS AND MACROS 
//==============================================================================
#define MAY2018_29Time      1527590650

#define TIME_ARRAY_ELEMENTS             3u            //!< Time string array elements for integer conversion
#define YEAR_INCREMENT                  100u          //!< Offset of 100 to save year
#define MONTH_VALUE                     1u            //!< First element of intTimeVal array to save month
#define DAY_VALUE                       2u            //!< Second element of intTimeVal array to save month
#define HOUR_VALUE                      3u            //!< Third element of intTimeVal array to save month
#define MINUTE_VALUE                    4u            //!< Fourth element of intTimeVal array to save month
#define SECONDS_VALUE                   5u            //!< Fifth element of intTimeVal array to save month

#define SYS_TASK_SCHEDULE_TIMEOUT       5u            // 5 Seconds Interval
//==============================================================================
//  LOCAL DATA DECLARATIONS
//==============================================================================

static RTCDRV_TimerID_t RTCTimerId;
//==============================================================================
//  LOCAL FUNCTION PROTOTYPES
//==============================================================================
static void PeriodicFunctionCall( RTCDRV_TimerID_t id, void * user );
//==============================================================================
//  GLOBAL DATA DECLARATIONS
//==============================================================================

//==============================================================================
//  LOCAL FUNCTIONS IMPLEMENTATION
//==============================================================================

//------------------------------------------------------------------------------
//  static void PeriodicFunctionCall( RTCDRV_TimerID_t id, void * user )
//
//   Author:  Dilawar Ali
//   Date:    2018/05/29
//
//!  This function Periodically every time when RTC timer timeout
//
//------------------------------------------------------------------------------
static void PeriodicFunctionCall( RTCDRV_TimerID_t id, void * user )
{
    static uint8_t sysTaskScheduleTimeOut = SYS_TASK_SCHEDULE_TIMEOUT;
    SysMsg_t *msg = NULL;
    RTOS_ERR  err;
    // This RTC Block is being used to handle UART data communication of Cellular
    uint8_t *rxBuffer = gCellularDriver.UARTRxBuffer;
    // If cellular uart is reading data
    if(cellHttpsReceiving.isUARTReadStarted == true)
    {
        // When 200 ms time is over
        if(++cellHttpsReceiving.UARTWaitCounter >= 2u)
        {
            // clear the counter
            cellHttpsReceiving.UARTWaitCounter = 0;
            // Get UART receive Status
            UARTDRV_GetReceiveStatus(gCellularDriver.cellUART, &rxBuffer, &cellHttpsReceiving.readyBytes, &cellHttpsReceiving.remainingBytes);
            UARTDRV_Abort(gCellularDriver.cellUART, uartdrvAbortReceive);
        }
    }
    
    if(sysTaskScheduleTimeOut == 0)
    {
        sysTaskScheduleTimeOut = SYS_TASK_SCHEDULE_TIMEOUT;
        msg = (SysMsg_t*)GetTaskMessageFromPool();
        if(msg != NULL)
        {
            msg->msgId = EVENT_MANAGEMENT_EVENT_RECEVIED;
            msg->msgInfo = 1;
            msg->ptrData = NULL;
            OSTaskQPost(&SYSTaskTCB, (void *)msg, sizeof(SysMsg_t), OS_OPT_POST_FIFO, &err);
        }
    }
    else
    {
        sysTaskScheduleTimeOut--;
    }
    
}

//==============================================================================
//  GLOBAL FUNCTIONS IMPLEMENTATION
//==============================================================================


//------------------------------------------------------------------------------
//  void RTCInit(void)
//
//   Author:  Dilawar Ali
//   Date:    2018/05/29
//
//!  This function initialize the system RTC
//
//------------------------------------------------------------------------------
void RTCInit(void)
{
    // Initialization of RTCDRV driver
    RTCDRV_Init();
    // Reserve a timer
    RTCDRV_AllocateTimer( &RTCTimerId );
    //Set RTC Clock Initial seconds
    RTCDRV_SetWallClock(MAY2018_29Time);
    // Start a oneshot timer with 100 millisecond timeout
    RTCDRV_StartTimer( RTCTimerId, rtcdrvTimerTypePeriodic, 100, PeriodicFunctionCall, NULL );
}

//------------------------------------------------------------------------------
//  void UpdateRTCTimeFromSNTP(uint32_t timeStamp)
//
//   Author:  Dilawar Ali
//   Date:    2018/04/23
//
//!  This function Update RTC Clock time from SNTP server
//
//------------------------------------------------------------------------------
void UpdateRTCTimeFromSNTP(uint32_t timeStamp)
{
    timeStamp -= SECOND_SINCE_1970;
    RTCDRV_SetWallClock(timeStamp);
}

//------------------------------------------------------------------------------
//   uint32_t GetRTCTime(void)
//
//   Author:  Dilawar Ali
//   Date:    2018/04/29
//
//!  This function Get RTC Clock time
//
//------------------------------------------------------------------------------
uint32_t GetRTCTime(void)
{
    return RTCDRV_GetWallClock();
}

//------------------------------------------------------------------------------
//   uint32_t GetRTCTicks(void)
//
//   Author:  Dilawar Ali
//   Date:    2018/06/08
//
//!  This function Get RTC Clock ticks
//
//------------------------------------------------------------------------------
uint32_t GetRTCTicks(void)
{
    return RTCDRV_GetWallClockTicks32();
}

//------------------------------------------------------------------------------
//   void UpdateRTCTime(uint8_t time[])
//
//   Author:   Muhammad Shuaib, Ported from MORRISON
//   Date:     2017/05/03
//
//!  This function Update the RTC Time from Cellular
//
//------------------------------------------------------------------------------
void UpdateRTCTime(uint8_t cellularTime[])
{  
    // Array to store the value of time string for integer conversion
    uint8_t timeArray[TIME_ARRAY_ELEMENTS];
    // Time array address offset variable
    uint8_t arrayOffsetVal = 0u;
    // Variable to get int32_t value of time
    int32_t intTimeVal[9];
    // Variable to store local time for Unix timestamp conversion
    struct tm localTimeValue;
    // Variable to get the value of local time converted to Unix timestamp
    time_t unixTimeStamp;
    // Loop counter variable
    uint8_t loopCounter = 0u;
    
    // Get highest priority interface running info
    
    // Update time via Cellular only if it is not updated already
    // Initialize time int32_t value array
    memset(intTimeVal,0,7u);
    // Fetch time from cellularTime array
    for(loopCounter = 0u; loopCounter <= 6; loopCounter++)
    {
        // Reset the time string array
        memset(timeArray, 0, TIME_ARRAY_ELEMENTS);
        // Get the value
        strncpy((char*)(&(timeArray[0])),(char const*)&(cellularTime[arrayOffsetVal]),2u);
        // Convert the value to integer value and store in local variable
        intTimeVal[loopCounter] = (int32_t)(strtol((const char *)timeArray, NULL, 10));
        // Increment offset variable value
        arrayOffsetVal += TIME_ARRAY_ELEMENTS;
    }
    // Save year value
    localTimeValue.tm_year = intTimeVal[0u] + (int32_t)YEAR_INCREMENT;
    // Save month value,decremented by one (Jan = 0 and Dec = 11)
    localTimeValue.tm_mon = (intTimeVal[MONTH_VALUE]- 1);
    // Save day value
    localTimeValue.tm_mday = intTimeVal[DAY_VALUE];
    // Save hour value
    localTimeValue.tm_hour = intTimeVal[HOUR_VALUE];
    // Save minutes value
    localTimeValue.tm_min = intTimeVal[MINUTE_VALUE];
    // Save seconds value
    localTimeValue.tm_sec = intTimeVal[SECONDS_VALUE];
    localTimeValue.tm_isdst = -1; // Is DST on? 1 = yes, 0 = no, -1 = unknown
    // Convert the value to Unix timestamp
    unixTimeStamp = mktime(&localTimeValue);
    // Save Unix timestamp value into hibernate module
    RTCDRV_SetWallClock(unixTimeStamp);
}
