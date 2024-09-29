// Handle Event Management in queue

//==============================================================================
//
//  Event.c
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
//  Source:        Event.c
//
//  Project:       Frey
//
//  Author:        Dilawar Ali
//
//  Date:          2018/05/22
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
#include "Event.h"
#include  <common/include/rtos_utils.h>
#include "main.h"
//==============================================================================
//  CONSTANTS, TYPEDEFS AND MACROS 
//==============================================================================

//==============================================================================
//  LOCAL DATA DECLARATIONS
//==============================================================================

//==============================================================================
//  LOCAL FUNCTION PROTOTYPES
//==============================================================================

//==============================================================================
//  GLOBAL DATA DECLARATIONS
//==============================================================================

GPSInfo_t GPSReceivedCoordinates;
InstInfo_t InstrumentInfo;

//==============================================================================
//  LOCAL FUNCTIONS IMPLEMENTATION
//==============================================================================


//==============================================================================
//  GLOBAL FUNCTIONS IMPLEMENTATION
//==============================================================================


//------------------------------------------------------------------------------
//  SysMsg_t* GetTaskMessageFromPool(void)
//
//   Author:  Dilawar Ali
//   Date:    2018/05/22
//
//!  This function Get the memory for TASK message from its memory pool
//
//------------------------------------------------------------------------------
SysMsg_t* GetTaskMessageFromPool(void)
{
    RTOS_ERR        err;
    void         *p_msg;
    OS_MSG_SIZE   msg_size;
    CPU_TS        ts;
    
    p_msg = OSQPend(&taskMessagesFreeQueue, 0, OS_OPT_PEND_NON_BLOCKING, &msg_size, &ts, &err);
    
    return (SysMsg_t*)p_msg;
}

//------------------------------------------------------------------------------
//   void ReturnTaskMessageToPool(SysMsg_t* msg)
//
//   Author:  Dilawar Ali
//   Date:    2018/05/22
//
//!  This function Returns the TASK message memory to its memory pool
//
//------------------------------------------------------------------------------
void ReturnTaskMessageToPool(SysMsg_t* msg)
{
    RTOS_ERR        err;
    
    OSQPost(&taskMessagesFreeQueue, msg, sizeof(SysMsg_t), OS_OPT_POST_FIFO + OS_OPT_POST_ALL + OS_OPT_POST_NO_SCHED, &err);
    APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
}


//------------------------------------------------------------------------------
//  ComEvent_t* GetEventMessageFromPool(void)
//
//   Author:  Dilawar Ali
//   Date:    2018/05/22
//
//!  This function Get the memory for Event message from its memory pool
//
//------------------------------------------------------------------------------
ComEvent_t* GetEventMessageFromPool(void)
{
    RTOS_ERR        err;
    void         *p_msg;
    OS_MSG_SIZE   msg_size;
    CPU_TS        ts;
    
    p_msg = OSQPend(&eventMessagesFreeQueue, 10, OS_OPT_PEND_NON_BLOCKING, &msg_size, &ts, &err);
    
    return (ComEvent_t*)p_msg;
}

//------------------------------------------------------------------------------
//  void ReturnEventMessageToPool(ComEvent_t* msg, BOOLEAN isEventSent)
//
//   Author:  Dilawar Ali
//   Date:    2018/05/22
//
//!  This function returns the Event message memory to its memory pool
//
//------------------------------------------------------------------------------
void ReturnEventMessageToPool(ComEvent_t* msg, BOOLEAN isEventSent)
{
    RTOS_ERR        err;
    if(isEventSent == true)
    {
        OSQPost(&eventMessagesFreeQueue, msg, sizeof(ComEvent_t), OS_OPT_POST_FIFO + OS_OPT_POST_ALL + OS_OPT_POST_NO_SCHED, &err);
    }
    else
    {
        OSQPost(&eventMessagesQueue, msg, sizeof(ComEvent_t), OS_OPT_POST_FIFO + OS_OPT_POST_ALL + OS_OPT_POST_NO_SCHED, &err);
    }
    APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
}


uint8_t GetNextSequence(void)
{
    return 0;
}
