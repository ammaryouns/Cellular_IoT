//==============================================================================
//
//  Cellular.h
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
//  Source:        CellularATCommands.h
//
//  Project:       Frey
//
//  Author:        Dilawar Ali
//
//  Date:          2018/05/28
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

#ifndef TIMER_H
#define TIMER_H
//==============================================================================
//  INCLUDES
//==============================================================================
#include "ExtCommunication.h"
#include <rtcdriver.h>

//==============================================================================
//  GLOBAL CONSTANTS, TYPEDEFS AND MACROS 
//==============================================================================

#define TICKS_PER_SECONDS               1000u

typedef void(*PtrPeriodicFunc)(void);


//------------------------------------------------------------------------------
//  void UpdateRTCTimeFromSNTP(uint32_t timeStamp)
//
//   Author:  Dilawar Ali
//   Date:    2018/04/23
//
//!  This function Update RTC Clock time from SNTP server
//
//------------------------------------------------------------------------------
void UpdateRTCTimeFromSNTP(uint32_t timeStamp);

//------------------------------------------------------------------------------
//   uint32_t GetRTCTime(void)
//
//   Author:  Dilawar Ali
//   Date:    2018/04/29
//
//!  This function Get RTC Clock time
//
//------------------------------------------------------------------------------
uint32_t GetRTCTime(void);


//------------------------------------------------------------------------------
//  void RTCInit(void)
//
//   Author:  Dilawar Ali
//   Date:    2018/05/29
//
//!  This function initialize the system RTC
//
//------------------------------------------------------------------------------
void RTCInit(void);

//------------------------------------------------------------------------------
//   uint32_t GetRTCTicks(void)
//
//   Author:  Dilawar Ali
//   Date:    2018/06/08
//
//!  This function Get RTC Clock ticks
//
//------------------------------------------------------------------------------
uint32_t GetRTCTicks(void);

//------------------------------------------------------------------------------
//   void UpdateRTCTime(uint8_t time[])
//
//   Author:   Muhammad Shuaib, Ported from MORRISON
//   Date:     2017/05/03
//
//!  This function Update the RTC Time from Cellular
//
//------------------------------------------------------------------------------
void UpdateRTCTime(uint8_t cellularTime[]);
#endif