//==============================================================================
//
//  main.h
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
//  Source:        main.h
//
//  Project:       Cornell
//
//  Author:        Dilawar Ali
//
//  Date:          2018/04/23
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
#ifndef MAIN_H
#define MAIN_H
//==============================================================================
//  INCLUDES
//==============================================================================
#include  <kernel/include/os.h>

//==============================================================================
//  GLOBAL CONSTANTS, TYPEDEFS AND MACROS 
//==============================================================================
#define WAIT_FOREVER    0

#define BOOLEAN bool

#define FIRMWARE_VERSION_MAJOR        1u                                        //!< Firmware version major (in v1.2.3 - 1 is major)
#define FIRMWARE_VERSION_MINOR        0u                                        //!< Firmware version minor (in v1.2.3 - 2 is minor)  
#define FIRMWARE_VERSION_BUILD        4u                                        //!< Firmware version build (in v1.2.3 - 3 is build)



//Meant for main.h
#define SERIAL_NUMBER_LENGTH                                    16u
#define DATE_LENGTH                                             04u
#define PART_NUMBER_LENGTH                                      16u
#define TECH_INITIALS_LENGTH                                    04u
#define JOB_NUMBER_LENGTH                                        8u
#define NETWORK_ID_LENGTH                                       20u


#define FIND_MIN(X,Y)   ( (X) < (Y) ? (X) : (Y) )
//==============================================================================
//  GLOBAL DATA
//==============================================================================

extern OS_Q   eventMessagesQueue, eventMessagesFreeQueue;
extern OS_Q   taskMessagesFreeQueue;

extern OS_TCB   CellTaskTCB;
extern OS_TCB   SYSTaskTCB;

extern BOOLEAN isSpiReadyToSleep;
extern BOOLEAN isCellularReadyToSleep;
//==============================================================================
//  EXTERNAL OR GLOBAL FUNCTIONS
//==============================================================================

//------------------------------------------------------------------------------
//  void QueuesInit(void)
//
//   Author:  Dilawar Ali
//   Date:    2018/05/22
//
//!  This function Initialize the Queues memory pool
//
//------------------------------------------------------------------------------
void QueuesInit(void);

//------------------------------------------------------------------------------
//  void ClearWatchDogCounter(void)
//
//   Author:  Dilawar Ali
//   Date:    2018/09/17
//
//!  This function Initialize the Queues memory pool
//
//------------------------------------------------------------------------------
void ClearWatchDogCounter(void);
#endif