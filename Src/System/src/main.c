//==============================================================================
//
//  main.c
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
//  Source:        main.c
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
#include "main.h"

#include <stdint.h>
#include <stdio.h>
#include "efm32tg11b120f128gm32.h"
#include  <cpu/include/cpu.h>
#include  <common/include/common.h>
#include  <common/include/lib_def.h>
#include  <common/include/rtos_utils.h>
#include  <common/include/rtos_err.h>
#include  <em_cmu.h>
#include  <em_chip.h>
#include  <em_wdog.h>
#include  <em_emu.h>

#include "Systask.h"
#include "Cellular.h"
#include "event.h"
#include "Timer.h"

//==============================================================================
//  CONSTANTS, TYPEDEFS AND MACROS 
//==============================================================================


#define  SYS_TASK_PRI              5u
#define  SYS_TASK_STACK_SIZE       512u

#define  CELL_TASK_PRI             15u
#define  CELL_TASK_STACK_SIZE      1024u

#define MAX_TASK_MESSAGES          30u
#define MAX_EVENS_MESSAGES         20u
//==============================================================================
//  LOCAL DATA DECLARATIONS
//==============================================================================

static  CPU_STK  SYSTaskStack[SYS_TASK_STACK_SIZE];
static  CPU_STK  CellTaskStack[CELL_TASK_STACK_SIZE];

static SysMsg_t   taskMessageQueuesData [MAX_TASK_MESSAGES];
static ComEvent_t evensMessageQueueData [MAX_EVENS_MESSAGES];

//==============================================================================
//  LOCAL FUNCTION PROTOTYPES
//==============================================================================
void QueuesInit(void);
static void IdleTaskHookFunc(void);
//==============================================================================
//  GLOBAL DATA DECLARATIONS
//==============================================================================
uint32_t maintmp = 0;

OS_Q   eventMessagesQueue, eventMessagesFreeQueue;
OS_Q   taskMessagesFreeQueue;

OS_TCB   CellTaskTCB;
OS_TCB   SYSTaskTCB;

BOOLEAN isSpiReadyToSleep = true;
BOOLEAN isCellularReadyToSleep = true;

uint32_t coreClkFreq = 0;

WDOG_Init_TypeDef  wDogInitParam= 
{                                                                            
    true,                       /* Start watchdog when init done */            
    false,                      /* WDOG counting during debug halt */      
    true,                       /* WDOG counting when in EM2 */            
    true,                       /* WDOG counting when in EM3 */            
    false,                      /* EM4 can be entered */                       
    true,                       /* block disabling LFRCO/LFXO in CMU */ 
    false,                      /* Do not lock WDOG configuration (if locked, reset needed to unlock) */             
    wdogClkSelULFRCO,           /* Select 1kHZ WDOG oscillator */         
    wdogPeriod_16k,             /* Set 16sec timeout period */ 
    wdogWarnDisable,            /* Disable warning interrupt */           
    wdogIllegalWindowDisable,   /* Disable illegal window interrupt */    
    false                       /* Do not disable reset */                
};

//==============================================================================
//  LOCAL FUNCTIONS IMPLEMENTATION
//==============================================================================
static void IdleTaskHookFunc(void)
{
    if((isSpiReadyToSleep == true) && (isCellularReadyToSleep == true))
    {
//        printf("\r\nSleep\r\n");
//        EMU_EnterEM3(true); // cannot enable the EM3 mode, LFRCO is required for RTC.
//        EMU_EnterEM2(true);
    }
}

//==============================================================================
//  GLOBAL FUNCTIONS IMPLEMENTATION
//==============================================================================

int main(void)
{
    RTOS_ERR  err;
    //--------------------------------------------------------------------------
    // CPU MANAGEMENT
    //--------------------------------------------------------------------------
    
    
    // Starting HFRCO (internal) clock
    CMU_OscillatorEnable(cmuOsc_HFRCO, true, true);
    // Enabling High Frequercny internal RC clock
    CMU_ClockSelectSet(cmuClock_HF , cmuSelect_HFRCO);

    
    // Select 21 MHz band for HFRCO
    CMU_HFRCOBandSet(cmuHFRCOFreq_48M0Hz);
    
    // Initialize the RTC clock
    RTCInit();
    WDOGn_Init(WDOG0, &wDogInitParam);
    
    coreClkFreq = CMU_ClockFreqGet(cmuClock_CORE);
    /* Setup SysTick Timer for 1 msec interrupts  */
    if (SysTick_Config(CMU_ClockFreqGet(cmuClock_CORE) / 1000)) {
        while (1) ;
    }
   
    
    // Start the RTOS
    OSInit(&err);

    //----------------------------- Queues INIT -----------------------------------------------
    OSQCreate(&taskMessagesFreeQueue, "Msg Free Queue", MAX_TASK_MESSAGES, &err);
    APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
    
    OSQCreate(&eventMessagesQueue, "Comm Queue", MAX_EVENS_MESSAGES, &err);
    APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
    
    OSQCreate(&eventMessagesFreeQueue, "Comm free Queue", MAX_EVENS_MESSAGES, &err);
    APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
    
    // -----------------------------  TASKS INIT -----------------------------------------------
    APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
    //Create Sys Task
    OSTaskCreate(&SYSTaskTCB,
                 "SYS Task",
                 SysTask,
                 DEF_NULL,
                 SYS_TASK_PRI,
                 &SYSTaskStack[0],
                 (SYS_TASK_STACK_SIZE / 10u),
                 SYS_TASK_STACK_SIZE,
                 15u,
                 0u,
                 DEF_NULL,
                 (OS_OPT_TASK_STK_CLR),
                 &err);
    
    APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
    //Create Cellular Task
    OSTaskCreate(&CellTaskTCB,
                 "Cellular Task",
                 CellularTask,
                 DEF_NULL,
                 CELL_TASK_PRI,
                 &CellTaskStack[0],
                 (CELL_TASK_STACK_SIZE / 10u),
                 CELL_TASK_STACK_SIZE,
                 15u,
                 0u,
                 DEF_NULL,
                 (OS_OPT_TASK_STK_CLR),
                 &err);
    
    APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
    
    OS_AppIdleTaskHookPtr = IdleTaskHookFunc;
    
    OSStart(&err);
    APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);    
    return (1);
}


//------------------------------------------------------------------------------
//  void QueuesInit(void)
//
//   Author:  Dilawar Ali
//   Date:    2018/05/22
//
//!  This function Initialize the Queues memory pool
//
//------------------------------------------------------------------------------
void QueuesInit(void)
{
    RTOS_ERR  err;
    
    for(uint8_t loopCounter = 0; loopCounter < MAX_TASK_MESSAGES; loopCounter++)
    {
        OSQPost(&taskMessagesFreeQueue, &taskMessageQueuesData[loopCounter], sizeof(SysMsg_t), OS_OPT_POST_FIFO + OS_OPT_POST_ALL + OS_OPT_POST_NO_SCHED, &err);
        APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
    }
    
    for(uint8_t loopCounter = 0; loopCounter < MAX_EVENS_MESSAGES; loopCounter++)
    {
        OSQPost(&eventMessagesFreeQueue, &evensMessageQueueData[loopCounter], sizeof(ComEvent_t), OS_OPT_POST_FIFO + OS_OPT_POST_ALL + OS_OPT_POST_NO_SCHED, &err);
        APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
    }
}

//------------------------------------------------------------------------------
//  void ClearWatchDogCounter(void)
//
//   Author:  Dilawar Ali
//   Date:    2018/09/17
//
//!  This function Initialize the Queues memory pool
//
//------------------------------------------------------------------------------
void ClearWatchDogCounter(void)
{
    WDOGn_Feed(WDOG0);
}