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

#ifndef CELLULARATCOMMAND_H
#define CELLULARATCOMMAND_H
//==============================================================================
//  INCLUDES
//==============================================================================
#include <stdint.h>
#include <string.h>
//==============================================================================
//  GLOBAL CONSTANTS, TYPEDEFS AND MACROS 
//==============================================================================

#define CELL_MAX_RESPONSE_BYTES             1024u

//==============================================================================
//  GLOBAL DATA STRUCTURES DEFINITION
//==============================================================================

typedef enum
{
    ATC_AT = 0,
    ATC_ATE,
    ATC_CMEE,
    ATC_CPIN_Q,
    ATC_CGSN,
    ATC_ICCID,
    ATI,
    ATC_URAT,
    ATC_CSQ,
    ATC_CREG_Q,
    ATC_COPS_Q,
    ATC_CGDCONT,
    ATC_CCLK_Q,
    ATC_CGATT,
    ATC_CGACT,
    ATC_CGPADDR,
    ATC_USECMNG,
    ATC_CERTWRITE,
    ATC_USECPRF_1,
    ATC_USECPRF_2,
    ATC_USECPRF_3,
    ATC_USECPRF_4,
    ATC_USOCR,
    ATC_UDCONF,
    ATC_USOSEC,
    ATC_USOCO,
    AT_USODL,
    ATC_USODL_CLOSE,
    ATC_WRITEHEADER,
    ATC_USOWR,
    ATC_USORD,
    ATC_USOCL,
    ATC_USOCLCFG,
    ATC_CFUN_0,
    ATC_CFUN_1,
    ATC_GNNS_SUPPLY_EN,
    ATC_GNNS_DATA_READY,
    ATC_UI2CO,
    ATC_UI2CW,
    ATC_UI2CR,
    ATC_GNSS_ON, //
    ATC_GNSS_OFF,
    ATC_GNSS_TEST,
    ATC_UGIND, //AT+UGIND=1
    ATC_UGPRF, //AT+UGPRF=16 
    ATC_UGZDA, //AT+UGZDA=1, get GNSS data and time
    ATC_UGGGA, //AT+UGGGA=1, enable and get $GGA message, time position and fix related data
    ATC_UGGGA_DATA,
    ATC_UGGSV_ENABLE,
    ATC_UGGSV_DATA,
    ATC_ULOCGNSS,
    ATC_ULOC,
    
    
    ATC_LAST_POS,
} ATCOMMAND_INDEX_ENUM;

typedef struct CellularDriver * PtrCellularDriver_t;
typedef int32_t (*FPtrCmpFunc_t)(uint8_t response[], int32_t response_buf_length);

typedef struct ATCommand
{
    uint8_t * cmd;
    uint32_t timeout;
    const FPtrCmpFunc_t cmp;
    uint32_t responseBytes;
    uint32_t responseDelayTime;
} ATCOMMAND_STRUCT;

//==============================================================================
//  GLOBAL DATA
//==============================================================================
extern ATCOMMAND_STRUCT CelluarATCommands[ATC_LAST_POS];

//==============================================================================
//  EXTERNAL OR GLOBAL FUNCTIONS
//==============================================================================

//------------------------------------------------------------------------------
//  uint32_t CreateUARTTXdata(ATCOMMAND_INDEX_ENUM cmdIndex, uint8_t Buffer[], uint32_t buffSize)
//
//   Author:  Dilawar Ali
//   Date:    2018/05/28
//
//!  This function create cellular UART Tx data i.e AT command Data
//
//------------------------------------------------------------------------------
uint32_t CreateUARTTXdata(ATCOMMAND_INDEX_ENUM cmdIndex, uint8_t Buffer[], uint32_t buffSize);
#endif