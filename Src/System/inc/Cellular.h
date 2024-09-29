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
//  Source:        Cellular.h
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

#ifndef CELLULAR_H
#define CELLULAR_H
//==============================================================================
//  INCLUDES
//==============================================================================
#include <stdint.h>
#include <em_usart.h>
#include <uartdrv.h>

#include "CellularATCommands.h"
#include "ExtCommunication.h"
#include "Event.h"
//==============================================================================
//  GLOBAL CONSTANTS, TYPEDEFS AND MACROS 
//==============================================================================

#define CELLULAR_UART_TX_BUFFER_SIZE        256u
#define CELLULAR_UART_RX_BUFFER_SIZE        1024u
#define CELLULAR_URL_BUFFER_SIZE            128u
#define CELLULAR_DATA_BUFFER_SIZE           1024u
#define CELLULAR_HEADER_BUFFER_SIZE         300u

#define CELLULAR_IMEI_LENGTH                16u
#define CELLULAR_CARRIER_LENGTH             32u
#define CELLULAR_ICCID_LENGTH               20u
#define IP_ADDR_LEN                         16u
#define SIM_APN_LEN                         64u
#define PHONE_NUMBER_LEN                    12u

#define CELL_END_DATA_CHAR                  36u
//---------------------- Cellular Error Codes ----------------------------------

#define ERR_UNKNOWN_ERROR                  (-1)
#define ERR_MODULE_NOT_RESPONDING          (-2)
#define ERR_AT_CMD_PARSER_FUCN_UNDEFINED   (-3)
#define ERR_CELLULAR_CME_ERROR             (-4)
#define ERR_UART_RX_TIMEOUT                (-5)
#define ERR_UART_RX_DATA_NULL              (-6)
#define ERR_UART_TX_DATA_NULL              (-7)
#define ERR_UART_NOT_OPEN                  (-8)
#define ERR_SIM_CARD_NOT_FOUND             (-9)
#define ERR_SIM_CARD_REGISTRATION_FAILED   (-10)
#define ERR_CERTIFICATE_INVALID            (-11)
#define ERR_TCP_SOCKET_ERROR               (-12)
#define ERR_UNABLE_TO_OPEN_TCP_SOCK        (-13)
#define ERR_TCP_SOCKET_WRITE_FAILED        (-14)
#define ERR_JSON_CREATE_FAILED             (-15)
#define ERR_INCOMPLETE_DATA_RECEIVED       (-16)
#define ERR_INVALID_SIGNAL_STRENGTH        (-17)
#define ERR_INVALID_AT_COMMAND             (-18)
#define ERR_SERVER_RESPONSE_PARSING_ERROR  (-19)
#define ERR_UNABLE_TO_OPEN_DIRECT_LINK     (-20)
//==============================================================================
//  GLOBAL DATA STRUCTURES DEFINITION
//==============================================================================

typedef enum
{
    
    CELL_INIT = 0,
    CELL_GPS_CONFIGURE,
    CELL_CONNECTION_TEST,
    CELL_SEND_EVENT_TO_INET,
    CELL_SEND_SMS,
    CELL_GET_GPS_COORDINATES,
    CELL_GPS_OFF,
    
    CELL_LAST_INVALID,
}CELL_MSG_ID_t;

typedef struct
{
    CELL_MSG_ID_t msgId;
    uint16_t msgInfo;
    void * ptrData;
}CellMsg_t;

typedef enum
{
    CELLULAR_UNINIT = 0,
    CELLULAR_IDLE,
    CELLULAR_READY,
    CELLULAR_BUSY,
    CELLULAR_ERROR,
} CELLL_STATE_t;

typedef struct CellularDriver
{
    UARTDRV_Handle_t cellUART;
    uint8_t UARTTxBuffer[CELLULAR_UART_TX_BUFFER_SIZE+1];
    uint8_t UARTRxBuffer[CELLULAR_UART_RX_BUFFER_SIZE+1];
    
    CELLL_STATE_t cellularState;
    int32_t       errorCode;
    
    uint8_t signalStrength;
    uint8_t imei[CELLULAR_IMEI_LENGTH+1];
    uint8_t carrier[CELLULAR_CARRIER_LENGTH+1];
    uint8_t iccid[CELLULAR_ICCID_LENGTH+1];
    uint8_t ipAddr[IP_ADDR_LEN+1];
    uint32_t TCPSocket;
    
    uint8_t antennaType;
    uint8_t APN[SIM_APN_LEN+1];
    uint8_t simPhoneNumber[PHONE_NUMBER_LEN+1];
    
    uint8_t cellNumber1[PHONE_NUMBER_LEN+1];
    uint8_t cellNumber2[PHONE_NUMBER_LEN+1];
    
    BOOLEAN isCellularRegistered;
    BOOLEAN isRTCTimeUpdated;
    
    ATCOMMAND_INDEX_ENUM currentATIndex;
    PTR_COMM_EVT_t runningCommEvent;
}CellularDriver_t;

typedef struct
{
    BOOLEAN isTokenValid;
    BOOLEAN isEventSent;
    BOOLEAN isUARTReadStarted;
    int8_t  endReading;
    uint8_t UARTWaitCounter;
    uint32_t status;
    
    uint32_t readyBytes;
    uint32_t remainingBytes;
    uint32_t receivedBytes;
    uint32_t contentReceived;
    uint32_t contentLength;
    
    uint8_t *jsonHeader;
}ReceivedDataInfo_t;
//==============================================================================
//  GLOBAL DATA
//==============================================================================
extern CellularDriver_t gCellularDriver;
extern ReceivedDataInfo_t cellHttpsReceiving;
extern uint8_t httpUrlBuffer[];
extern uint8_t tokenBuffer[];
extern uint8_t cellDataBuffer[];
extern uint8_t cellHeaderBuffer[];

extern uint8_t cellular_initialized;
extern uint8_t gps_initialized;


//==============================================================================
//  EXTERNAL OR GLOBAL FUNCTIONS
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
void CellularTask(void *arg);
#endif
