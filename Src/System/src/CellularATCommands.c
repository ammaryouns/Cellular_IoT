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
//  Source:        CellularATCommands.c
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


//==============================================================================
//  INCLUDES
//==============================================================================

#include "CellularATCommands.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "ExtCommunication.h"
#include "Cellular.h"
#include "Timer.h"
#include "Event.h"
//==============================================================================
//  CONSTANTS, TYPEDEFS AND MACROS 
//==============================================================================

//==============================================================================
//  LOCAL DATA DECLARATIONS
//==============================================================================

//==============================================================================
//  LOCAL FUNCTION PROTOTYPES
//==============================================================================
static int32_t OKMsgCmpFun                (uint8_t response[],  int32_t response_buf_length);
static int32_t CPINCmpFun                 (uint8_t response[],  int32_t response_buf_length);
static int32_t IMEICmpFun                 (uint8_t response[],  int32_t response_buf_length);
static int32_t ICCIDCmpFun                (uint8_t response[],  int32_t response_buf_length);
static int32_t SignalStrengthCmpFun       (uint8_t response[],  int32_t response_buf_length);
static int32_t CREGQouteCmpFun            (uint8_t response[],  int32_t response_buf_length);
static int32_t OperatorQryCmpFun          (uint8_t response[],  int32_t response_buf_length);
static int32_t TimeQryCmpFun              (uint8_t response[],  int32_t response_buf_length);
static int32_t ShowIPCmpFun               (uint8_t response[],  int32_t response_buf_length);
static int32_t InputCmpFun                (uint8_t response[],  int32_t response_buf_length);
static int32_t CertWriteCmpFun            (uint8_t response[],  int32_t response_buf_length);
static int32_t TCPSocketCmpFun            (uint8_t response[],  int32_t response_buf_length);
static int32_t SocketOpenCmpFun           (uint8_t response[],  int32_t response_buf_length);
static int32_t SocDirectLinkCmpFun        (uint8_t response[],  int32_t response_buf_length);
static int32_t SocketDataReadWriteCmpFun  (uint8_t response[],  int32_t response_buf_length);
static int32_t DirectLinkDownCmpFun       (uint8_t response[],  int32_t response_buf_length);
static int32_t GPSParserCmpFun            (uint8_t response[],  int32_t response_buf_length);
static int32_t GPSSetParserCmpFun         (uint8_t response[],  int32_t response_buf_length);

static void ParseGPSReceivedData(uint8_t GPSData[], uint32_t Length);
static uint8_t TokenizeString(uint8_t *srcString, uint8_t dstToken[][25], uint8_t c_Delimiter, uint8_t messageLength);
//==============================================================================
//  GLOBAL DATA DECLARATIONS
//==============================================================================
/*
ATC_AT = 0,
ATC_ATE,
ATC_CMEE,
ATC_CPIN_Q,
ATC_CGSN,
ATC_ICCID,
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

ATC_LAST_POS,
}
*/
ATCOMMAND_STRUCT CelluarATCommands[ATC_LAST_POS]=
{
    {//ATC_AT
        "AT\r\n",
        1000,
        OKMsgCmpFun,
        6u,
        0u,
    },
    {//ATC_ATE
        "ATE0\r\n",
        1000,
        OKMsgCmpFun,
        6u,
        0u,
    },
    {//ATC_CMEE
        "AT+CMEE=2\r\n",
        2000,
        OKMsgCmpFun,
        6u,
        0u,
    },
    {//ATC_CPIN_Q
        "AT+CPIN?\r\n",
        1500,
        CPINCmpFun,
        CELL_MAX_RESPONSE_BYTES,
        0u,
    },
    {//ATC_CGSN
        "AT+CGSN\r\n",
        1500,
        IMEICmpFun,
        CELL_MAX_RESPONSE_BYTES,
        0u,
    },   
    {//ATC_ICCID
        "AT+ICCID\r\n",
        1500,
        ICCIDCmpFun,
        CELL_MAX_RESPONSE_BYTES,
        0u,
    },
    
    {//ATI
        "ATI\r\n",
        1000u,
        OKMsgCmpFun,
        CELL_MAX_RESPONSE_BYTES,
        0u,
    },
    
    {//ATC_URAT
        "AT+URAT=7,8,9\r\n",
        1000u,
        OKMsgCmpFun,
        CELL_MAX_RESPONSE_BYTES,
        0u,
    },
    {//ATC_CSQ
        "AT+CSQ\r\n",
        1500,
        SignalStrengthCmpFun,
        CELL_MAX_RESPONSE_BYTES,
        0u,
    },
    {//ATC_CREG_Q
        "AT+CREG?\r\n",
        2000,
        CREGQouteCmpFun,
        CELL_MAX_RESPONSE_BYTES,
        0u,
    },
    {//ATC_COPS_Q
        "AT+COPS?\r\n",
        3000,
        OperatorQryCmpFun,
        CELL_MAX_RESPONSE_BYTES,
        0u,
    },
    {//ATC_CGDCONT
        "AT+CGDCONT=1,\"IP\",\"11583.mcs\"\r\n",
        3000,
        OKMsgCmpFun,
        6u,
        0,
    },
    
    {//ATC_CCLK_Q
        "AT+CCLK?\r\n",
        2000,
        TimeQryCmpFun,
        CELL_MAX_RESPONSE_BYTES,
        0u,
    },
    {//ATC_CGATT
        "AT+CGATT=1\r\n",
        2000,
        OKMsgCmpFun,
        6u,
        0u,
    },
    {//ATC_CGACT
        "AT+CGACT=1,1\r\n",
        4000,
        OKMsgCmpFun,
        CELL_MAX_RESPONSE_BYTES,
        0u,
    },
    
    {//ATC_CGPADDR,
        "AT+CGPADDR=1\r\n",
        2000,
        ShowIPCmpFun,
        CELL_MAX_RESPONSE_BYTES,
        0u,
    },
    {//ATC_USECMNG
        "AT+USECMNG=0,0,\"iNetCert.der\",1367\r\n",
        1000,
        InputCmpFun,
        CELL_MAX_RESPONSE_BYTES,
        0u,
    },
    {//ATC_CERTWRITE
        (uint8_t *)inetwasdev1Cert,
        1000,
        CertWriteCmpFun,
        CELL_MAX_RESPONSE_BYTES,
        0u,
    },
    {//ATC_USECPRF_1
        "AT+USECPRF=0,0,1\r\n",
        15000,
        OKMsgCmpFun,
        6u,
        0u,
    },
    {//ATC_USECPRF_2
        "AT+USECPRF=0,1,0\r\n",
        15000,
        OKMsgCmpFun,
        6u,
        0u,
    },
    {//ATC_USECPRF_3
        "AT+USECPRF=0,2,0\r\n",
        15000,
        OKMsgCmpFun,
        6u,
        0u,
    },
    {//ATC_USECPRF_4
        "AT+USECPRF=0,3,\"iNetCert.der\"\r\n",
        15000,
        OKMsgCmpFun,
        6u,
        0u,
    },
    {//ATC_USOCR
        "AT+USOCR=6\r\n",
        12000,
        TCPSocketCmpFun,
        CELL_MAX_RESPONSE_BYTES,
        0u,
    },
    {//ATC_UDCONF
        //"AT+UDCONF=7,<*socket>,36\r\n",
        cellDataBuffer,
        500,
        OKMsgCmpFun,
        CELL_MAX_RESPONSE_BYTES,
        0u,
    },
    {//ATC_USOSEC,
        //"AT+USOSEC=<*socket>,1,0",
        cellDataBuffer,
        2000,
        OKMsgCmpFun,
        6u,
        0u,
    },
    {//ATC_USOCO,
        //"AT+USOCO=<*socket>,"inetuploadft.indsci.com",443",
        cellDataBuffer,
        40000,
        SocketOpenCmpFun,
        CELL_MAX_RESPONSE_BYTES,
        0u,
    },
    {//AT_USODL,
        //"AT+USODL=<*socket>",
        cellDataBuffer,
        2000,
        SocDirectLinkCmpFun,
        CELL_MAX_RESPONSE_BYTES,
        0u,
    },
    {//ATC_USODL_CLOSE
        "+++",
        500,
        DirectLinkDownCmpFun,
        CELL_MAX_RESPONSE_BYTES,
        0u,
        
    },
    {//ATC_WRITEHEADER,,
        cellHeaderBuffer,
        0,
        NULL,
        0,
        0u,
    },
    {//ATC_USOWR,
        cellDataBuffer,
        25000,
        SocketDataReadWriteCmpFun,
        CELL_MAX_RESPONSE_BYTES,
        0u,
    },
    {//ATC_USORD
        //AT+USORD=<socket>, datalength
        cellDataBuffer,
        5000,
        OKMsgCmpFun,
        CELL_MAX_RESPONSE_BYTES,
        0u,
    },
    {//ATC_USOCL,
        //"AT+USOCL=<*socket>",
        cellDataBuffer,
        5000,
        OKMsgCmpFun,
        CELL_MAX_RESPONSE_BYTES,
        0u,
    },
    {//ATC_USOCLCFG
        "AT+USOCLCFG=1\r\n",
        1000,
        OKMsgCmpFun,
        6u,
        0u,
    },
    
    {//ATC_CFUN_0
        "AT+CFUN=0\r\n",
        1000,
        OKMsgCmpFun,
        CELL_MAX_RESPONSE_BYTES,
        0u,
    },
    {//ATC_CFUN_1
        "AT+CFUN=1\r\n",
        1000,
        OKMsgCmpFun,
        CELL_MAX_RESPONSE_BYTES,
        0u,
    },    
    
    
    
    {//ATC_GNNS_SUPPLY_EN,
        "AT+UGPIOC=23,3\r\n",  // Configure GPIO2 as GNSS supply enable
        5000,
        OKMsgCmpFun,
        6u,
        0u,
    },
    {//ATC_GNNS_DATA_READY,
        "AT+UGPIOC=24,4\r\n",  // Configure GPIO3 as GNSS data ready input
        5000,
        OKMsgCmpFun,
        6u,
        0u,
    },
    {//ATC_UI2CO,
        "AT+UI2CO=1,0,0,0x42,0\r\n",  // 
        5000,
        OKMsgCmpFun,
        6u,
        0u,
    },
    {//ATC_UI2CW
        cellDataBuffer,  // 
        50000,
        OKMsgCmpFun,
        CELL_MAX_RESPONSE_BYTES,
        0u,
    },
    
    {//ATC_UI2CR,
        "AT+UI2CR=3\r\n",  // 
        5000,
        OKMsgCmpFun,
        6u,
        0u,
    },
    {//ATC_GNSS_ON
        "AT+UGPS=1,0,3\r\n",  // 
        50000,
        OKMsgCmpFun,
        CELL_MAX_RESPONSE_BYTES,
        0u,
    },
    {//ATC_GNSS_OFF
        "AT+UGPS=0\r\n",  // 
        5000,
        OKMsgCmpFun,
        CELL_MAX_RESPONSE_BYTES,
        0u,
    },
    {//ATC_GNSS_TEST
        "AT+UGPS=?\r\n",  // 
        5000,
        OKMsgCmpFun,
        CELL_MAX_RESPONSE_BYTES,
        0u,
    },
    
    {//ATC_UGIND,
        "AT+UGIND=1\r\n",  // 
        5000,
        OKMsgCmpFun,
        CELL_MAX_RESPONSE_BYTES,
        0u,
    },
    {//ATC_UGPRF,
        "AT+UGPRF=16\r\n",  // 
        15000,
        OKMsgCmpFun,
        CELL_MAX_RESPONSE_BYTES,
        0u,
    },
    {//ATC_UGZDA,
        "AT+UGZDA=1\r\n",  // get GNSS data and time
        5000,
        OKMsgCmpFun,
        6u,
        0u,
    },
    {//ATC_UGGGA,
        "AT+UGGGA=1\r\n",  //enable and get $GGA message, time position and fix related data
        5000,
        OKMsgCmpFun,
        200u,
        0u,
    },
    {//ATC_UGGGA_DATA,
        "AT+UGGGA?\r\n",  //get UGGA data
        8000,
        GPSParserCmpFun,
        200u,
        0u,
    },
    {//ATC_UGGSV_ENABLE
        "AT+UGGSV=1\r\n",  //enable last UGGSV
        8000,
        OKMsgCmpFun,
        10u,
        0u,
    },
    {//ATC_UGGSV_DATA
        "AT+UGGSV?\r\n",  //enable last UGGSV
        8000,
        GPSSetParserCmpFun,
        200u,
        0u,
    },
    {//ATC_ULOCGNSS
        "AT+ULOCGNSS=15\r\n",
        5000,
        OKMsgCmpFun,
        200u,
        0u,
    },
    {//ATC_ULOC
        "AT+ULOC=2,3,0,120,10\r\n",
        5000,
        OKMsgCmpFun,
        200u,
        0u,
    },

    

    
};
//==============================================================================
//  LOCAL FUNCTIONS IMPLEMENTATION
//==============================================================================

//------------------------------------------------------------------------------
//  static int32_t OKMsgCmpFun(uint8_t response[],  int32_t response_buf_length)
//
//   Author:  Dilawar Ali
//   Date:    2018/05/28
//
//!  This function parse the response of commands that should return 'OK'
//
//------------------------------------------------------------------------------
static int32_t OKMsgCmpFun(uint8_t response[],  int32_t response_buf_length)
{
    uint8_t *startPtr = NULL;
    int32_t ret = -1;
    if(strncmp((char const*)response, "OK", 2u) == 0)
    {
        ret = 0;
    }
    else
    {
        startPtr = (uint8_t *)strstr((char const*) response, "OK");
        if (startPtr != NULL)
        {
            ret = 0;
        }
    }
    
    return ret;
}

//------------------------------------------------------------------------------
//  static int32_t CPINCmpFun(uint8_t response[],  int32_t response_buf_length)
//
//   Author:  Dilawar Ali
//   Date:    2018/05/28
//
//!  This function parse the response of SIM card detection command
//
//------------------------------------------------------------------------------
static int32_t CPINCmpFun(uint8_t response[],  int32_t response_buf_length)
{
    int32_t ret = ERR_INCOMPLETE_DATA_RECEIVED;
    if(strncmp((char const*)response, "+CPIN: READY", 12u) == 0)
    {
        ret = 0;
    }
    return ret;
}

//------------------------------------------------------------------------------
//  static int32_t IMEICmpFun(uint8_t response[],  int32_t response_buf_length)
//
//   Author:  Dilawar Ali
//   Date:    2018/05/28
//
//!  This function parse the IMEI Number from response of AT command
//
//------------------------------------------------------------------------------
static int32_t IMEICmpFun(uint8_t response[],  int32_t response_buf_length)
{
    int32_t ret = 0;
    if(response_buf_length >= 23u)
    {
        strncpy((char *)gCellularDriver.imei, (char const*)response, 15u);
    }
    else
    {
        ret = ERR_INCOMPLETE_DATA_RECEIVED;
    }
    return ret;
}


//------------------------------------------------------------------------------
//  static int32_t ICCIDCmpFun(uint8_t response[],  int32_t response_buf_length)
//
//   Author:  Dilawar Ali
//   Date:    2018/06/11
//
//!  This function parse the SIM Card ICCID Number from response of AT command
//
//------------------------------------------------------------------------------
static int32_t ICCIDCmpFun(uint8_t response[],  int32_t response_buf_length)
{
    int32_t ret = 0;
    uint8_t *iccid = NULL;
    uint8_t counter = 0;
    if(strncmp((char const*)response, "ICCID: ", 7u) == 0)
    {
        iccid = (uint8_t *)strstr((char const*)response, ": ");
        if(iccid != NULL)
        {
            iccid++;
            iccid++;
            while((*iccid != '\r') && (*iccid != '\0'))
            {
                gCellularDriver.iccid[counter++] = *iccid++;
                if(counter >= CELLULAR_ICCID_LENGTH)
                {
                    break;
                }
            }
            ret = 0;
        }
    }
    else
    {
        ret = ERR_INCOMPLETE_DATA_RECEIVED;
    }
    return ret;
}
//------------------------------------------------------------------------------
//  static int32_t SignalStrengthCmpFun(uint8_t response[],  int32_t response_buf_length)
//
//   Author:  Dilawar Ali
//   Date:    2018/05/28
//
//!  This function parse the Signal Strength from response of AT command
//
//------------------------------------------------------------------------------
static int32_t SignalStrengthCmpFun(uint8_t response[],  int32_t response_buf_length)
{
    int32_t ret = 0;
    uint32_t signalStrength, bitErrorRate;
    if(strncmp((char const*)response, "+CSQ:", 5u) == 0)
    {
        sscanf((char const*)response, "+CSQ: %d,%d", &signalStrength, &bitErrorRate);
        if((signalStrength == 99u) && (bitErrorRate == 99u))
        {
            ret = ERR_INVALID_SIGNAL_STRENGTH;
        }
        else
        {
            gCellularDriver.signalStrength = (uint8_t) signalStrength;
        }
    }
    else
    {
        ret = ERR_INCOMPLETE_DATA_RECEIVED;
    }
    return ret;
}


//------------------------------------------------------------------------------
//  static int32_t CREGQouteCmpFun(uint8_t response[],  int32_t response_buf_length)
//
//   Author:  Dilawar Ali
//   Date:    2018/05/28
//
//!  This function parse the Status of sim registration on cellular Network from 
//!  response of AT command
//
//------------------------------------------------------------------------------
static int32_t CREGQouteCmpFun(uint8_t response[],  int32_t response_buf_length)
{
    int32_t ret = ERR_INCOMPLETE_DATA_RECEIVED;
    int32_t register_state = -1;
    
    if(strncmp((char const*)response, "+CREG: ", 6u) == 0)
    {
        sscanf((char const*)response, "+CREG: 0,%d", &register_state);
        //        printf("Register state: %d\n", register_state);
        
        switch(register_state)
        {
            break;
        case 1:
            //Registered at home network
            gCellularDriver.isCellularRegistered = true;
            ret = 0;
            break;
        case 5:
            //Registered in roaming
            gCellularDriver.isCellularRegistered = true;
            ret = 0;
            break;
        default:
            gCellularDriver.isCellularRegistered = false;
            break;
        }
    }
    else
    {
        //DO Nothing
    }
    
    return ret;
}

//------------------------------------------------------------------------------
//  static int32_t OperatorQryCmpFun(uint8_t response[],  int32_t response_buf_length)
//
//   Author:  Dilawar Ali
//   Date:    2018/05/28
//
//!  This function parse the Cellular Operator information from response of AT command
//
//------------------------------------------------------------------------------
static int32_t OperatorQryCmpFun(uint8_t response[],  int32_t response_buf_length)
{
    int32_t ret = ERR_INCOMPLETE_DATA_RECEIVED;
    uint8_t *cellOperator = NULL;
    uint32_t counter = 0;
    if(strncmp((char const*)response, "+COPS: ", 6u) == 0)
    {
        cellOperator = (uint8_t *)strstr((char const*)response, "\"");
        if(cellOperator != NULL)
        {
            cellOperator++;
            while((*cellOperator != '\"') && (*cellOperator != '\0'))
            {
                gCellularDriver.carrier[counter++] = *cellOperator++;
                if(counter >= CELLULAR_CARRIER_LENGTH)
                {
                    break;
                }
            }
            ret = 0;
        }
    }
    
    return ret;
}


//------------------------------------------------------------------------------
//  static int32_t TimeQryCmpFun(uint8_t response[],  int32_t response_buf_length)
//
//   Author:  Dilawar Ali
//   Date:    2018/05/28
//
//!  This function parse the Cellular Operator information from response of AT command
//
//------------------------------------------------------------------------------
static int32_t TimeQryCmpFun(uint8_t response[],  int32_t response_buf_length)
{
    int32_t ret = ERR_INCOMPLETE_DATA_RECEIVED;
    
    uint8_t *time = NULL;
    uint8_t tBuff[24] = {0};
    uint32_t counter = 0;
    if(strncmp((char const*)response, "+CCLK: ", 6u) == 0)
    {
        time = (uint8_t *)strstr((char const*)response, "\"");
        if(time != NULL)
        {
            time++;
            while((*time != '\"') && (*time != '\0'))
            {
                tBuff[counter++] = *time;
                if(counter >= 24)
                {
                    break;
                }
                time++;
            }
//            if(gCellularDriver.isRTCTimeUpdated == false)
//            {
                UpdateRTCTime(tBuff);
                gCellularDriver.isRTCTimeUpdated = true;
//            }
            gCellularDriver.isRTCTimeUpdated = true;
            ret = 0;
        }
    }
    return ret;
}


//------------------------------------------------------------------------------
//  static int32_t ShowIPCmpFun(uint8_t response[],  int32_t response_buf_length)
//
//   Author:  Dilawar Ali
//   Date:    2018/05/28
//
//!  This function parse the Cellular Network IP address from response of AT command
//
//------------------------------------------------------------------------------
static int32_t ShowIPCmpFun(uint8_t response[],  int32_t response_buf_length)
{
    int32_t ret = ERR_INCOMPLETE_DATA_RECEIVED;
    uint8_t *ip = NULL;
    uint32_t counter = 0;
    if(strncmp((char const*)response, "+CGPADDR: ", 9u) == 0)
    {
        ip = (uint8_t *)strstr((char const*)response, ",");
        if(ip != NULL)
        {
            ip++;
            while((*ip != '\r') && (*ip != '\0'))
            {
                gCellularDriver.ipAddr[counter++] = *ip;
                if(counter >= IP_ADDR_LEN)
                {
                    break;
                }
                ip++;
            }
            ret = 0;
        }
    }
    return ret;
}


//------------------------------------------------------------------------------
//  static int32_t InputCmpFun(uint8_t response[],  int32_t response_buf_length)
//
//   Author:  Dilawar Ali
//   Date:    2018/05/28
//
//!  This function parse the data input character '>' from response of AT command
//
//------------------------------------------------------------------------------
static int32_t InputCmpFun(uint8_t response[],  int32_t response_buf_length)
{
    int32_t ret = ERR_INCOMPLETE_DATA_RECEIVED;
    uint8_t *startPtr = NULL;
    
    if(strncmp((char const*)response, ">", 1u) == 0)
    {
        ret = 0;
    }
    else
    {
        startPtr = (uint8_t *)strstr((char const*) response, ">");
        if (startPtr != NULL)
        {
            ret = 0;
        }
    }
    
    return ret;
}

//------------------------------------------------------------------------------
//  static int32_t CertWriteCmpFun(uint8_t response[],  int32_t response_buf_length);
//
//   Author:  Dilawar Ali
//   Date:    2018/06/12
//
//!  This function parse the response of Certificate write
//
//------------------------------------------------------------------------------
static int32_t CertWriteCmpFun(uint8_t response[],  int32_t response_buf_length)
{
    int32_t ret = ERR_INCOMPLETE_DATA_RECEIVED;
    if(strncmp((char const*)response, "+USECMNG: ", 9u) == 0)
    {
        ret = 0;
    }
    return ret;
}

//------------------------------------------------------------------------------
//  static int32_t TCPSocketCmpFun(uint8_t response[],  int32_t response_buf_length)
//
//   Author:  Dilawar Ali
//   Date:    2018/05/28
//
//!  This function parse the Info of TCP configuration from response of AT command
//
//------------------------------------------------------------------------------
static int32_t TCPSocketCmpFun(uint8_t response[],  int32_t response_buf_length)
{
    uint8_t *startPtr = NULL;
    int32_t ret;
    ret = ERR_INCOMPLETE_DATA_RECEIVED;
    if(strncmp((char const*)response, "+USOCR: ", 7u) == 0)
    {
        sscanf((char const*)response, "+USOCR: %d", &gCellularDriver.TCPSocket);
        //        printf("TCP Socket ID: %d\n", gCellularDriver.TCPSocket);
        ret = 0;
    }
    else
    {
        startPtr = (uint8_t *)strstr((char const*) response, "+USOCR:");
        if (startPtr != NULL)
        {
            sscanf((char const*)response, "+USOCR: %d", &gCellularDriver.TCPSocket);
            ret = 0;
        }
    }
    return ret;
}


//------------------------------------------------------------------------------
//  static int32_t SocketOpenCmpFun(uint8_t response[],  int32_t response_buf_length)
//
//   Author:  Dilawar Ali
//   Date:    2018/05/28
//
//!  This function parse the Socket status from response of AT command
//
//------------------------------------------------------------------------------
static int32_t SocketOpenCmpFun(uint8_t response[],  int32_t response_buf_length)
{
    uint8_t *startPtr = NULL;
    int32_t ret = ERR_INCOMPLETE_DATA_RECEIVED;
    //    printf("Socket Open Response: %s\r\n", response);
    if(strncmp((char const*)response, "OK", 2u) == 0)
    {
        ret = 0;
    }
    else
    {
        startPtr = (uint8_t *)strstr((char const*) response, "OK");
        if (startPtr != NULL)
        {
            ret = 0;
        }
    }
    
    return ret;
}


//------------------------------------------------------------------------------
//  static int32_t SocDirectLinkCmpFun(uint8_t response[],  int32_t response_buf_length)
//
//   Author:  Dilawar Ali
//   Date:    2018/05/28
//
//!  This function parse the Direct socket data link status from response of AT command
//
//------------------------------------------------------------------------------
static int32_t SocDirectLinkCmpFun(uint8_t response[],  int32_t response_buf_length)
{
    uint8_t *startPtr = NULL;
    int32_t ret = ERR_INCOMPLETE_DATA_RECEIVED;
    if(strncmp((char const*)response, "CONNECT", 7u) == 0)
    {
        ret = 0;
    }
    else
    {
        startPtr = (uint8_t *)strstr((char const*) response, "CONNECT");
        if (startPtr != NULL)
        {
            ret = 0;
        }
    }
    return ret;
}

//------------------------------------------------------------------------------
//  static int32_t SocketDataReadWriteCmpFun(uint8_t response[],  int32_t response_buf_length)
//
//   Author:  Dilawar Ali
//   Date:    2018/05/28
//
//!  This function parse the status of data written to socket from response of AT command
//
//------------------------------------------------------------------------------
static int32_t SocketDataReadWriteCmpFun(uint8_t response[],  int32_t response_buf_length)
{
    int32_t ret = ERR_INCOMPLETE_DATA_RECEIVED;
    
    float32_t version;
    uint8_t *idLength = NULL;
    uint8_t *httpResponse = (uint8_t *)strstr((char const*)response, "HTTP");
    if(httpResponse != NULL)
    {
        sscanf((char const*)httpResponse, "HTTP/%f %d", &version, &cellHttpsReceiving.status);
        //        printf("Server response: %d\n", cellHttpsReceiving.status);
    }
    if((cellHttpsReceiving.status == 200u) || (cellHttpsReceiving.status == 201u))
    {
        if(cellHttpsReceiving.isTokenValid == false)
        {
            cellHttpsReceiving.jsonHeader = (uint8_t *)strstr((char const*)response, "{");
            if(cellHttpsReceiving.jsonHeader != NULL)
            {
                ret = 0;
            }
        }
        else
        {
            cellHttpsReceiving.isEventSent = true;
            idLength = (uint8_t *)strstr((char const*)httpResponse, "Content-Length:");
            sscanf((char const*)idLength, "Content-Length: %d", &cellHttpsReceiving.contentLength);
            cellHttpsReceiving.jsonHeader = (uint8_t *)strstr((char const*)httpResponse, "\r\n\r\n");
            if(cellHttpsReceiving.jsonHeader != NULL)
            {
                cellHttpsReceiving.jsonHeader = &cellHttpsReceiving.jsonHeader[4];
                
                if((cellHttpsReceiving.contentLength > 0u) && (cellHttpsReceiving.jsonHeader != NULL))
                {
                    ret = 0;
                }
            }
        }
    }
    else if(cellHttpsReceiving.status == 401u)
    {
        cellHttpsReceiving.isTokenValid = false;
    }
    else
    {
        ret = ERR_INCOMPLETE_DATA_RECEIVED;
    }
    
    return ret;
}

//------------------------------------------------------------------------------
//  static int32_t DirectLinkDownCmpFun  (uint8_t response[],  int32_t response_buf_length)
//
//   Author:  Dilawar Ali
//   Date:    2018/05/28
//
//!  This function is for response received for Direct link down command
//
//------------------------------------------------------------------------------
static int32_t DirectLinkDownCmpFun  (uint8_t response[],  int32_t response_buf_length)
{
    int32_t ret = 0;
    if(strncmp((char const*)response, "DISCONNECTED: ", 12u) == 0)
    {
        ret = 0;
    }
    return ret;
}

//------------------------------------------------------------------------------
//  static int32_t GPSParserCmpFun  (uint8_t response[],  int32_t response_buf_length)
//
//   Author:  Dilawar Ali
//   Date:    2018/08/18
//
//!  This function parse NMEA string received from GNSS module
//
//------------------------------------------------------------------------------
static int32_t GPSParserCmpFun  (uint8_t response[],  int32_t response_buf_length)
{
    int32_t ret = ERR_INCOMPLETE_DATA_RECEIVED;
    printf("%s\r\n", response);
    if(response_buf_length > 22)
    {
        if(strncmp((char const*)response, "+UGGGA: ", 6u) == 0)
        {
            ParseGPSReceivedData(&response[10], (response_buf_length - 10 - 11));
            ret = 0;
        }
    }
    return ret;
}

//------------------------------------------------------------------------------
//  static int32_t GPSSetParserCmpFun  (uint8_t response[],  int32_t response_buf_length)
//
//   Author:  Dilawar Ali
//   Date:    2018/08/18
//
//!  This function parse NMEA string received from GNSS module
//
//------------------------------------------------------------------------------
static int32_t GPSSetParserCmpFun  (uint8_t response[],  int32_t response_buf_length)
{
    int32_t ret = ERR_INCOMPLETE_DATA_RECEIVED;
    printf("%s\r\n", response);
    if(response_buf_length > 22)
    {
        if(strncmp((char const*)response, "+UGGSV: ", 6u) == 0)
        {
//            ParseGPSReceivedData(&response[10], (response_buf_length - 10 - 11));
            ret = 0;
        }
    }
    return ret;
}
//------------------------------------------------------------------------------
//  void ParseGPSReceivedData(uint8_t GPSData[], uint32_t Length)
//
//   Author:  Dilawar Ali
//   Date:    2018/08/18
//
//!  This function NMEA sentence received from GNSS module
//
//------------------------------------------------------------------------------
static void ParseGPSReceivedData(uint8_t GPSData[], uint32_t Length)
{
    uint8_t sToken[16][25];
    memset(sToken, 0, (16*25));
    
    uint8_t numberOfTokens = TokenizeString(GPSData, sToken, ',', Length);
    if(numberOfTokens == 16)
    {
        GPSReceivedCoordinates.latitude             = (float32_t) atof((char*)sToken[1u]);
        GPSReceivedCoordinates.latitudeDir          = (int8_t)    *sToken[2u];
        GPSReceivedCoordinates.longitude            = (float32_t) atof((char*)sToken[3u]);
        GPSReceivedCoordinates.longitudeDir         = (int8_t)    *sToken[4u];
        GPSReceivedCoordinates.accuracy             = (int32_t)   atoi((char*)sToken[5u]);
        GPSReceivedCoordinates.horizantalDilution   = (float32_t) atof((char*)sToken[7u]);
        if(GPSReceivedCoordinates.accuracy > 0)
        {
            GPSReceivedCoordinates.isGpsValid = true;
        }
        else
        {
            GPSReceivedCoordinates.isGpsValid = false;
        }
    }
    
}

//==============================================================================
//
//   TokenizeString(char *s_String, char s_Token[][25], char c_Delimiter, 
//                                                  unsigned char messageLength)
//
//   Author:   Tayyab Tahir(in Cornell)
//   Date:     2018/05/21   
//
//!  This function Tokenizes GPS string wrt to parsed delimeter 
//
//==============================================================================
static uint8_t TokenizeString(uint8_t *srcString, uint8_t dstToken[][25], uint8_t c_Delimiter, uint8_t messageLength)
{
  int j = 0;
  unsigned int i_Offset = 0;
  char b_Flag = 0;
  int count = 0;
  for (i_Offset = 0;i_Offset <= messageLength; i_Offset++)
  {
    // loop length controlers
    if(((count+1) >= (16)) || ((j+1) >= (25)))
    {
      //Buffer Overflow control
      break;
    }
    
    if (srcString[i_Offset] != c_Delimiter && srcString[i_Offset] != '\t' && srcString[i_Offset] != '\n' && srcString[i_Offset] != '\0')
    {
      dstToken[count][j] = srcString[i_Offset];
      j++;
      b_Flag = 1;
      continue;
    }
    if (b_Flag)
    {
      dstToken[count][j] = '\0';
      count++;
      j = 0;
      b_Flag = 0;
    }
  }
  return (count + 1);
}

//==============================================================================
//  GLOBAL FUNCTIONS IMPLEMENTATION
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
uint32_t CreateUARTTXdata(ATCOMMAND_INDEX_ENUM cmdIndex, uint8_t Buffer[], uint32_t buffSize)
{
    int32_t size = 0;
    switch(cmdIndex)
    {
    case ATC_UDCONF:
        size = snprintf((char *)Buffer, buffSize, "AT+UDCONF=7,%d,36\r\n\0", gCellularDriver.TCPSocket);
        break;
    case ATC_USOSEC:
        size = snprintf((char *)Buffer, buffSize, "AT+USOSEC=%d,1,0\r\n\0", gCellularDriver.TCPSocket);
        break;
        
    case ATC_USOCO:
        size = snprintf((char *)Buffer, buffSize, "AT+USOCO=%d,\"%s\",%d\r\n\0", gCellularDriver.TCPSocket, INET_HOST, INET_SSL_PORT);
        break;
        
    case AT_USODL:
        size = snprintf((char *)Buffer, buffSize, "AT+USODL=%d\r\n\0", gCellularDriver.TCPSocket);
        break;
        
    case ATC_USOCL:
        size = snprintf((char *)Buffer, buffSize, "\0");//"AT+USOCL=%d\r\n\0", gCellularDriver.TCPSocket);
        break;
    case ATC_USORD:
        size = snprintf((char *)Buffer, buffSize, "AT+USORD=%d,256\r\n\0", gCellularDriver.TCPSocket);
        break;
    default:
        // Invalid Request
        size = ERR_INVALID_AT_COMMAND;
        break;
    }
    return size;
}
