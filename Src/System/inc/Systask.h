//==============================================================================
//
//  Systask.h
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
//  Source:        Systask.h
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

#ifndef __SYSTASK_H

#define __SYSTASK_H

//==============================================================================
//  INCLUDES
//==============================================================================

#include <stdbool.h>
#include <stdint.h>
#include "Main.h"

//==============================================================================
//  GLOBAL CONSTANTS, TYPEDEFS AND MACROS 
//==============================================================================
#define FW_BUFFER_SIZE      64u
#define SIM_APN_LEN         64u
//==============================================================================
//  GLOBAL DATA STRUCTURES DEFINITION
//==============================================================================
typedef struct
{
    float longitude;
    uint8_t  longitudeDir;
    float latitude;
    uint8_t  latitudeDir;
    float horizantalDilution;
    uint8_t accuracy; 
    bool  isGpsValid;
}GPSInfo_t;

typedef struct
{
    uint8_t seconds;
    uint8_t minutes;
    uint8_t hours;
}TimeInfo_t;

typedef struct
{
    uint8_t day;
    uint8_t month;
    uint16_t year;
}DateInfo_t;

typedef struct
{
    TimeInfo_t time;
    DateInfo_t date;
}DateTimeInfo_t;

typedef enum
{
    
    EVENT_MANAGEMENT_EVENT_RECEVIED = 0,
    POWER_MANAGEMENT_EVENT_RECEVIED,
    WRITE_DATA_TO_FLASH,
    
    DEVICE_SHUTDOWN_MSG,
    
    INVALID_MSG_TYPE,
}SYS_MSG_ID_t;



typedef struct
{
    SYS_MSG_ID_t msgId;
    uint16_t msgInfo;
    void * ptrData;
}SysMsg_t;

typedef enum
{
    WIRELESS_DEVICE_DEFAULT =0,
    WIRELESS_DEVICE_WIFI,
    WIRELESS_DEVICE_CELLULAR
        
}WIRELESS_DEVICE_TYPE_t;

typedef enum
{
    
    UNKNOWN_FILE = 0,
    
    MAIN_FIRMWARE,
    CA_CERTIFICATE,
    HASH_KEY,
    SERVICE_PACK,
    
}FIRMWARE_FILE_TYPE_ENUM;

typedef enum
{
    
    NACK = 0x15,
    ACK  = 0x06,

}ACK_NACK_ENUM;

typedef enum
{
    
    NO_ERROR= 0,
    BAD_LENGTH,
    INVALID_CRC,
    BAD_FRAME_LENGTH,
    ALLOCATION_ERROR,
    DEVICE_NOT_SUPPORTED,

}FW_PACKET_ERROR_TYPE_ENUM;

typedef enum
{
    MAIN_HEADER_PACKET = 0,
    FILE_HEADER_PACKET,
    FW_DATA_PACKET,
    FW_DESCRIPTOR_PACKET,
    
} FIRMWARE_PACKET_TYPE_ENUM;

typedef struct
{    
    uint32_t  totalDataPackets;
    uint32_t  remainingDataPackets;
    uint32_t  previousSequenceNumber;
    uint32_t  currentSequenceNumber;
    uint32_t  totalFileSize;
    uint32_t  remainingFileSize;
    uint32_t  fileCRC;
    uint8_t   totalNumberOfFiles;
    
    uint8_t   fwBuffer[FW_BUFFER_SIZE];
    uint8_t   bufferDataSize;
    
    FIRMWARE_PACKET_TYPE_ENUM expectedPacketType;
    
    ACK_NACK_ENUM             returnMessage;
    WIRELESS_DEVICE_TYPE_t    deviceType;
    FIRMWARE_FILE_TYPE_ENUM   fileType;
    FW_PACKET_ERROR_TYPE_ENUM errorType;
    
}Firmware_File_t;

typedef struct
{
    uint32_t numberOfParameters;
    uint8_t instrumentSerialNumber[SERIAL_NUMBER_LENGTH + 1];
    uint8_t mfgDate[DATE_LENGTH + 1];
    uint8_t partNumber[PART_NUMBER_LENGTH + 1];
    uint8_t techInitials[TECH_INITIALS_LENGTH + 1];
    uint8_t jobNumber[JOB_NUMBER_LENGTH + 1];
    uint8_t simApn[SIM_APN_LEN + 1];
}
Device_Parameters_t;

//==============================================================================
//  GLOBAL DATA
//==============================================================================
extern Firmware_File_t fwFile;
extern Device_Parameters_t deviceParams;
//==============================================================================
//  EXTERNAL OR GLOBAL FUNCTIONS
//==============================================================================
void SysTask(void *arg);


#endif /* __SYSTASK_H */