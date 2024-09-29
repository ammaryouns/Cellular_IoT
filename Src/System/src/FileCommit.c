//==============================================================================
//
//  FileCommit.c
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
//  Source:        FileCommit.c
//
//  Project:       Frey
//
//  Author:        Dilawar Ali
//
//  Date:          2018/10/03
//
//  Revision:      1.0
//
//==============================================================================
//  FILE DESCRIPTION
//==============================================================================
//
//! \file
//! This file contains the prototypes of global functions and declaration of global 
//! data used To write data to DataFlash. This module provides support for reading
//! writing firmware and other files to data flash
//


//==============================================================================
//  INCLUDES
//==============================================================================
#include "FileCommit.h"
#include <stdio.h>
#include <stdlib.h>

#include "SysTask.h"
#include "jsmn.h"

//==============================================================================
//  CONSTANTS, TYPEDEFS AND MACROS 
//==============================================================================
#define PARAMS_JASON_DATA_BUFF_LEN      256
#define PARAMS_JSON_MAX_TOKENS          16
//==============================================================================
//  LOCAL DATA DECLARATIONS
//==============================================================================

//==============================================================================
//  LOCAL FUNCTION PROTOTYPES
//==============================================================================


//==============================================================================
//  GLOBAL DATA DECLARATIONS
//==============================================================================


//==============================================================================
//  LOCAL FUNCTIONS IMPLEMENTATION
//==============================================================================

//==============================================================================
//  Global FUNCTIONS IMPLEMENTATION
//==============================================================================

//------------------------------------------------------------------------------
//  int32_t FileCommitWriteFileToFlash(FILE_DESCRIPTOR_STRUCT *fd, uint8_t firmwareBuf[], uint16_t size)
//
//   Author:  Dilawar Ali
//   Date:    2018/10/03
//
//!  This function Write Firmware Data to Flash
//
//------------------------------------------------------------------------------
int32_t FileCommitWriteFileToFlash(FILE_DESCRIPTOR_STRUCT *fd, uint8_t firmwareBuf[], uint16_t size)
{
    int32_t ret = 0;
    uint32_t bufferWriteSize = 0;
    uint32_t bufferIndex = 0;
    uint8_t loopCounter = 0;
    
    switch (fd->fileAction)
    {
    case ERASE_FILE_SECTORS:

        for(loopCounter = 0; ((loopCounter < fd->fileNumerOfSectors) && (loopCounter <= DATAFLASH_TOTAL_SECTORS)); loopCounter++)
        {
            ret = DataFlashEraseSector((loopCounter + fd->fileStartSector), false);
            
            if((loopCounter + fd->fileStartSector) == 0)
            {
                ret = DataFlashEraseSector((loopCounter + fd->fileStartSector), true);
            }
        }
        break;
        
    case WRITE_FILE_PACKET:
        while (size > 0)
        {
            bufferWriteSize = DATAFLASH_SRAM_BUFFER_SIZE - fd->DataFlashBuffDataSize;
            bufferWriteSize = FIND_MIN(bufferWriteSize, size);
            
            ret = DataFlashWriteBuffer(fd->DataFlashBuffDataSize, &firmwareBuf[bufferIndex], bufferWriteSize);
            if(ret >= 0)
            {
                size -= bufferWriteSize;
                bufferIndex += bufferWriteSize;
                fd->DataFlashBuffDataSize += bufferWriteSize;
                
                if(fd->DataFlashBuffDataSize == DATAFLASH_SRAM_BUFFER_SIZE)
                {
                    ret = DataFlashWriteBufferToPage(fd->lastPageNumber);
                    fd->lastPageNumber++;
                    fd->DataFlashBuffDataSize = 0;
                }
            }
            else
            {
                break;
            }
        }
        if(ret >= 0)
        {
            fd->FileRemainingSize -= size;
            if((fd->FileRemainingSize == 0) && (fd->DataFlashBuffDataSize > 0))
            {
                ret = DataFlashWriteBufferToPage(fd->lastPageNumber); 
                fd->DataFlashBuffDataSize = 0;
            }
        }
        break;
        
    case WRITE_DESCRIPTOR:
        ret = DataFlashErasePage(FIRMWARE_DESCRIPTOR_PAGE_NUMBER);
        if(ret >= 0)
        {
            ret = DataFlashWriteBuffer(fd->DataFlashBuffDataSize, firmwareBuf, bufferWriteSize);
            if(ret >= 0)
            {
                ret = DataFlashWriteBufferToPage(fd->lastPageNumber); 
            }
        }
        break;
        
    default:
        break;
    }
    
    return ret;
}

//------------------------------------------------------------------------------
//  int32_t FileCommitParseReceivedFwFilePacket(FILE_DESCRIPTOR_STRUCT *fd, uint8_t dataBuff, uint32_t dataSize)
//
//   Author:  Dilawar Ali
//   Date:    2018/10/04
//
//!  This function Parse firmware packet received from instrument
//
//------------------------------------------------------------------------------
int32_t FileCommitParseReceivedFwFilePacket(FILE_DESCRIPTOR_STRUCT *fd, Firmware_File_t *file)
{
    int8_t loopCounter = 0;
    int32_t ret = 0;
    FIRMWARE_PACKET_TYPE_ENUM fwPacketType = MAIN_HEADER_PACKET;
    
    if(((file->fwBuffer[0] == 'M') && (file->fwBuffer[1] == 'H')) && (file->expectedPacketType == MAIN_HEADER_PACKET))
    {
        fwPacketType = MAIN_HEADER_PACKET;
        file->expectedPacketType = FILE_HEADER_PACKET;
    }
    else if(((file->fwBuffer[0] == 'F') && (file->fwBuffer[1] == 'H')) && (file->expectedPacketType == FILE_HEADER_PACKET))
    {
        fwPacketType = FILE_HEADER_PACKET;
        file->expectedPacketType = FW_DATA_PACKET;
    }
    else if(((file->fwBuffer[0] == 'D') && (file->fwBuffer[1] == 'S')) && ((fd->FileRemainingSize - fd->DataFlashBuffDataSize) == 0))
    {
        fwPacketType = FW_DESCRIPTOR_PACKET;
        file->expectedPacketType = FILE_HEADER_PACKET;
    }
    else if(file->expectedPacketType == FW_DATA_PACKET)
    {
        fwPacketType = FW_DATA_PACKET;
        file->expectedPacketType = FW_DATA_PACKET;
    }
    else
    {
        //Run, Instrument is on Fire
        ret = ERR_FILE_PACKET_TYPE_INVALID;
    }
    
    DataFlashDisablePowerSaving();
    
    switch(fwPacketType)
    {
    case MAIN_HEADER_PACKET:
        if(file->fwBuffer[2] == WIRELESS_DEVICE_CELLULAR)
        {
            file->totalNumberOfFiles = file->fwBuffer[3];
            file->totalFileSize      = ((file->fwBuffer[4] << 0) || (file->fwBuffer[5] << 8) || (file->fwBuffer[6] << 16) || (file->fwBuffer[7] << 24));
        }
        else
        {
            ret = ERR_FILE_DEVICE_TYPE_NOT_SUPPORTED;
        }
        break;
        
    case FILE_HEADER_PACKET:
        switch(file->fwBuffer[2])
        {
        case MAIN_FIRMWARE:
            fd->DataFlashBuffDataSize = 0;
            fd->fileAction            = ERASE_FILE_SECTORS;
            fd->fileNumerOfSectors    = FIRMWARE_NUMBER_OF_SECTORS;
            fd->fileStartSector       = FIRMWARE_START_SECTOR_NUMBER;
            fd->lastPageNumber        = FIRMWARE_FIRST_PAGE_NUMBER;
            fd->FileTotalSize         = ((file->fwBuffer[3] << 0) || (file->fwBuffer[4] << 8) || (file->fwBuffer[5] << 16) || (file->fwBuffer[6] << 24));
            fd->FileRemainingSize     = fd->FileTotalSize;
            
            
            FileCommitWriteFileToFlash(fd, file->fwBuffer, file->bufferDataSize);
            break;
            
        default:
            // You are not working right since you had a electric shock!
            ret = ERR_FILE_TYPE_NOT_SUPPORTED;
        }
        break;
        
    case FW_DATA_PACKET:
        fd->fileAction = WRITE_FILE_PACKET;
        FileCommitWriteFileToFlash(fd, file->fwBuffer, file->bufferDataSize);
        break;
        
    case FW_DESCRIPTOR_PACKET:
        file->fwBuffer[file->bufferDataSize++] = (uint8_t)(fd->FileTotalSize >> 0);
        file->fwBuffer[file->bufferDataSize++] = (uint8_t)(fd->FileTotalSize >> 8);
        file->fwBuffer[file->bufferDataSize++] = (uint8_t)(fd->FileTotalSize >> 16);
        file->fwBuffer[file->bufferDataSize++] = (uint8_t)(fd->FileTotalSize >> 24);
        
        for(loopCounter = file->bufferDataSize; loopCounter > 0; loopCounter--)
        {
            file->fwBuffer[loopCounter] = file->fwBuffer[loopCounter - 1];
        }
        file->fwBuffer[0] = file->bufferDataSize;
        
        fd->fileAction = WRITE_DESCRIPTOR;
        FileCommitWriteFileToFlash(fd, file->fwBuffer, file->bufferDataSize);
        
        break;
    }
    
    DataFlashEnablePowerSaving();
    
    return ret;
}


//------------------------------------------------------------------------------
//  int32_t SaveCurrentDeviceParameters(Device_Parameters_t *params)
//
//   Author:  Dilawar Ali
//   Date:    2018/10/31
//
//!  This function write the existing parameters to data flash
//
//------------------------------------------------------------------------------
int32_t SaveCurrentDeviceParameters(Device_Parameters_t *params)
{
    int32_t ret = 0;
    uint8_t paramsJasonDataBuffer[PARAMS_JASON_DATA_BUFF_LEN] = {0};
    uint16_t jasonSize = 0;
    
    uint32_t paramsPageNumber = PARAMS_START_PAGE_NUMBER;
    
    // Create the Jason struture of params
    jasonSize += snprintf((char*)&paramsJasonDataBuffer[jasonSize], (PARAMS_JASON_DATA_BUFF_LEN - jasonSize), "{");
    jasonSize += snprintf((char*)&paramsJasonDataBuffer[jasonSize], (PARAMS_JASON_DATA_BUFF_LEN - jasonSize), "\"NOP\":%d,", params->numberOfParameters);
    jasonSize += snprintf((char*)&paramsJasonDataBuffer[jasonSize], (PARAMS_JASON_DATA_BUFF_LEN - jasonSize), "\"SimApn\":\"%s\",", params->simApn);
    jasonSize += snprintf((char*)&paramsJasonDataBuffer[jasonSize], (PARAMS_JASON_DATA_BUFF_LEN - jasonSize), "\"SN\":\"%s\",", params->instrumentSerialNumber);
    jasonSize += snprintf((char*)&paramsJasonDataBuffer[jasonSize], (PARAMS_JASON_DATA_BUFF_LEN - jasonSize), "\"JN\":\"%s\",", params->jobNumber);
    jasonSize += snprintf((char*)&paramsJasonDataBuffer[jasonSize], (PARAMS_JASON_DATA_BUFF_LEN - jasonSize), "\"mfgDate\":\"%s\",", params->mfgDate);
    jasonSize += snprintf((char*)&paramsJasonDataBuffer[jasonSize], (PARAMS_JASON_DATA_BUFF_LEN - jasonSize), "\"PN\":\"%s\",", params->partNumber);
    jasonSize += snprintf((char*)&paramsJasonDataBuffer[jasonSize], (PARAMS_JASON_DATA_BUFF_LEN - jasonSize), "\"TI\":\"%s\",", params->techInitials);
    jasonSize += snprintf((char*)&paramsJasonDataBuffer[jasonSize], (PARAMS_JASON_DATA_BUFF_LEN - jasonSize), "}\0");
    
    // Erase the Parameter Sector
    ret = DataFlashEraseSector(PARAMS_SECTOR, true);
    if(ret >= 0)
    {
        ret = DataFlashWriteBuffer(0, paramsJasonDataBuffer, (jasonSize + 1));
        if(ret >= 0)
        {
            ret = DataFlashWriteBufferToPage(paramsPageNumber);
        }
    }
    
    return ret;
}


//------------------------------------------------------------------------------
//  int32_t GetDeviceParamsFromFlash(Device_Parameters_t *params)
//
//   Author:  Dilawar Ali
//   Date:    2018/10/31
//
//!  This function update the device parameters from stored parameters in flash
//
//------------------------------------------------------------------------------
int32_t GetDeviceParamsFromFlash(Device_Parameters_t *params)
{
    int32_t ret = 0;
    uint32_t size = 0, len = 0;
    jsmn_parser js;
    jsmntok_t js_tok[PARAMS_JSON_MAX_TOKENS];
    
    uint8_t paramsJasonDataBuffer[PARAMS_JASON_DATA_BUFF_LEN + 1] = {0};
    uint32_t paramsPageNumber = PARAMS_START_PAGE_NUMBER;
    
    ret = DataFlashReadPage(paramsPageNumber, paramsJasonDataBuffer, DATAFLASH_BYTES_PER_PAGE);
    if(ret >= 0)
    {
        jsmn_init(&js);
        len = strlen((char const*)paramsJasonDataBuffer);
        
        int numOfToken=(sizeof(js_tok)/sizeof(js_tok[0]));
        
        ret = jsmn_parse(&js, (char const*)paramsJasonDataBuffer, len, js_tok, numOfToken);
        if(ret > 0)
        {
            for(uint32_t i=1; i < ret; ++i)
            {
                size = js_tok[i].end - js_tok[i].start;
                if((0 == strncmp((char const*)&paramsJasonDataBuffer[js_tok[i].start], "NOP", size)) && (size != 0))
                {
                    size = js_tok[i+1].end - js_tok[i+1].start;
                    params->numberOfParameters = atoi((char *)&paramsJasonDataBuffer[js_tok[i+1].start]);
                }
                else if (( 0 == strncmp((char const*)&paramsJasonDataBuffer[js_tok[i].start], "SimApn", size)) && (size != 0))
                {
                    size = js_tok[i+1].end - js_tok[i+1].start;
                    sprintf((char *)params->simApn, "%.*s" ,size, (char *)&paramsJasonDataBuffer[js_tok[i+1].start]);                    
                }
                else if (( 0 == strncmp((char const*)&paramsJasonDataBuffer[js_tok[i].start], "SN", size)) && (size != 0))
                {
                    size = js_tok[i+1].end - js_tok[i+1].start;
                    sprintf((char *)params->instrumentSerialNumber, "%.*s" ,size, (char *)&paramsJasonDataBuffer[js_tok[i+1].start]);                    
                }
                else if (( 0 == strncmp((char const*)&paramsJasonDataBuffer[js_tok[i].start], "JN", size)) && (size != 0))
                {
                    size = js_tok[i+1].end - js_tok[i+1].start;
                    sprintf((char *)params->jobNumber, "%.*s" ,size, (char *)&paramsJasonDataBuffer[js_tok[i+1].start]);                    
                }
                else if (( 0 == strncmp((char const*)&paramsJasonDataBuffer[js_tok[i].start], "mfgDate", size)) && (size != 0))
                {
                    size = js_tok[i+1].end - js_tok[i+1].start;
                    sprintf((char *)params->mfgDate, "%.*s" ,size, (char *)&paramsJasonDataBuffer[js_tok[i+1].start]);                    
                }
                else if (( 0 == strncmp((char const*)&paramsJasonDataBuffer[js_tok[i].start], "PN", size)) && (size != 0))
                {
                    size = js_tok[i+1].end - js_tok[i+1].start;
                    sprintf((char *)params->partNumber, "%.*s" ,size, (char *)&paramsJasonDataBuffer[js_tok[i+1].start]);                    
                }
                else if (( 0 == strncmp((char const*)&paramsJasonDataBuffer[js_tok[i].start], "TI", size)) && (size != 0))
                {
                    size = js_tok[i+1].end - js_tok[i+1].start;
                    sprintf((char *)params->techInitials, "%.*s" ,size, (char *)&paramsJasonDataBuffer[js_tok[i+1].start]);                    
                }
                else
                {
                    //Do Nothing
                }
            }
        }
        else
        {
            ret = ERR_PARAMS_JSON_PARSER_FAILED;
        }
    }
    
    return ret;
}
//==============================================================================
//  End Of File
//==============================================================================