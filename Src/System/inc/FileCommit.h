//==============================================================================
//
//  FileCommit.h
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
//  Source:        FileCommit.h
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


#ifndef FILECOMMIT_H
#define FILECOMMIT_H

//==============================================================================
//  INCLUDES
//==============================================================================
#include <stdint.h>
#include <main.h>
#include "DataFlash.h"
#include "SysTask.h"
//==============================================================================
//  GLOBAL CONSTANTS, TYPEDEFS AND MACROS 
//==============================================================================
typedef enum
{
    ERASE_FILE_SECTORS = 0,
    WRITE_FILE_PACKET,
    WRITE_DESCRIPTOR,
    
}INSTRUCTION_ACTION_TYPE_ENUM;


typedef struct
{
    INSTRUCTION_ACTION_TYPE_ENUM fileAction;
    uint8_t  fileNumerOfSectors;
    uint8_t  fileStartSector;
    
    uint16_t lastPageNumber;
    uint16_t DataFlashBuffDataSize;
    
    uint32_t FileTotalSize;
    uint32_t FileRemainingSize;
    
}FILE_DESCRIPTOR_STRUCT;

//------------ Files Descriptor Page -------------------------------------------
#define FIRMWARE_DESCRIPTOR_PAGE_NUMBER  0u

//------------ Firmware File Pages ---------------------------------------------
#define FIRMWARE_FIRST_PAGE_NUMBER  256u
#define FIRMWARE_LAST_PAGE_NUMBER   767u

//------------ Firmware File sectors -------------------------------------------
#define FIRMWARE_START_SECTOR_NUMBER 1u
#define FIRMWARE_NUMBER_OF_SECTORS   2u

//------------ Params Descriptor Pages -----------------------------------------
#define PARAMS_START_PAGE_NUMBER    8
#define PARAMS_LAST_PAGE_NUMBER     255
//------------ Params Descriptor Sectors ---------------------------------------
#define PARAMS_SECTOR               0

// --------------- File Commit Error Codes -------------------------------------
#define ERR_FILE_PACKET_TYPE_INVALID        (-150)
#define ERR_FILE_DEVICE_TYPE_NOT_SUPPORTED  (-151)
#define ERR_FILE_TYPE_NOT_SUPPORTED         (-152)
#define ERR_PARAMS_JSON_PARSER_FAILED       (-153)
//---------------------- File Commit Error Codes -------------------------------

//#define ERR_DATAFLASH_PAGE_INVALID            (-201)
//#define ERR_DATAFLASH_BLOCK_INVALID           (-202)
//#define ERR_DATAFLASH_SECTOR_INVALID          (-203)
//#define ERR_DATAFLASH_SPI_TRANSACTION_FAILED  (-204)
//#define ERR_DATAFLASH_EPE_INTERNAL_ERROR      (-205)
//#define ERR_DATAFLASH_READ_BUFF_SIZE_INVALID  (-206)

//==============================================================================
//  GLOBAL DATA STRUCTURES DEFINITION
//==============================================================================

//==============================================================================
//  GLOBAL DATA
//==============================================================================


//==============================================================================
//  EXTERNAL OR GLOBAL FUNCTIONS
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
int32_t FileCommitWriteFileToFlash(FILE_DESCRIPTOR_STRUCT *fd, uint8_t firmwareBuf[], uint16_t size);

//------------------------------------------------------------------------------
//  int32_t FileCommitParseReceivedFwFilePacket(FILE_DESCRIPTOR_STRUCT *fd, uint8_t dataBuff, uint32_t dataSize)
//
//   Author:  Dilawar Ali
//   Date:    2018/10/04
//
//!  This function Parse firmware packet received from instrument
//
//------------------------------------------------------------------------------
int32_t FileCommitParseReceivedFwFilePacket(FILE_DESCRIPTOR_STRUCT *fd, Firmware_File_t *file);

//------------------------------------------------------------------------------
//  int32_t GetDeviceParamsFromFlash(Device_Parameters_t *params)
//
//   Author:  Dilawar Ali
//   Date:    2018/10/31
//
//!  This function update the device parameters from stored parameters in flash
//
//------------------------------------------------------------------------------
int32_t GetDeviceParamsFromFlash(Device_Parameters_t *params);

//------------------------------------------------------------------------------
//  int32_t SaveCurrentDeviceParameters(Device_Parameters_t *params)
//
//   Author:  Dilawar Ali
//   Date:    2018/10/31
//
//!  This function write the existing parameters to data flash
//
//------------------------------------------------------------------------------
int32_t SaveCurrentDeviceParameters(Device_Parameters_t *params);
#endif