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
//  Source:        DataFlash.h
//
//  Project:       Frey
//
//  Author:        Dilawar Ali
//
//  Date:          2018/10/01
//
//  Revision:      1.0
//
//==============================================================================
//  FILE DESCRIPTION
//==============================================================================
//
//! \file
//! This file contains the prototypes of global functions and declaration of global 
//! data used to use the DataFlash. This is hardware driver module for DataFlash.
//

#ifndef DATAFLASH_H
#define DATAFLASH_H

//==============================================================================
//  INCLUDES
//==============================================================================
#include <stdint.h>

#include <main.h>
//==============================================================================
//  GLOBAL CONSTANTS, TYPEDEFS AND MACROS 
//==============================================================================


/*
DataFlash Memory devisions:

Sector 0a  | Block 0      | pages 0-7      | Descriptor Writing
           |              |                |
Sector 0b  | Block 1-31   | pages 8-255    | Parameters Writing
           |              |                |
Sector 1-2 | Block 32-95  | pages 256-767  | Firmware Writing
           |              |                |
Sector 3-7 | Block 96-255 | pages 767-2047 | For Future Use
*/



#define DATAFLASH_SRAM_BUFFER_SIZE   256u
#define DATAFLASH_BYTES_PER_PAGE     256u

#define DATAFLASH_TOTAL_PAGES    2047u
#define DATAFLASH_TOTAL_BLOCKS   255u
#define DATAFLASH_TOTAL_SECTORS  7u

//---------------------- DataFlash Error Codes ----------------------------------

#define ERR_DATAFLASH_PAGE_INVALID            (-101)
#define ERR_DATAFLASH_BLOCK_INVALID           (-102)
#define ERR_DATAFLASH_SECTOR_INVALID          (-103)
#define ERR_DATAFLASH_SPI_TRANSACTION_FAILED  (-104)
#define ERR_DATAFLASH_EPE_INTERNAL_ERROR      (-105)
#define ERR_DATAFLASH_BUFF_SIZE_INVALID       (-106)


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
//  int32_t DataFlashInit(void)
//
//   Author:  Dilawar Ali
//   Date:    2018/10/01
//
//!  This function initialize the SPI and dataflash module
//
//------------------------------------------------------------------------------
int32_t DataFlashInit(void);

//------------------------------------------------------------------------------
//  int32_t DataFlashWriteBuffer(uint8_t startIndex, uint8_t data[], uint32_t size)
//
//   Author:  Dilawar Ali
//   Date:    2018/10/01
//
//!  This function Write the given bytes into the data flash fd defined sector/block/page
//
//------------------------------------------------------------------------------
int32_t DataFlashWriteBuffer(uint8_t startIndex, uint8_t data[], uint32_t size);

//------------------------------------------------------------------------------
//  int32_t DataFlashWriteBufferToPage(uint16_t pageNumber)
//
//   Author:  Dilawar Ali
//   Date:    2018/10/02
//
//!  This function Write the data flash buffer data to defined page number
//
//------------------------------------------------------------------------------
int32_t DataFlashWriteBufferToPage(uint16_t pageNumber);

//------------------------------------------------------------------------------
//  int32_t DataFlashErasePage (uint16_t pageNumber)
//
//   Author:  Dilawar Ali
//   Date:    2018/10/01
//
//!  This function erase the specified Page
//
//------------------------------------------------------------------------------
int32_t DataFlashErasePage (uint16_t pageNumber);

//------------------------------------------------------------------------------
//  int32_t DataFlashEraseBlock (uint16_t blockNumber)
//
//   Author:  Dilawar Ali
//   Date:    2018/10/01
//
//!  This function erase the specified Block
//
//------------------------------------------------------------------------------
int32_t DataFlashEraseBlock (uint32_t blockNumber);

//------------------------------------------------------------------------------
//  int32_t DataFlashEraseSector (uint8_t sectorNumber, BOOLEAN isSector0b)
//
//   Author:  Dilawar Ali
//   Date:    2018/10/01
//
//!  This function erase the specified Sector
//
//------------------------------------------------------------------------------
int32_t DataFlashEraseSector (uint8_t sectorNumber, BOOLEAN isSector0b);

//------------------------------------------------------------------------------
//  int32_t DataFlashReadPage (uint16_t pageNumber)
//
//   Author:  Dilawar Ali
//   Date:    2018/10/01
//
//!  This function read the specified Page from main memory
//
//------------------------------------------------------------------------------
int32_t DataFlashReadPage (uint16_t pageNumber, uint8_t dstBuffer[], uint32_t size);

//------------------------------------------------------------------------------
//  int32_t DataFlashEnablePowerSaving (void)
//
//   Author:  Dilawar Ali
//   Date:    2018/10/17
//
//!  This function set data flash to ultra deep power saving mode
//
//------------------------------------------------------------------------------
int32_t DataFlashEnablePowerSaving (void);

//------------------------------------------------------------------------------
//  int32_t DataFlashDisablePowerSaving (void)
//
//   Author:  Dilawar Ali
//   Date:    2018/10/17
//
//!  This function set data flash to stand by mode
//
//------------------------------------------------------------------------------
int32_t DataFlashDisablePowerSaving (void);

#endif