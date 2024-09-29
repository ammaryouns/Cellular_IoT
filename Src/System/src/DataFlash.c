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
//  Source:        DataFlash.c
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


//==============================================================================
//  INCLUDES
//==============================================================================
#include "DataFlash.h"
#include <em_gpio.h>

#include "SPI_Comm.h"

//==============================================================================
//  CONSTANTS, TYPEDEFS AND MACROS 
//==============================================================================
#define DATAFLASH_COMMAND_BUFFER_SIZE  264u
#define DATAFLASH_RESPONSE_BUFFER_SIZE 264u


#define DATAFLASH_SPI_BITRATE    5000000u
#define DATAFLASH_SPI_TIMEOUT    3000u

#define DATAFLASH_READY_MASK   0x8080

#define DATAFLASH_EPE_MASK     0x0020


//------ Data Flash OP Codes ---------------------------------------------------
#define STATUS_READ_OPCODE      0xD7
#define PAGE_ERASE_OPCODE       0x81
#define BLOCK_ERASE_OPCODE      0x50
#define SECTOR_ERASE_OPCODE     0x7C

#define BUFFER1_WRITE_OPCODE        0x84
#define BUFFER1_TO_MEM_WRITE_OPCODE 0x88

#define MAIN_MEMORY_PAGE_READ_OPCODE    0xD2

#define ULTRA_DEEP_POWER_DOWN_OPCODE    0x79
//==============================================================================
//  LOCAL DATA DECLARATIONS
//==============================================================================
static uint8_t dataFlashCommandBuffer[DATAFLASH_COMMAND_BUFFER_SIZE];
static uint8_t dataFlashResponseBuffer[DATAFLASH_RESPONSE_BUFFER_SIZE];

//==============================================================================
//  LOCAL FUNCTION PROTOTYPES
//==============================================================================
static int32_t WaitDataFlashToGetReady (void);
static int32_t DataFlashReadTransactionStatus(void);
static int32_t DataFlashSpiTransfer(uint8_t writeBuff[], uint8_t readBuff[], uint32_t size);
static int32_t DataFlashSpiWrite(uint8_t writeBuff[], uint32_t size);
static int32_t DataFlashSpiRead(uint8_t readBuff[], uint32_t size);
static void ReadDataFlashStatus(uint16_t *status);


//==============================================================================
//  GLOBAL DATA DECLARATIONS
//==============================================================================
SPIDRV_HandleData_t dataFlashSPIHandleData;
SPIDRV_Handle_t dataFlashSPIHandle = &dataFlashSPIHandleData;


//==============================================================================
//  LOCAL FUNCTIONS IMPLEMENTATION
//==============================================================================

//------------------------------------------------------------------------------
//  static int32_t WaitDataFlashToGetReady (void)
//
//   Author:  Dilawar Ali
//   Date:    2018/10/01
//
//!  This function read the data flash status to check is device bussy in operations
//
//------------------------------------------------------------------------------
static int32_t WaitDataFlashToGetReady (void)
{
    int32_t ret = 0;
    uint16_t dataFlashStatus = 0;
    while((dataFlashStatus & DATAFLASH_READY_MASK) != DATAFLASH_READY_MASK)
    {
        ReadDataFlashStatus(&dataFlashStatus);
        __asm("nop");
        __asm("nop");
        __asm("nop");
    }
    
    return ret;
}

//------------------------------------------------------------------------------
//  int32_t DataFlashSpiTransfer(uint8_t writeBuff[], uint8_t readBuff[], uint32_t size)
//
//   Author:  Dilawar Ali
//   Date:    2018/10/01
//
//!  This function write the data to flash and read response
//
//------------------------------------------------------------------------------
static int32_t DataFlashSpiTransfer(uint8_t writeBuff[], uint8_t readBuff[], uint32_t size)
{
    int32_t ret = 0;
    if(size > 0)
    {
        ret = SPIDRV_MTransferB(dataFlashSPIHandle, writeBuff, readBuff, size);
        if(ret != 0)
        {
            ret = ERR_DATAFLASH_SPI_TRANSACTION_FAILED;
        }
    }
    return ret;
}


//------------------------------------------------------------------------------
//  int32_t DataFlashSpiWrite(uint8_t writeBuff[], uint32_t size)
//
//   Author:  Dilawar Ali
//   Date:    2018/10/01
//
//!  This function write the data to flash
//
//------------------------------------------------------------------------------
static int32_t DataFlashSpiWrite(uint8_t writeBuff[], uint32_t size)
{
    int32_t ret = 0;
    if(size > 0)
    {
        
        ret = SPIDRV_MTransmitB(dataFlashSPIHandle, writeBuff, size);
        if(ret != 0)
        {
            ret = ERR_DATAFLASH_SPI_TRANSACTION_FAILED;
        }
    }
    return ret;
}

//------------------------------------------------------------------------------
//  int32_t DataFlashSpiRead(uint8_t readBuff[], uint32_t size)
//
//   Author:  Dilawar Ali
//   Date:    2018/10/01
//
//!  This function read response from data flash
//
//------------------------------------------------------------------------------
static int32_t DataFlashSpiRead(uint8_t readBuff[], uint32_t size)
{
    int32_t ret = 0;
    if(size > 0)
    {
        ret = SPIDRV_MReceiveB(dataFlashSPIHandle, readBuff, size);
        if(ret != 0)
        {
            ret = ERR_DATAFLASH_SPI_TRANSACTION_FAILED;
        }
    }
    return ret;
}


//------------------------------------------------------------------------------
//  void ReadDataFlashStatus(uint16_t *status)
//
//   Author:  Dilawar Ali
//   Date:    2018/10/01
//
//!  This function return the ready/busy status of the data Flash
//
//------------------------------------------------------------------------------
static void ReadDataFlashStatus(uint16_t *status)
{
    int32_t ret = 0;
    dataFlashCommandBuffer[0] = STATUS_READ_OPCODE;
    dataFlashCommandBuffer[1] = 0;
    dataFlashCommandBuffer[2] = 0;

    ret = DataFlashSpiTransfer(dataFlashCommandBuffer, dataFlashResponseBuffer, 3u);

    if(ret != 0)
    {
        *status = 0;
    }
    else
    {
        *status = ((dataFlashResponseBuffer[1] << 8) | (dataFlashResponseBuffer[2]));
    }
}



//------------------------------------------------------------------------------
//  int32_t  DataFlashReadTransactionStatus(void)
//
//   Author:  Dilawar Ali
//   Date:    2018/10/01
//
//!  This function return the transaction status of the data Flash
//
//------------------------------------------------------------------------------
static int32_t  DataFlashReadTransactionStatus(void)
{
    int32_t ret = 0;
    uint16_t dataFlashStatus = 0u;
    ReadDataFlashStatus(&dataFlashStatus);
    if(((dataFlashStatus & DATAFLASH_EPE_MASK) == DATAFLASH_EPE_MASK) && (dataFlashStatus != 0u))
    {
        ret = ERR_DATAFLASH_EPE_INTERNAL_ERROR;
    }
    return ret;
}


//==============================================================================
//  Global FUNCTIONS IMPLEMENTATION
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
int32_t DataFlashInit(void)
{
    int32_t ret = 0;
    SPIDRV_Init_t initData =
    {
        USART1,                     /* USART port                       */ 
        _USART_ROUTELOC0_TXLOC_LOC0, /* USART Tx pin location number    */ 
        _USART_ROUTELOC0_RXLOC_LOC0, /* USART Rx pin location number    */ 
        _USART_ROUTELOC0_CLKLOC_LOC0, /* USART Clk pin location number  */ 
        _USART_ROUTELOC0_CSLOC_LOC0, /* USART Cs pin location number    */ 
        DATAFLASH_SPI_BITRATE,      /* Bitrate                          */ 
        8,                          /* Frame length                     */ 
        0,                          /* Dummy Tx value for Rx only funcs */ 
        spidrvMaster,                /* SPI mode                         */ 
        spidrvBitOrderMsbFirst,     /* Bit order on bus                 */ 
        spidrvClockMode0,           /* SPI clock/phase mode             */ 
        spidrvCsControlAuto, /* CS controlled by the Application */ 
        spidrvSlaveStartImmediate   /* Slave start transfers immediately*/ 
    };
    
    GPIO_PinModeSet(gpioPortB, 13, gpioModeInputPull, 0);
    // Initialize an SPI driver instance.
    SPIDRV_Init(dataFlashSPIHandle, &initData);
    if(dataFlashSPIHandle == NULL)
    {
        ret = -1;
    }
    else
    {
        GPIO_PinModeSet(gpioPortB, 13, gpioModeInputPull, 1);
    }
    
    return ret;
}


//------------------------------------------------------------------------------
//  int32_t DataFlashWriteBuffer(uint8_t startIndex, uint8_t data[], uint32_t size)
//
//   Author:  Dilawar Ali
//   Date:    2018/10/01
//
//!  This function Write the given bytes into the data flash fd defined sector/block/page
//
//------------------------------------------------------------------------------
int32_t DataFlashWriteBuffer(uint8_t startIndex, uint8_t data[], uint32_t size)
{
    int32_t ret = 0;
    
    if((size > 0) && (size <= DATAFLASH_SRAM_BUFFER_SIZE))
    {
        ret = WaitDataFlashToGetReady();
        if(ret >= 0)
        {
            dataFlashCommandBuffer[0] = BUFFER1_WRITE_OPCODE;
            dataFlashCommandBuffer[1] = 0;
            dataFlashCommandBuffer[2] = 0;
            dataFlashCommandBuffer[3] = startIndex;
            
            memcpy(&dataFlashCommandBuffer[4], data, size);
            
            
            ret = DataFlashSpiTransfer(dataFlashCommandBuffer, dataFlashResponseBuffer, (size +4u));
            if(ret >= 0)
            {
                ret = WaitDataFlashToGetReady();
                if(ret >= 0)
                {
                    ret = DataFlashReadTransactionStatus();
                }   
            }
        }
    }
    else
    {
        ret = ERR_DATAFLASH_BUFF_SIZE_INVALID;
    }
    return ret;
}

//------------------------------------------------------------------------------
//  int32_t DataFlashWriteBufferToPage(uint16_t pageNumber)
//
//   Author:  Dilawar Ali
//   Date:    2018/10/02
//
//!  This function Write the data flash buffer data to defined page number
//
//------------------------------------------------------------------------------
int32_t DataFlashWriteBufferToPage(uint16_t pageNumber)
{
    int32_t ret = 0;
    if(pageNumber > DATAFLASH_TOTAL_PAGES)
    {
        ret = ERR_DATAFLASH_PAGE_INVALID;
    }
    else
    {
        dataFlashCommandBuffer[0] = BUFFER1_TO_MEM_WRITE_OPCODE;
        dataFlashCommandBuffer[1] = (uint8_t)(pageNumber >> 8u);
        dataFlashCommandBuffer[2] = (uint8_t)(pageNumber);
        dataFlashCommandBuffer[3] = 0;
        ret = DataFlashSpiWrite(dataFlashCommandBuffer, 4u);
        if(ret >= 0)
        {
            ret = WaitDataFlashToGetReady();
            if(ret >= 0)
            {
                ret = DataFlashReadTransactionStatus();
            }
            
        }
    }
    return ret;
}

//------------------------------------------------------------------------------
//  int32_t DataFlashErasePage (uint16_t pageNumber)
//
//   Author:  Dilawar Ali
//   Date:    2018/10/01
//
//!  This function erase the specified Page
//
//------------------------------------------------------------------------------
int32_t DataFlashErasePage (uint16_t pageNumber)
{
    int32_t ret = 0;
    
    if(pageNumber > DATAFLASH_TOTAL_PAGES)
    {
        ret = ERR_DATAFLASH_PAGE_INVALID;
    }
    else
    {
        ret = WaitDataFlashToGetReady();
        if(ret >= 0)
        {
            dataFlashCommandBuffer[0] = PAGE_ERASE_OPCODE;
            dataFlashCommandBuffer[1] = (uint8_t)(pageNumber >> 8u);
            dataFlashCommandBuffer[2] = (uint8_t)(pageNumber);
            dataFlashCommandBuffer[3] = 0;
            ret = DataFlashSpiWrite(dataFlashCommandBuffer, 4u);
            if(ret >= 0)
            {
                ret = WaitDataFlashToGetReady();
                if(ret >= 0)
                {
                    ret = DataFlashReadTransactionStatus();
                }
            }
        }
    }
    
    return ret;
}


//------------------------------------------------------------------------------
//  int32_t DataFlashEraseBlock (uint16_t blockNumber)
//
//   Author:  Dilawar Ali
//   Date:    2018/10/01
//
//!  This function erase the specified Block
//
//------------------------------------------------------------------------------
int32_t DataFlashEraseBlock (uint32_t blockNumber)
{
    int32_t ret = 0;
    uint32_t blockNumberMask = 0;
    if(blockNumber > DATAFLASH_TOTAL_BLOCKS)
    {
        ret = ERR_DATAFLASH_BLOCK_INVALID;
    }
    else
    {
        ret = WaitDataFlashToGetReady();
        if(ret >= 0)
        {
            dataFlashCommandBuffer[0] = BLOCK_ERASE_OPCODE;
            blockNumberMask = blockNumber;
            blockNumberMask <<= 11u;
            dataFlashCommandBuffer[1] = (uint8_t)(blockNumberMask >> 16u);;
            dataFlashCommandBuffer[2] = (uint8_t)(blockNumberMask >> 8u);
            dataFlashCommandBuffer[3] = (uint8_t)(blockNumberMask);
            
            ret = DataFlashSpiWrite(dataFlashCommandBuffer, 4u);
            if(ret >= 0)
            {
                ret = WaitDataFlashToGetReady();
                if(ret >= 0)
                {
                    ret = DataFlashReadTransactionStatus();
                }
            }
        }
    }
    
    return ret;
}


//------------------------------------------------------------------------------
//  int32_t DataFlashEraseSector (uint8_t sectorNumber, BOOLEAN isSector0b)
//
//   Author:  Dilawar Ali
//   Date:    2018/10/01
//
//!  This function erase the specified Sector
//
//------------------------------------------------------------------------------
int32_t DataFlashEraseSector (uint8_t sectorNumber, BOOLEAN isSector0b)
{
    int32_t ret = 0;
    
    if(sectorNumber > DATAFLASH_TOTAL_SECTORS)
    {
        ret = ERR_DATAFLASH_SECTOR_INVALID;
    }
    else
    {
        ret = WaitDataFlashToGetReady();
        if(ret >= 0)
        {
            dataFlashCommandBuffer[0] = SECTOR_ERASE_OPCODE;
            if((sectorNumber == 0) && (isSector0b == true))
            {
                dataFlashCommandBuffer[1] = sectorNumber;
                dataFlashCommandBuffer[2] = 0x08;
                dataFlashCommandBuffer[3] = 0;
            }
            else
            {
                dataFlashCommandBuffer[1] = sectorNumber;
                dataFlashCommandBuffer[2] = 0;
                dataFlashCommandBuffer[3] = 0;
            }
            ret = DataFlashSpiWrite(dataFlashCommandBuffer, 4u);
            if(ret >= 0)
            {
                ret = WaitDataFlashToGetReady();
                if(ret >= 0)
                {
                    ret = DataFlashReadTransactionStatus();
                }
            }
        }
    }
    
    return ret;
}

//------------------------------------------------------------------------------
//  int32_t DataFlashReadPage (uint16_t pageNumber)
//
//   Author:  Dilawar Ali
//   Date:    2018/10/01
//
//!  This function read the specified Page from main memory
//
//------------------------------------------------------------------------------
int32_t DataFlashReadPage (uint16_t pageNumber, uint8_t dstBuffer[], uint32_t size)
{
    int32_t ret = 0;
    
    if(pageNumber > DATAFLASH_TOTAL_PAGES)
    {
        ret = ERR_DATAFLASH_PAGE_INVALID;
    }
    else
    {
        if(size == DATAFLASH_BYTES_PER_PAGE)
        {
            dataFlashCommandBuffer[0] = MAIN_MEMORY_PAGE_READ_OPCODE;
            dataFlashCommandBuffer[1] = (uint8_t)(pageNumber >> 8u);
            dataFlashCommandBuffer[2] = (uint8_t)(pageNumber);
            dataFlashCommandBuffer[3] = 0;
            
            // 4 Dummy Bytes
            dataFlashCommandBuffer[4] = 0;
            dataFlashCommandBuffer[5] = 0;
            dataFlashCommandBuffer[6] = 0;
            dataFlashCommandBuffer[7] = 0;
            
            ret = DataFlashSpiTransfer(dataFlashCommandBuffer, dataFlashResponseBuffer, (size + 8u));
            if(ret >= 0)
            {
                ret = DataFlashReadTransactionStatus();
                if(ret >= 0)
                {
                    memcpy(dstBuffer, &dataFlashResponseBuffer[8], size);
                }
            }
        }
        else
        {
            ret = ERR_DATAFLASH_BUFF_SIZE_INVALID;
        }
    }
    return ret;
}

//------------------------------------------------------------------------------
//  int32_t DataFlashEnablePowerSaving (void)
//
//   Author:  Dilawar Ali
//   Date:    2018/10/17
//
//!  This function set data flash to ultra deep power saving mode
//
//------------------------------------------------------------------------------
int32_t DataFlashEnablePowerSaving (void)
{
    int32_t ret = 0;
    dataFlashCommandBuffer[0] = ULTRA_DEEP_POWER_DOWN_OPCODE;
    ret = DataFlashSpiWrite(dataFlashCommandBuffer, 1u);
    
    return ret;
}


//------------------------------------------------------------------------------
//  int32_t DataFlashDisablePowerSaving (void)
//
//   Author:  Dilawar Ali
//   Date:    2018/10/17
//
//!  This function set data flash to stand by mode
//
//------------------------------------------------------------------------------
int32_t DataFlashDisablePowerSaving (void)
{
    int32_t ret = 0;
    dataFlashCommandBuffer[0] = 0;
    ret = DataFlashSpiWrite(dataFlashCommandBuffer, 1u);
    
    return ret;
}

//==============================================================================
//  End Of File
//==============================================================================