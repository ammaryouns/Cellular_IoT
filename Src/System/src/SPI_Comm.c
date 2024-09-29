//==============================================================================
//
//  SPI_Communication.c
//
//  Copyright (C) 2018 by Industrial Scientific.
//
//  This document and all information contained within are confidential and
//  proprietary property of Industrial Scientific Corporation. All rights
//  reserved. It is not to be reproduced or reused without the prior approval
//  of Industrial Scientific Corporation.
//
//==============================================================================
//  FILE INFORMATION
//==============================================================================
//
//  Source:        SPI_Communication.c
//
//  Project:       FREY
//
//  Author:        TAYYAB TAHIR
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
//! This module takes care of all the communication/messages between smart battery
//! board and VPRO
//
//==============================================================================
//  REVISION HISTORY
//==============================================================================
//  Revision: 1.0  
//      
//
//==============================================================================
//  INCLUDES
//==============================================================================
#include <stdint.h>
#include <string.h>
#include <em_gpio.h>
#include <gpiointerrupt.h>
#include <stdio.h>
#include <em_int.h>

#include "SPI_Comm.h"
#include "SysTask.h"
#include "Event.h"
#include "ExtCommunication.h"
#include "Cellular.h"
#include "main.h"

//==============================================================================
//	CONSTANTS, TYPEDEFS AND MACROS 
//==============================================================================
#define MSG_MAX_ID_LEN    16u

//==============================================================================
//	LOCAL DATA STRUCTURE DEFINITION
//==============================================================================


//==============================================================================
//	GLOBAL DATA DECLARATIONS
//==============================================================================
SPIDRV_HandleData_t vproComSPIHandleData;
SPIDRV_Handle_t vproCommSPIHandle = &vproComSPIHandleData;

Remote_Instrument_t RemoteUnit;
uint8_t referenceUserName[SERIAL_NUMBER_LENGTH];
uint8_t referenceSiteName[SERIAL_NUMBER_LENGTH];
//Gps_Data_t GPSInfo;

//==============================================================================
//	LOCAL DATA DECLARATIONS
//==============================================================================

//SPI Related Paramters 

//Variables to hold the data received from the SPI PEripheral of VPRO
uint8_t slaveRxBuffer[SPI_SLAVE_BUFFER_LENGTH];
uint8_t slaveTxBuffer[SPI_SLAVE_BUFFER_LENGTH];
uint8_t tempDataBuffer[10];


unsigned int MasterRequest = NO_MESSAGE_LEFT;
static int CopyOfMasterRequest = NO_MESSAGE_LEFT;
//static int messageCounter = 0u;
static bool isStatusMessageSent = false;
static bool isQuickStatusMessageSent = false;
static bool isRadioConfigMessageSent = false;
//static bool isInetMessageSent = false;

static unsigned char MasterRequestedRadioParameter = NO_PARAMETER;

bool isEventBased = false;

int32_t TimerTime = 0;
int32_t TimerTimeA = 0;
int32_t TimerTimeB = 0;
//==============================================================================
//	LOCAL FUNCTION PROTOTYPES
//==============================================================================
//.These Functions handels the incoming data from the master device 
static void ProcessInstrumentDataMessage        (uint8_t *IncomingBuffer, uint8_t StartingIndex);
static void ProcessCannedMessage                (uint8_t *IncomingBuffer, uint8_t StartingIndex);
static void ProcessProximityAlarmMessage        (uint8_t *IncomingBuffer, uint8_t StartingIndex);
static void ProcessSetRadioConfigureMesssage    (uint8_t *IncomingBuffer, uint8_t StartingIndex);
static void ProcessGetRadioConfigureMesssage    (uint8_t *IncomingBuffer, uint8_t StartingIndex);
static void ProcessWassupMesssage               (uint8_t *IncomingBuffer, uint8_t StartingIndex);
static void ProcessWassupAgainMesssage          (uint8_t *IncomingBuffer, uint8_t StartingIndex);
static void ProcessFirmwareHeader               (uint8_t *IncomingBuffer, uint8_t StartingIndex);
static void ProcessFirmwareData                 (uint8_t *IncomingBuffer, uint8_t StartingIndex);
static void ProcessFirmwareFooter               (uint8_t *IncomingBuffer, uint8_t StartingIndex);


//.These Functions handels the outgoing messages to master device 
static uint8_t BuildDataStatusMessage       (uint8_t *OutgoingBuffer);
static uint8_t BuildQuickStatusMessage      (uint8_t *OutgoingBuffer);
static uint8_t BuildRadioConfigureMessage   (uint8_t *OutgoingBuffer);
static uint8_t BuildNoRemainingMesage       (uint8_t *OutgoingBuffer);
static uint8_t BuildCustomINETMessage       (uint8_t *OutgoingBuffer);
static uint8_t BuildFirmwareAckNackMessage  (uint8_t *OutgoingBuffer);

//.Miscellaneous functions
static void  IsMessageAvalibleForMasterDevice(void);
static void  ResendFailedChecksumMessage(void);
static uint8_t CalculateMessageChecksum(uint8_t *buffer,unsigned short startIndex,unsigned short endIndex);
static void UpdateIntrumentData(void);


static void CreateInstrumentEvent(void);
static void CSgpioCallback(uint8_t pin);
static void SPITransferCompleteFxn(SPIDRV_Handle_t handle, Ecode_t transferStatus, int itemsTransferred);
//==============================================================================
//	LOCAL AND GLOBAL FUNCTIONS IMPLEMENTATION
//==============================================================================


//==============================================================================
//
//  static void SPITransferCompleteFxn(SPI_Handle handle, SPI_Transaction *transaction)
//
//   Author:   Tayyab Tahir
//   Date:     2018/04/26
//
//!  This is the callback function for SPI Prorocol
//
//==============================================================================
static void SPITransferCompleteFxn(SPIDRV_Handle_t handle, Ecode_t transferStatus, int itemsTransferred)
{
    //d@Create variable to hold the SPI treansaction state 
    //SPI_Status SPI_Current_Status = SPI_TRANSFER_COMPLETED;
    //Get the status of the transaction that has been canceled
    //SPI_Current_Status = transaction->status;
}


//==============================================================================
//
//  void CSgpioCallback(uint8_t pin)
//
//   Author:   Tayyab Tahir
//   Date:     2018/04/26
//
//!  This is the callback function for CS GPIO INT
//
//==============================================================================
void CSgpioCallback(uint8_t pin)
{
    // Chceck Interrupt Pin
    if(pin == 13)
    {
        if(GPIO_PinInGet(gpioPortE, 13) >= 1)
        {
            // Rising Edge Interrupt
            //            printf("High\r\n");
            // Stop Ongoing Transfer
            SPIDRV_AbortTransfer(vproCommSPIHandle);
            //Process the data received from the master device 
            ProcessIncomingmessage(slaveRxBuffer,0);
            //Create events if master changes any state 
            CreateInstrumentEvent();
            //Update the InstrumentInfo structure 
            UpdateIntrumentData();
            //Prepare the outgoing buffer to respond master 
            (void)ManageOutgoingMessage();
            
            isSpiReadyToSleep = true;
            
        }
        else
        {
            //printf("Low\r\n");
            //Falling Edge Interrupt
            isSpiReadyToSleep = false;
            SPIDRV_STransfer(vproCommSPIHandle, slaveTxBuffer, slaveRxBuffer, SPI_SLAVE_BUFFER_LENGTH, SPITransferCompleteFxn, 0u);          
        }
    }
}


//==============================================================================
//
//  void InitializeIntrumentData(void)
//
//   Author:   Tayyab Tahir
//   Date:     2018/05/03
//
//!  This function handels the Instrument Information initialization 
//
//==============================================================================
void InitializeIntrumentData(void)
{
    //Clears the instrumentInfo struct
    memset(&InstrumentInfo,0,sizeof(InstInfo_t));
    
    //Initialize the InstrumentInfo structure 
    InstrumentInfo.BatteryType = WIRELESS_DEVICE_CELLULAR;
    InstrumentInfo.MessgaeVersion = MSG_VERSION_ONE;
    InstrumentInfo.InetStatus = INET_NOT_MONITORING_INSTRUMENT;
    InstrumentInfo.SmartBatteryStatus = NO_CONNECTION_AVALIBLE;
}


//==============================================================================
//
//  int32_t SPISlaveConfigure(void)
//
//   Author:   Tayyab Tahir
//   Date:     2018/04/26
//
//!  This function handels the slave initialization 
//
//==============================================================================
int32_t SPISlaveConfigure(void)
{
    int32_t ret = 0;
    SPIDRV_Init_t initData =
    {
        USART0,                     /* USART port                       */ 
        _USART_ROUTELOC0_TXLOC_LOC0, /* USART Tx pin location number    */ 
        _USART_ROUTELOC0_RXLOC_LOC0, /* USART Rx pin location number    */ 
        _USART_ROUTELOC0_CLKLOC_LOC0, /* USART Clk pin location number  */ 
        _USART_ROUTELOC0_CSLOC_LOC0, /* USART Cs pin location number    */ 
        0,                          /* Bitrate                          */ 
        8,                          /* Frame length                     */ 
        0,                          /* Dummy Tx value for Rx only funcs */ 
        spidrvSlave,                /* SPI mode                         */ 
        spidrvBitOrderMsbFirst,     /* Bit order on bus                 */ 
        spidrvClockMode0,           /* SPI clock/phase mode             */ 
        spidrvCsControlAuto, /* CS controlled by the Application */ 
        spidrvSlaveStartImmediate   /* Slave start transfers immediately*/ 
    };
    // Initialize an SPI driver instance.
    SPIDRV_Init(vproCommSPIHandle, &initData);
    if(vproCommSPIHandle == NULL)
    {
        ret = -1;
    }
    
    //---------------- CS PIN configurations as GPIO Interrupt Pin -----------------------------
    GPIOINT_Init();
    
    GPIO_PinModeSet(gpioPortE, 13, gpioModeInputPull, 1);
    GPIOINT_CallbackRegister(13, CSgpioCallback);
    GPIO_IntConfig(gpioPortE, 13, true, true, true);
    InitializeIntrumentData();
    return ret;
}

//==============================================================================
//
//  void UpdateIntrumentData(void)
//
//   Author:   Tayyab Tahir
//   Date:     2018/05/03
//
//!  This function updates the Instrument Information
//
//==============================================================================
void UpdateIntrumentData(void)
{
    
    if((gCellularDriver.cellularState == CELLULAR_READY) || (gCellularDriver.cellularState == CELLULAR_BUSY))
    {
        //Set the battery status to be connected via wifi
        InstrumentInfo.SmartBatteryStatus = RUNNING_NORMAL;
        //Inet is accepting the instrument 
        InstrumentInfo.InetStatus = INET_ACCEPTS_INSTRUMENT_AND_MONITORING;
    }
    else
    {
        //Set the battery status to be not connected via wifi
        InstrumentInfo.SmartBatteryStatus = NO_CONNECTION_AVALIBLE;
        //Inet is not accepting the instrument 
        InstrumentInfo.InetStatus = INET_NOT_MONITORING_INSTRUMENT;
    }    
}


//==============================================================================
//
//  void ProcessIncomingmessage(uint8_t *IncomingBuffer, uint8_t StartingIndex)
//
//   Author:   Tayyab Tahir
//   Date:     2018/04/23
//
//!  This function handeles data extraction from incoming message according to 
//!  their type.
//
//==============================================================================
void ProcessIncomingmessage(uint8_t *IncomingBuffer, uint8_t StartingIndex)
{
    uint8_t MessageType  = IncomingBuffer[StartingIndex+2];
    
    //    printf("Message Type: %x \n ",MessageType);
    switch (MessageType)
    {
    case COMMAND_SEND_DATA_TO_WIFI:   
        ProcessInstrumentDataMessage(IncomingBuffer , StartingIndex);
        break;
        
    case COMMAND_SEND_CANNED_MSG_TO_CLOUD:
        ProcessCannedMessage(IncomingBuffer , StartingIndex);
        break;
        
    case COMMAND_SEND_PROXIMITY_ALARM_TO_CLOUD:
        ProcessProximityAlarmMessage(IncomingBuffer , StartingIndex);
        break;
        
    case COMMAND_SEND_RADIO_CONFIGURE_MESSAGE:
        //printf("Send Radio Configure\n");
        ProcessSetRadioConfigureMesssage(IncomingBuffer , StartingIndex);
        break;
        
    case COMMAND_GET_RADIO_CONFIGURE_MESSAGE:
        ProcessGetRadioConfigureMesssage(IncomingBuffer , StartingIndex);
        MasterRequest = RESPOND_RADIO_CONFIGURATION;
        break;
        
    case COMMAND_SEND_WASSUP_MESSAGE:
        ProcessWassupMesssage(IncomingBuffer , StartingIndex);
        break;
        
    case COMMAND_SEND_WASSUP_AGAIN_MESSAGE:
        ProcessWassupAgainMesssage(IncomingBuffer , StartingIndex);
        break;
        
    case COMMAND_GET_FW_HEADER:
        ProcessFirmwareHeader(IncomingBuffer , StartingIndex);
        MasterRequest = RENSPOND_FW_ACK_NACK_MESSAGE;
        break;
        
    case COMMAND_GET_FW_DATA:
        ProcessFirmwareData(IncomingBuffer , StartingIndex);
        MasterRequest = RENSPOND_FW_ACK_NACK_MESSAGE;
        break;
        
    case COMMAND_GET_FW_FOOTER:
        ProcessFirmwareFooter(IncomingBuffer , StartingIndex);
        MasterRequest = RENSPOND_FW_ACK_NACK_MESSAGE;
        break;
        
        
    default:
        //Do Nothing 
        break;
    }
}

//==============================================================================
//
//  static void ProcessInstrumentDataMessage(uint8_t *IncomingBuffer, uint8_t StartingIndex)
//
//   Author:   Tayyab Tahir
//   Date:     2018/04/23
//
//!  This function extracts the instrument informnation from the incoming message.
//
//==============================================================================
static void ProcessInstrumentDataMessage(uint8_t *IncomingBuffer, uint8_t StartingIndex)
{
    uint8_t CalculatedChecksum = 0u;
    uint8_t ReceivedChecksum = 0u;
    uint8_t Index = 0u;
    uint8_t LoopCounter = 0u;
    uint8_t TempData = 0u;
    uint8_t SensorCount = 0u;
    
    // Extract message byte for checksum calculation
    TempData= IncomingBuffer[LENGTH_BYTE];
    
    //Received checksum is present a byte before end framing characters
    ReceivedChecksum = IncomingBuffer[StartingIndex+5+TempData];
    //Calculate checksum excluding the start and end framing characters 
    CalculatedChecksum = CalculateMessageChecksum(IncomingBuffer, (unsigned short)(StartingIndex+2), (unsigned short)(StartingIndex+4+TempData));
    
    if (ReceivedChecksum == CalculatedChecksum)
    {
        // Byte 0-1 Start framing Character of the message ..Ignore
        Index ++;
        Index ++;
        
        // Byte 2 Messgae Type ..Ignore
        Index ++;
        
        // Byte 3 Version of message 
        Index ++;
        
        // Byte 4 Instrument Type
        RemoteUnit.InstrumentType = (INSTRUMENT_TYPE_t)IncomingBuffer[Index++];
        
        // Byte 5 Payload Length 
        TempData = IncomingBuffer[Index++];             //53 + (5 * n) where n is number of sensors with gas reading 
        SensorCount = (TempData - I_AM_OK_MESSAGE_SIZE)/5;
        //RemoteUnit.SensorsInfo.numberOfSensors = SensorCount;
        
        // Byte 6-21 Instrument Serial Number
        for (LoopCounter = 0; LoopCounter < SERIAL_NUMBER_LENGTH ;LoopCounter++)
        {
            RemoteUnit.SerialNumber[LoopCounter] = IncomingBuffer[Index++];
        }
        
        // Byte 22 Message Counter
        RemoteUnit.MessageCounter = IncomingBuffer[Index++];
        
        // Byte 23 Instrument Status
        RemoteUnit.InstrumentState = IncomingBuffer[Index++];
        
        // Byte 24-39 Username 
        for (LoopCounter = 0; LoopCounter < SERIAL_NUMBER_LENGTH ;LoopCounter++)
        {
            RemoteUnit.UserName[LoopCounter] = IncomingBuffer[Index++];
        }
        
        // Byte 40-55 Sitename 
        for (LoopCounter = 0; LoopCounter < SERIAL_NUMBER_LENGTH ;LoopCounter++)
        {
            RemoteUnit.SiteName[LoopCounter] = IncomingBuffer[Index++];
        }
        
        // Byte 56-90 Sensor Data (5 Bytes per sensor )
        for (LoopCounter = 0; LoopCounter < SensorCount ;LoopCounter++)
        {
            //Byte 1: Containing Sensor Type            
            RemoteUnit.SensorsInfo.sensorArray[LoopCounter].SensorType = (SENSOR_TYPES_t) IncomingBuffer[Index++];
            //Byte 2: Containing MeasuringUnit
            RemoteUnit.SensorsInfo.sensorArray[LoopCounter].SensorMeasuringUnits = (GAS_MEASUREMENT_UNITS_t) IncomingBuffer[Index++];
            //Byte 3: Containing Sensor Status(High Nibble) & Measuring Resolution(Low Nibble)
            RemoteUnit.SensorsInfo.sensorArray[LoopCounter].SensorStatus = (SENSOR_STATUS_t) HINIBBLE_WORD8(IncomingBuffer[Index]); 
            RemoteUnit.SensorsInfo.sensorArray[LoopCounter].DecimalPlaces = LONIBBLE_WORD8(IncomingBuffer[Index++]);
            //Byte 4: Containing Sensor Reading (High Byte)
            RemoteUnit.SensorsInfo.sensorArray[LoopCounter].SensorReadingHigh = IncomingBuffer[Index++];
            //Byte 5: Containing Sensor Reading (Low Byte)
            RemoteUnit.SensorsInfo.sensorArray[LoopCounter].SensorReadingLow = IncomingBuffer[Index++];
        }
        if (SensorCount == 0)
        {
            //Reset the sensors status's and sensor readings if detailed message isnt received 
            for (LoopCounter = 0; LoopCounter < MAX_SENSOR_SUPPORTED ;LoopCounter++)
            {
                //Byte 3: Containing Sensor Status(High Nibble) & Measuring Resolution(Low Nibble)
                RemoteUnit.SensorsInfo.sensorArray[LoopCounter].SensorStatus = SENSOR_NORMAL; 
                
                if (RemoteUnit.SensorsInfo.sensorArray[LoopCounter].SensorType == SENSOR_GAS_TYPE_O2)
                {
                    //If sensor type is oxygen then set the normal reading value i-e 209
                    RemoteUnit.SensorsInfo.sensorArray[LoopCounter].SensorReadingHigh = 0;
                    RemoteUnit.SensorsInfo.sensorArray[LoopCounter].SensorReadingLow = 209;
                }
                else
                {
                    //If sensor type is other then oxygen then set the normal reading value i-e 0
                    RemoteUnit.SensorsInfo.sensorArray[LoopCounter].SensorReadingHigh = 0;
                    RemoteUnit.SensorsInfo.sensorArray[LoopCounter].SensorReadingLow = 0;
                }
            }
        }
        else
        {
            //Populate sensor data only when Vpro sends the detailed message 
            RemoteUnit.SensorsInfo.numberOfSensors = SensorCount;
        }
        
        // Byte 91-92 Checksum 
        Index ++;
        Index ++;
        
        // Byte 93-94 End framing Character of the message 
        Index ++;
        Index ++;
        
    }
    else
    {
        //Checksum failed no parsing needed
    }
}


//==============================================================================
//
//  static void ProcessCannedMessage(uint8_t *IncomingBuffer, uint8_t StartingIndex)
//
//   Author:   Tayyab Tahir
//   Date:     2018/04/23
//
//!  This function extracts the custom message sent by the master device from 
//!  incoming message.
//
//==============================================================================
static void ProcessCannedMessage(uint8_t *IncomingBuffer, uint8_t StartingIndex)
{    
    uint8_t CalculatedChecksum = 0u;
    uint8_t ReceivedChecksum = 0u;
    uint8_t Index = 0u;
    uint8_t LoopCounter = 0u;
    uint8_t TempData = 0u;
    
    // Extract message byte for checksum calculation
    TempData= IncomingBuffer[LENGTH_BYTE];
    
    ReceivedChecksum = IncomingBuffer[StartingIndex+5+TempData];
    CalculatedChecksum = CalculateMessageChecksum(IncomingBuffer, (unsigned short)(StartingIndex+2), (unsigned short)(StartingIndex+4+TempData));
    
    
    if (ReceivedChecksum == CalculatedChecksum)
    {
        // Byte 0-1 Start framing Character of the message ..//Ignore
        Index ++;
        Index ++;
        
        // Byte 2 Messgae Type ..Ignore
        Index ++;
        
        // Byte 3 Version of message ..//Ignore
        Index ++;
        
        // Byte 4 Instrument Type
        RemoteUnit.InstrumentType = (INSTRUMENT_TYPE_t)IncomingBuffer[Index++];
        
        // Byte 5 Length of payload 
        TempData = IncomingBuffer[Index++];
        
        // Byte 6-21 Instrument serial number
        for (LoopCounter = 0; LoopCounter < SERIAL_NUMBER_LENGTH ;LoopCounter++)
        {
            RemoteUnit.SerialNumber[LoopCounter] = IncomingBuffer[Index++];
        }
        
        // Byte 22 Messgae counter
        RemoteUnit.MessageCounter = IncomingBuffer[Index++];
        
        // Byte 23-70 Message
        for (LoopCounter = 0; LoopCounter < TempData ;LoopCounter++)
        {
            RemoteUnit.UserCustomMessage[LoopCounter] = IncomingBuffer[Index++];
        }
        
        // Byte 71-72 Checksum ..//Ignore
        Index ++;
        Index ++;
        
        // Byte 73-74 End framing Character of the message ..//Ignore
        Index ++;
        Index ++;
    }
    else
    {
        //Checksum failed no parsing needed
    }
    
}


//==============================================================================
//
//  static void ProcessSetRadioConfigureMesssage(uint8_t *IncomingBuffer, uint8_t StartingIndex)
//
//   Author:   Tayyab Tahir
//   Date:     2018/04/23
//
//!  This function extracts the Radio Configuarations sent by the master device from 
//!  incoming message. 
//
//==============================================================================
static void ProcessSetRadioConfigureMesssage(uint8_t *IncomingBuffer, uint8_t StartingIndex)
{
    uint8_t CalculatedChecksum = 0u;
    uint8_t ReceivedChecksum = 0u;
    uint8_t Index = 0u;
    uint8_t LoopCounter = 0u;
    uint8_t ParameterType = 0u;
    uint8_t TempData = 0u;
    
    // Extract message byte for checksum calculation
    TempData= IncomingBuffer[LENGTH_BYTE];
    
    ReceivedChecksum = IncomingBuffer[StartingIndex+5+TempData];
    CalculatedChecksum = CalculateMessageChecksum(IncomingBuffer, (unsigned short)(StartingIndex+2), (unsigned short)(StartingIndex+4+TempData));
    
    if (ReceivedChecksum == CalculatedChecksum)
    {
        // Byte 0-1 Start framing Character of the message ..//Ignore
        Index ++;
        Index ++;
        
        // Byte 2 Messgae Type ..Ignore
        Index ++;
        
        // Byte 3 Version of message ..//Ignore
        Index ++;
        
        // Byte 4 Parameter to be set
        ParameterType = IncomingBuffer[Index++];
        
        //Byte 5 Length of message 
        TempData = IncomingBuffer[Index++];
        
        // Byte 6- Onward parameter data
        switch (ParameterType)
        {
        case SERIAL_NUMBER:
            for (LoopCounter = 0; LoopCounter < TempData ;LoopCounter++)
            {
                // Read 16 Byte serial number
                InstrumentInfo.SerialNumber[LoopCounter] = IncomingBuffer[Index++];
            }
            break;
            
        case MANUFACTURING_DATE:
            for (LoopCounter = 0; LoopCounter < TempData ;LoopCounter++)
            {
                // Read 4 Byte Mfg date
                InstrumentInfo.ManufacturingDate[LoopCounter] = IncomingBuffer[Index++];
            }
            break;
            
        case INST_PART_NUMBER:
            for (LoopCounter = 0; LoopCounter < TempData ;LoopCounter++)
            {
                // Read 16 Byte Part number
                InstrumentInfo.PartNumber[LoopCounter] = IncomingBuffer[Index++];
            }
            break;
            
        case TECHNICIAN_INITIALS:
            for (LoopCounter = 0; ((LoopCounter < TempData) && (LoopCounter < TECH_INITIALS_LENGTH)) ;LoopCounter++)
            {
                // Read 4 Byte Technician initials 
                InstrumentInfo.TechniciansInitials[LoopCounter] = IncomingBuffer[Index++];
            }
            break;
            
        case JOB_NUMBER:
            for (LoopCounter = 0; LoopCounter < TempData ;LoopCounter++)
            {
                // Read 8 Byte Job number
                InstrumentInfo.JobNumber[LoopCounter] = IncomingBuffer[Index++];
            }
            break;
            
        case HARDWARE_VERSION:
            // Read 1 Byte Hardware version
            InstrumentInfo.HardwareVersion = IncomingBuffer[Index++];
            break;
            
            
        case BATTERY_TYPE:   
            // Read 1 Byte Network Encryption Type
            InstrumentInfo.BatteryType = IncomingBuffer[Index++];
            break;
            
        case SIM_APN:   
            // Read 1 Byte Network Encryption Type
            for (LoopCounter = 0; ((LoopCounter < TempData) && (LoopCounter < SIM_APN_LEN)) ;LoopCounter++)
            {
                // Write 64 byte apn
                gCellularDriver.APN[LoopCounter] = IncomingBuffer[Index++];
            }
            break;
            
        case CELL_NUMBER_1:   
            // Read 1 Byte Network Encryption Type
            for (LoopCounter = 0; ((LoopCounter < TempData) && (LoopCounter < PHONE_NUMBER_LEN)) ;LoopCounter++)
            {
                // Write 64 byte apn
                gCellularDriver.cellNumber1[LoopCounter] = IncomingBuffer[Index++];
            }
            break;
            
        case CELL_NUMBER_2:   
            // Read 1 Byte Network Encryption Type
            for (LoopCounter = 0; ((LoopCounter < TempData) && (LoopCounter < PHONE_NUMBER_LEN)) ;LoopCounter++)
            {
                // Write 64 byte apn
                gCellularDriver.cellNumber2[LoopCounter] = IncomingBuffer[Index++];
            }
            break;
            
        }
        
        // Byte  Checksum 
        Index ++;
        Index ++;
        
        // Byte  End framing Character of the message 
        Index ++;
        Index ++;
        
    }
    else
    {
        //Checksum failed no parsing needed
    }
}


//==============================================================================
//
//  static void ProcessGetRadioConfigureMesssage(uint8_t *IncomingBuffer, uint8_t StartingIndex)
//
//   Author:   Tayyab Tahir
//   Date:     2018/04/23
//
//!  This function extracts the Radio Configuarations requested by the master device 
//!  from incoming message. 
//
//==============================================================================
static void ProcessGetRadioConfigureMesssage(uint8_t *IncomingBuffer, uint8_t StartingIndex)
{
    uint8_t Index = 0u;
    
    // Byte 0-1 Start framing Character of the message ..//Ignore
    Index ++;
    Index ++;
    
    // Byte 2 Messgae Type ..Ignore
    Index ++;
    
    // Byte 3 Version of message ..//Ignore
    Index ++;
    
    // Byte 4 Parameter to be set
    MasterRequestedRadioParameter = IncomingBuffer[Index++];
    
    // Byte  End framing Character of the message 
    Index ++;
    Index ++;
    
    // Set this flag to make sure that slave send the data required by the master 
    isRadioConfigMessageSent = true;
}


//==============================================================================
//
//  static void ProcessProximityAlarmMessage(uint8_t *IncomingBuffer, uint8_t StartingIndex)
//
//   Author:   Tayyab Tahir
//   Date:     2018/04/23
//
//!  This function extracts the Alarm messages sent by the master device from
//!  incoming message. 
//
//==============================================================================
static void ProcessProximityAlarmMessage(uint8_t *IncomingBuffer, uint8_t StartingIndex)
{
    uint8_t CalculatedChecksum = 0u;
    uint8_t ReceivedChecksum = 0u;
    uint8_t Index = 0u;
    uint8_t LoopCounter = 0u;
    uint8_t TempData = 0u;
    
    // Extract message byte for checksum calculation
    TempData= IncomingBuffer[LENGTH_BYTE];
    
    ReceivedChecksum = IncomingBuffer[StartingIndex+5+TempData];
    CalculatedChecksum = CalculateMessageChecksum(IncomingBuffer, (unsigned short)(StartingIndex+2), (unsigned short)(StartingIndex+4+TempData));
    
    if (ReceivedChecksum == CalculatedChecksum)
    {
        // Byte 0-1 Start framing Character of the message ..Ignore
        Index ++;
        Index ++;
        
        // Byte 2 Messgae Type ..Ignore
        Index ++;
        
        // Byte 3 Version of message 
        Index ++;
        
        // Byte 4 Instrument Type
        RemoteUnit.InstrumentType = (INSTRUMENT_TYPE_t)IncomingBuffer[Index++];
        
        // Byte 5 Length of payload
        TempData = IncomingBuffer[Index++];
        
        // Byte 6-21 Instrument serial number
        for (LoopCounter = 0; LoopCounter < SERIAL_NUMBER_LENGTH ;LoopCounter++)
        {
            RemoteUnit.SerialNumber[LoopCounter] = IncomingBuffer[Index++];
        }
        
        // Byte 22 Messgae counter
        RemoteUnit.MessageCounter = IncomingBuffer[Index++];
        
        // Byte 23 Alarm type
        RemoteUnit.InstrumentState = IncomingBuffer[Index++];
        
        // Byte 24-39 Username 
        for (LoopCounter = 0; LoopCounter < SERIAL_NUMBER_LENGTH ;LoopCounter++)
        {
            RemoteUnit.UserName[LoopCounter] = IncomingBuffer[Index++];
        }
        
        // Byte 40-55 Sitename 
        for (LoopCounter = 0; LoopCounter < SERIAL_NUMBER_LENGTH ;LoopCounter++)
        {
            RemoteUnit.SiteName[LoopCounter] = IncomingBuffer[Index++];
        }
        
        // Byte 56 Site Security Level
        RemoteUnit.SiteSecurityLevel = IncomingBuffer[Index++];
        
        // Byte 57 User Security Level
        RemoteUnit.UserSecurityLevel = IncomingBuffer[Index++];
        
        // Byte 58-59 Checksum 
        Index ++;
        Index ++;
        
        // Byte 60-61 End framing Character of the message 
        Index ++;
        Index ++;
    }
    else
    {
        //Checksum failed no parsing needed
    }
}


//==============================================================================
//
//  static void ProcessWassupMesssage(uint8_t *IncomingBuffer, uint8_t StartingIndex)
//
//   Author:   Tayyab Tahir
//   Date:     2018/04/23
//
//!  This function extracts the wassup call from the incoming message 
//
//==============================================================================
static void ProcessWassupMesssage(uint8_t *IncomingBuffer, uint8_t StartingIndex)
{
    uint8_t Index = 0u;
    
    // Byte 0-1 Start framing Character of the message ..Ignore
    Index ++;
    Index ++;
    
    // Byte 2 Messgae Type ..Ignore
    Index ++;
    
    // Byte 3-4 End framing Character of the message 
    Index ++;
    Index ++;
    
    // This functions decides that what messgae to send to master in response to wassup 
    IsMessageAvalibleForMasterDevice();
    
}


//==============================================================================
//
//  static void ProcessWassupAgainMesssage(uint8_t *IncomingBuffer, uint8_t StartingIndex)
//
//   Author:   Tayyab Tahir
//   Date:     2018/04/23
//
//!  This function extracts the wassup again call from the incoming message 
//
//==============================================================================
static void ProcessWassupAgainMesssage(uint8_t *IncomingBuffer, uint8_t StartingIndex)
{   
    uint8_t Index = 0u;
    
    // Byte 0-1 Start framing Character of the message ..Ignore
    Index ++;
    Index ++;
    
    // Byte 2 Messgae Type ..Ignore
    Index ++;
    
    // Byte 3-4 End framing Character of the message 
    Index ++;
    Index ++;
    
    ResendFailedChecksumMessage();
}


//==============================================================================
//
//  static void  IsMessageAvalibleForMasterDevice()
//
//   Author:   Tayyab Tahir
//   Date:     2018/04/23
//
//!  This function decides which message to be sent upon wassup message  
//
//==============================================================================
static void  IsMessageAvalibleForMasterDevice()
{
    if (MasterRequest == NO_MESSAGE_LEFT)
    {
        if (isStatusMessageSent == false)
        {
            //Prepare the Status Message in response to the first wassup message 
            MasterRequest = RESPOND_STATUS_MSG;
            //Reset the status message flag to ensure that it is sent 
            isStatusMessageSent = true;
        }
        else if(isQuickStatusMessageSent == false)
        {
            //Prepare the Quick status Message upon wassup message 
            MasterRequest = RESPOND_QUICK_STATUS_MSG;
            isQuickStatusMessageSent = true;
        }
        else if(isRadioConfigMessageSent == false)
        {
            //Prepare the radio configuration message requested by vpro
            MasterRequest = RESPOND_RADIO_CONFIGURATION;
            //Reset the flag to ensure that setting message is being sent to vpro
            isRadioConfigMessageSent = true;
        }
        else if(InstrumentInfo.IsINETMessageAvalibleforVpro == true)
        {
            //Prepare the Inet message for VPRO
            MasterRequest = PASS_INET_MESSAGE;
            //Reset the flag to ensure that Inet message is being sent to vpro 
            InstrumentInfo.IsINETMessageAvalibleforVpro = false;
        }
        else 
        {
            MasterRequest = NO_MESSAGE_LEFT;
            //Send the quick status message in the next transaction 
            isQuickStatusMessageSent = false;
        }
    }
    
    //Save copy of master message for resending purpose
    CopyOfMasterRequest = MasterRequest;
}

//==============================================================================
//
//  static void  ResendFailedChecksumMessage(void)
//
//   Author:   Tayyab Tahir
//   Date:     2018/04/23
//
//!  This function decides which message to be sent upon wassup again message  
//
//==============================================================================
static void  ResendFailedChecksumMessage(void)
{
    //Resend the last message that have been recorded
    MasterRequest = CopyOfMasterRequest;
}

//==============================================================================
//
//  uint8_t ManageOutgoingMessage(void)
//
//   Author:   Tayyab Tahir
//   Date:     2018/04/23
//
//!  This function manages the response to be sent to master device upon request  
//
//==============================================================================
uint8_t ManageOutgoingMessage(void)
{
    uint32_t tempData = MasterRequest;
    uint8_t ReturnLength = 0u;
    
    switch (tempData)
    {
    case RESPOND_STATUS_MSG:
        //Reset the Transmit buffer before sending the data 
        memset(slaveTxBuffer,NULL,SPI_SLAVE_BUFFER_LENGTH);
        //Construct the requested the string  
        ReturnLength = BuildDataStatusMessage(slaveTxBuffer);
        break;
        
    case RESPOND_RADIO_CONFIGURATION:  
        //Reset the Transmit buffer before sending the data 
        memset(slaveTxBuffer,NULL,SPI_SLAVE_BUFFER_LENGTH);
        //Construct the requested the string
        ReturnLength = BuildRadioConfigureMessage(slaveTxBuffer);
        break;
        
    case RESPOND_QUICK_STATUS_MSG:
        //Reset the Transmit buffer before sending the data 
        memset(slaveTxBuffer,NULL,SPI_SLAVE_BUFFER_LENGTH);
        //Construct the requested the string
        ReturnLength = BuildQuickStatusMessage(slaveTxBuffer);
        break;
        
    case PASS_INET_MESSAGE:
        //Reset the Transmit buffer before sending the data 
        memset(slaveTxBuffer,NULL,SPI_SLAVE_BUFFER_LENGTH);
        //Construct the requested the string
        ReturnLength = BuildCustomINETMessage(slaveTxBuffer);
        break;
        
    case NO_MESSAGE_LEFT:
        //Reset the Transmit buffer before sending the data 
        memset(slaveTxBuffer,NULL,SPI_SLAVE_BUFFER_LENGTH);
        //Construct the requested the string
        ReturnLength = BuildNoRemainingMesage(slaveTxBuffer);
        break;
    
    case RENSPOND_FW_ACK_NACK_MESSAGE:
        //Reset the Transmit buffer before sending the data 
        memset(slaveTxBuffer,NULL,SPI_SLAVE_BUFFER_LENGTH);
        //Construct the requested the string
        ReturnLength = BuildFirmwareAckNackMessage(slaveTxBuffer);
        break;
    
    default:
        break;
    }
    
    //Reset Master Request Message 
    MasterRequest = NO_MESSAGE_LEFT;
    
    return ReturnLength;
}

//==============================================================================
//
//  static void BuildDataStatusMessage(uint8_t *OutgoingBuffer)
//
//   Author:   Tayyab Tahir
//   Date:     2018/04/23
//
//!  This function writes the status message to the outgoing buffer   
//
//==============================================================================
static uint8_t BuildDataStatusMessage(uint8_t *OutgoingBuffer)
{
    uint8_t Index = 0u;
    uint8_t LoopCouter = 0u;
    uint8_t returnLength = 0u;
    
    // Byte 0-1 Start framing Character of the message 
    OutgoingBuffer[Index++] = START_FRAMING_CHARACTER;
    OutgoingBuffer[Index++] = START_FRAMING_CHARACTER;
    
    // Byte 2 Message Type: normal status message
    OutgoingBuffer[Index++] = SEND_READ_STATUS_MESSAGE;
    
    //Byte 3 Version
    OutgoingBuffer[Index++]  = InstrumentInfo.MessgaeVersion;
    
    //Byte 4 Battery Status 
    OutgoingBuffer[Index++] = InstrumentInfo.SmartBatteryStatus ;
    
    //Byte 5 INet Status 
    OutgoingBuffer[Index++] = InstrumentInfo.InetStatus;
    
    //Byte 6-7 RSSI
    OutgoingBuffer[Index++] = '2';
    OutgoingBuffer[Index++] = '8';
    
    //Byte 8-23 Serial Number
    for(LoopCouter = 0; LoopCouter < SERIAL_NUMBER_LENGTH ;LoopCouter++)
    {
        OutgoingBuffer[Index++] = InstrumentInfo.SerialNumber[LoopCouter];
    }
    
    // Byte 24-39 Network ID
    for(LoopCouter = 0; LoopCouter < MSG_MAX_ID_LEN ;LoopCouter++)
    {
        OutgoingBuffer[Index++] = gCellularDriver.carrier[LoopCouter];
    }
    
    //Byte 40-43 Current Time
    OutgoingBuffer[Index++] = 'C';
    OutgoingBuffer[Index++] = 'C'; 
    OutgoingBuffer[Index++] = 'C';
    OutgoingBuffer[Index++] = 'C';
    
    // Byte 44-47 GPS Latitude 
    OutgoingBuffer[Index++] = '1';
    OutgoingBuffer[Index++] = '2';
    OutgoingBuffer[Index++] = '3';
    OutgoingBuffer[Index++] = '4';
    
    // Byte 48 GPS Latitude Direction
    OutgoingBuffer[Index++] = 'E';
    
    // Byte 49-52 GPS Longitude
    OutgoingBuffer[Index++] = '5';
    OutgoingBuffer[Index++] = '6';
    OutgoingBuffer[Index++] = '7';
    OutgoingBuffer[Index++] = '8';
    
    // Byte 53 GPS Longitude Direction
    OutgoingBuffer[Index++] = 'W';
    
    // Byte 54-55 CheckSum of the message
    OutgoingBuffer[Index] = CalculateMessageChecksum(OutgoingBuffer, (unsigned short)2, (unsigned short)(Index-1));
    Index++;
    OutgoingBuffer[Index++] = 0x00;
    
    // Byte 56-57 Start framing Character of the message 
    OutgoingBuffer[Index++] = END_FRAMING_CHARACTER;
    OutgoingBuffer[Index++] = END_FRAMING_CHARACTER;
    
    returnLength = Index-1;
    return returnLength;
}


//==============================================================================
//
//  static void BuildQuickStatusMessage(uint8_t *OutgoingBuffer)
//
//   Author:   Tayyab Tahir
//   Date:     2018/04/23
//
//!  This function writes the quick status message to the outgoing buffer   
//
//==============================================================================
static uint8_t BuildQuickStatusMessage(uint8_t *OutgoingBuffer)
{
    uint8_t Index = 0u;
    uint8_t LoopCouter = 0u;
    uint8_t returnLength = 0u;
    
    // Byte 0-1 Start framing Character of the message 
    OutgoingBuffer[Index++] = START_FRAMING_CHARACTER;
    OutgoingBuffer[Index++] = START_FRAMING_CHARACTER;
    
    // Byte 2 Message Type: quick status message
    OutgoingBuffer[Index++] = SEND_READ_QUICK_STATUS_MESSAGE;
    
    //Byte 3 Version
    OutgoingBuffer[Index++]  = InstrumentInfo.MessgaeVersion;
    
    //Byte 4 Battery Status 
    OutgoingBuffer[Index++] = InstrumentInfo.SmartBatteryStatus;
    
    //Byte 5 INet Status 
    OutgoingBuffer[Index++] = InstrumentInfo.InetStatus;
    
    //Byte 6-7 RSSI
    OutgoingBuffer[Index++] = 'Z';
    OutgoingBuffer[Index++] = 'Z';
    
    //Byte 8 Cloud Message
    OutgoingBuffer[Index++] = InstrumentInfo.CloudMessageStatus;
    
    //Byte 9-24 Network ID
    for(LoopCouter = 0; LoopCouter < MSG_MAX_ID_LEN ;LoopCouter++)
    {
        OutgoingBuffer[Index++] = gCellularDriver.carrier[LoopCouter];
    }
    
    // Byte 24-28 GPS Latitude 
    OutgoingBuffer[Index++] = '1';
    OutgoingBuffer[Index++] = '2';
    OutgoingBuffer[Index++] = '3';
    OutgoingBuffer[Index++] = '4';
    
    // Byte 29 GPS Latitude Direction
    OutgoingBuffer[Index++] = 'E';
    
    // Byte 30-33 GPS Longitude
    OutgoingBuffer[Index++] = '5';
    OutgoingBuffer[Index++] = '6';
    OutgoingBuffer[Index++] = '7';
    OutgoingBuffer[Index++] = '8';
    
    // Byte 34 GPS Longitude Direction
    OutgoingBuffer[Index++] = 'W';
    // Byte 35-37 Firmware Version
    OutgoingBuffer[Index++] = FIRMWARE_VERSION_MAJOR;
    OutgoingBuffer[Index++] = FIRMWARE_VERSION_MINOR;
    OutgoingBuffer[Index++] = FIRMWARE_VERSION_BUILD;
    // Byte 35-36 CheckSum of the message
    OutgoingBuffer[Index] = CalculateMessageChecksum(OutgoingBuffer, (unsigned short)2, (unsigned short)(Index-1));
    Index++;
    OutgoingBuffer[Index++] = 0x00;
    
    // Byte 56-57 Start framing Character of the message 
    OutgoingBuffer[Index++] = END_FRAMING_CHARACTER;
    OutgoingBuffer[Index++] = END_FRAMING_CHARACTER;
    
    returnLength = Index-1;
    return returnLength;
}


//==============================================================================
//
//  static void BuildRadioConfigureMessage(uint8_t *OutgoingBuffer)
//
//   Author:   Tayyab Tahir
//   Date:     2018/04/23
//
//!  This function writes the requested parameter value to the outgoing buffer   
//
//==============================================================================
static uint8_t BuildRadioConfigureMessage(uint8_t *OutgoingBuffer)
{
    uint8_t Index = 0u;
    uint8_t LoopCounter = 0u;
    uint8_t returnLength = 0u;
    
    // Byte 0-1 Start framing Character of the message 
    OutgoingBuffer[Index++] = START_FRAMING_CHARACTER;
    OutgoingBuffer[Index++] = START_FRAMING_CHARACTER;
    
    // Byte 2 Message Type: Radio configure message
    OutgoingBuffer[Index++] = RESPOND_GET_RADIO_CONFIGURE_MESSAGE;
    
    //Byte 3 Version/Config 
    OutgoingBuffer[Index++]  = InstrumentInfo.MessgaeVersion;
    
    //Byte 4 Parameters to be read 
    OutgoingBuffer[Index++]  = MasterRequestedRadioParameter;
    
    //Byte 5 Payload length 
    OutgoingBuffer[Index++] = 0x00;
    
    
    //Byte 6 Payload (Requested Radio parameter)
    switch (MasterRequestedRadioParameter)
    {
    case SERIAL_NUMBER:
        for (LoopCounter = 0; LoopCounter < SERIAL_NUMBER_LENGTH ;LoopCounter++)
        {
            // Read 16 Byte serial number
            OutgoingBuffer[Index++] = InstrumentInfo.SerialNumber[LoopCounter];
        }
        //Populate length Byte left earlier
        OutgoingBuffer[NETWORK_CONFG_LENGTH_BYTE] = SERIAL_NUMBER_LENGTH + ONE_BYTE_LENGTH;
        break;
        
    case MANUFACTURING_DATE:
        for (LoopCounter = 0; LoopCounter < DATE_LENGTH ;LoopCounter++)
        {
            // Read 4 Byte Mfg date
            OutgoingBuffer[Index++] = InstrumentInfo.ManufacturingDate[LoopCounter];
        }
        //Populate length Byte left earlier
        OutgoingBuffer[NETWORK_CONFG_LENGTH_BYTE] = DATE_LENGTH + ONE_BYTE_LENGTH;
        break;
        
    case INST_PART_NUMBER:
        for (LoopCounter = 0; LoopCounter < PART_NUMBER_LENGTH ;LoopCounter++)
        {
            // Read 16 Byte Part number
            OutgoingBuffer[Index++] = InstrumentInfo.PartNumber[LoopCounter];
        }
        //Populate length Byte left earlier
        OutgoingBuffer[NETWORK_CONFG_LENGTH_BYTE] = PART_NUMBER_LENGTH + ONE_BYTE_LENGTH;
        break;
        
    case TECHNICIAN_INITIALS:
        for (LoopCounter = 0; LoopCounter < TECH_INITIALS_LENGTH ;LoopCounter++)
        {
            // Read 4 Byte Technician initials 
            OutgoingBuffer[Index++] =  InstrumentInfo.TechniciansInitials[LoopCounter];
        }
        //Populate length Byte left earlier
        OutgoingBuffer[NETWORK_CONFG_LENGTH_BYTE] = TECH_INITIALS_LENGTH + ONE_BYTE_LENGTH;
        break;
        
    case JOB_NUMBER:
        for (LoopCounter = 0; LoopCounter < JOB_NUMBER_LENGTH ;LoopCounter++)
        {
            // Read 8 Byte Job number
            OutgoingBuffer[Index++] =  InstrumentInfo.JobNumber[LoopCounter];
        }
        //Populate length Byte left earlier
        OutgoingBuffer[NETWORK_CONFG_LENGTH_BYTE] = JOB_NUMBER_LENGTH + ONE_BYTE_LENGTH;
        break;
        
    case HARDWARE_VERSION:
        // Read 1 Byte Hardware version
        OutgoingBuffer[Index++] = InstrumentInfo.HardwareVersion;
        //Populate length Byte left earlier
        OutgoingBuffer[NETWORK_CONFG_LENGTH_BYTE] = 1 + ONE_BYTE_LENGTH;
        break;
        
    case BATTERY_TYPE:   
        // Read 1 Byte Network Encryption Type
        OutgoingBuffer[Index++] = InstrumentInfo.BatteryType;
        //Populate length Byte left earlier
        OutgoingBuffer[NETWORK_CONFG_LENGTH_BYTE] = 1 + ONE_BYTE_LENGTH;
        break;
        
    case SIM_CARRIER:
        for (LoopCounter = 0; LoopCounter < CELLULAR_CARRIER_LENGTH ;LoopCounter++)
        {
            // Read 64 Byte Network PAssword
            OutgoingBuffer[Index++] =  gCellularDriver.carrier[LoopCounter];
        }
        //Populate length Byte left earlier
        OutgoingBuffer[LENGTH_BYTE] = CELLULAR_CARRIER_LENGTH + ONE_BYTE_LENGTH;
        break;
      
    case SIM_ICCID:
        for (LoopCounter = 0; LoopCounter < CELLULAR_ICCID_LENGTH ;LoopCounter++)
        {
            // Read 64 Byte Network PAssword
            OutgoingBuffer[Index++] =  gCellularDriver.iccid[LoopCounter];
        }
        //Populate length Byte left earlier
        OutgoingBuffer[LENGTH_BYTE] = CELLULAR_ICCID_LENGTH + ONE_BYTE_LENGTH;
        break;
       
        
    case MODULE_IMEI:
        for (LoopCounter = 0; LoopCounter < CELLULAR_IMEI_LENGTH ;LoopCounter++)
        {
            // Read 64 Byte Network PAssword
            OutgoingBuffer[Index++] =  gCellularDriver.imei[LoopCounter];
        }
        //Populate length Byte left earlier
        OutgoingBuffer[LENGTH_BYTE] = CELLULAR_IMEI_LENGTH + ONE_BYTE_LENGTH;
        break;
        
    case SIM_APN:
        for (LoopCounter = 0; LoopCounter < SIM_APN_LEN ;LoopCounter++)
        {
            // Read 64 Byte Network PAssword
            OutgoingBuffer[Index++] =  gCellularDriver.APN[LoopCounter];
        }
        //Populate length Byte left earlier
        OutgoingBuffer[LENGTH_BYTE] = SIM_APN_LEN + ONE_BYTE_LENGTH;
        break;
        
    case CELL_NUMBER_1:
        for (LoopCounter = 0; LoopCounter < PHONE_NUMBER_LEN ;LoopCounter++)
        {
            // Read 64 Byte Network PAssword
            OutgoingBuffer[Index++] =  gCellularDriver.cellNumber1[LoopCounter];
        }
        //Populate length Byte left earlier
        OutgoingBuffer[LENGTH_BYTE] = PHONE_NUMBER_LEN + ONE_BYTE_LENGTH;
        break;
        
    case CELL_NUMBER_2:
        for (LoopCounter = 0; LoopCounter < PHONE_NUMBER_LEN ;LoopCounter++)
        {
            // Read 64 Byte Network PAssword
            OutgoingBuffer[Index++] =  gCellularDriver.cellNumber2[LoopCounter];
        }
        //Populate length Byte left earlier
        OutgoingBuffer[LENGTH_BYTE] = PHONE_NUMBER_LEN + ONE_BYTE_LENGTH;
        break;
        
        
    case CELL_PHONE_NUMBER:
        // Coming Soon
        break;
        
    case CELL_ANTENNA_TYPE:
        OutgoingBuffer[Index++] =  gCellularDriver.antennaType;
        //Populate length Byte left earlier
        OutgoingBuffer[LENGTH_BYTE] = ONE_BYTE_LENGTH + ONE_BYTE_LENGTH;  
        break;
        
        
    }
    
    // Byte (5 + n) – (5 + n + 1) CheckSum of the message
    OutgoingBuffer[Index] = CalculateMessageChecksum(OutgoingBuffer, (unsigned short)2, (unsigned short)(Index-1));
    Index++;
    OutgoingBuffer[Index++] = 0x00;   
    
    // Byte  (5 + n + 2) – (5 + n + 3) End framing Character of the message 
    OutgoingBuffer[Index++] = END_FRAMING_CHARACTER;
    OutgoingBuffer[Index++] = END_FRAMING_CHARACTER; 
    
    returnLength = Index-1;
    return returnLength;
}


//==============================================================================
//
//  static void BuildNoRemainingMesage(uint8_t *OutgoingBuffer)
//
//   Author:   Tayyab Tahir
//   Date:     2018/04/23
//
//!  This function writes the no new message to the outgoing buffer   
//
//==============================================================================
static uint8_t BuildNoRemainingMesage(uint8_t *OutgoingBuffer)
{
    uint8_t Index = 0u;
    uint8_t returnLength = 0u;
    
    // Byte 0-1 Start framing Character of the message 
    OutgoingBuffer[Index++] = START_FRAMING_CHARACTER;
    OutgoingBuffer[Index++] = START_FRAMING_CHARACTER;
    
    // Byte 2 Message Type: Radio configure message
    OutgoingBuffer[Index++] = RESPOND_NO_MORE_MESSAGES;
    
    // Byte 3-4 End framing Character of the message 
    OutgoingBuffer[Index++] = END_FRAMING_CHARACTER;
    OutgoingBuffer[Index++] = END_FRAMING_CHARACTER; 
    
    returnLength = Index-1;
    return returnLength;
}


//==============================================================================
//
//  static void BuildCustomINETMessage(uint8_t *OutgoingBuffer)
//
//   Author:   Tayyab Tahir
//   Date:     2018/04/23
//
//!  This function writes the custom messagesent from INET to the outgoing buffer   
//
//==============================================================================
static uint8_t BuildCustomINETMessage(uint8_t *OutgoingBuffer)
{
    uint8_t Index = 0u;
    uint8_t LoopCouter = 0u;
    uint8_t returnLength = 0u;
    
    // Byte 0-1 Start framing Character of the message 
    OutgoingBuffer[Index++] = START_FRAMING_CHARACTER;
    OutgoingBuffer[Index++] = START_FRAMING_CHARACTER;
    
    // Byte 2 Message Type: Radio configure message
    OutgoingBuffer[Index++] = RESPOND_TEXT_INET_MESSAGE;
    
    // Byte 3 Version of message/Config
    OutgoingBuffer[Index++] = InstrumentInfo.MessgaeVersion;
    
    // Byte 4 Device (Smart battery)type (Wifi/Cell)
    OutgoingBuffer[Index++] = InstrumentInfo.BatteryType;
    
    // Byte 5 Payload length
    OutgoingBuffer[Index++] = CUSTOM_MSG_LENGTH + ONE_BYTE_LENGTH + ONE_BYTE_LENGTH; //1 Byte for Payload Length , 1Byte for the instrument alarm 
    
    // Byte 6-50 Custom message from Inet 
    for(LoopCouter=0; LoopCouter < CUSTOM_MSG_LENGTH;LoopCouter++)
    {
        OutgoingBuffer[Index++] = InstrumentInfo.INETCustomMessage[LoopCouter];
    }
    
    //Byte 51 Alarm for the Master to go in 
    OutgoingBuffer[Index++] = 0x00; //.Alarm type from inet to set on vpro
    
    // Byte 52-53 CheckSum of the message
    OutgoingBuffer[Index] = CalculateMessageChecksum(OutgoingBuffer, (unsigned short)2, (unsigned short)(Index-1));
    Index++;
    OutgoingBuffer[Index++] = 0x00;
    
    // Byte 54-55 End framing Character of the message 
    OutgoingBuffer[Index++] = END_FRAMING_CHARACTER;
    OutgoingBuffer[Index++] = END_FRAMING_CHARACTER;   
    
    returnLength = Index-1;
    return returnLength;
}


//==============================================================================
//
//static uint8_t CalculateMessageChecksum(
//                                              uint8_t *buffer,     
//                                              unsigned short startIndex,
//                                              unsigned short endIndex)                                           
//   Author:   Tayyab Tahir
//   Date:     2018/04/23
//
//!  This function calculate the checksums for outgoing as well as incoming messages
//
//==============================================================================
static uint8_t CalculateMessageChecksum(
                                        uint8_t *buffer,     //!< Buffer for which checksum is being calculated
                                        uint16_t startIndex, //!< Start index for checksum calculation
                                        uint16_t endIndex    //!< End index for checksum calculation 
                                            )
{
    // Variable for calculating the checksum, Making it short for the starup so
    uint8_t calculatedChecksum = 0u;
    int32_t index = 0;
    
    // Calculate the checksum (by summing up all bytes)
    for (index = (int)startIndex; index < (int)endIndex; index++)
    {
        // Add the values in calculated checksum
        calculatedChecksum = (calculatedChecksum + buffer[index]);
    }
    return (calculatedChecksum);   
}

//==============================================================================
//
//static uint8_t CalculateMessageChecksum(void)
//
//   Author:   Dilawar Ali
//   Date:     2018/04/29
//
//!  This function create communication event when data received from instrument has changed
//
//==============================================================================
static void CreateInstrumentEvent(void)
{
    RTOS_ERR  err;
    
    uint8_t loopCounter = 0;
    BOOLEAN isInfoChanged = false;
    PTR_COMM_EVT_t commEvt = NULL;
    CellMsg_t *msg;
    SysMsg_t  *sysMsg;
    
    static BOOLEAN isFirstMessage = true;
    uint32_t secondsSince1970 = 0;
    static uint32_t eventsCreated = 0;
    if (gCellularDriver.isRTCTimeUpdated == true)
    {
        secondsSince1970 = RTCDRV_GetWallClock();
        
        
        if (isFirstMessage == true)
        {
            if(RemoteUnit.SensorsInfo.numberOfSensors > 0)
            {
                isFirstMessage = false;
                isInfoChanged = true;
                //Update the time when first event is created 
                InstrumentInfo.LastPeriodicMessageTimeStamp = secondsSince1970;
            }
        }
        else
        {
            if(RemoteUnit.InstrumentState != RemoteUnit.InternalInstrumentState)
            {
                isInfoChanged = true;
            }
            else if((strcmp((char const*)referenceUserName, (char const*)RemoteUnit.UserName) != 0) || (strcmp((char const*)referenceSiteName, (char const*)RemoteUnit.SiteName) != 0))
            {
                isInfoChanged = true;
            }
            else
            {
                for(loopCounter = 0; loopCounter < RemoteUnit.SensorsInfo.numberOfSensors; loopCounter++)
                {
                    if(RemoteUnit.SensorsInfo.sensorArray[loopCounter].SensorStatus != RemoteUnit.SensorsInfo.sensorArray[loopCounter].InternalSensorStatus)
                    {
                        isInfoChanged = true;
                        break;
                    }
                }
            }
        }
        
        // Create Periodic events after specified interval of time 
        if((IS_PERIODIC_EVENT_ENABLED == true) &&
           (false == isInfoChanged) && //No New Event is present
               (RemoteUnit.InstrumentType != INSTRUMENT_TYPE_DEFAULT) && //Valid instrument communicating check
                   ( InstrumentInfo.InetStatus == INET_ACCEPTS_INSTRUMENT_AND_MONITORING) && //Inet connection is avalible 
                       ( InstrumentInfo.SmartBatteryStatus == RUNNING_NORMAL) && //Wifi is running Normal
                           ((secondsSince1970 - InstrumentInfo.LastPeriodicMessageTimeStamp) > PERIODIC_INET_MESSSAGE_INTERVAL) //Specified interval has been passed since last interval 
                               )
        {
            //Update the time when periodic event is created 
            InstrumentInfo.LastPeriodicMessageTimeStamp = secondsSince1970;
            //Update the flag to signal event creation 
            isInfoChanged = true;
        }
        
        if(isInfoChanged == true)
        {
            strncpy((char *)referenceUserName, (char const*)RemoteUnit.UserName, SERIAL_NUMBER_LENGTH);
            strncpy((char *)referenceSiteName, (char const*)RemoteUnit.SiteName, SERIAL_NUMBER_LENGTH);
            RemoteUnit.InternalInstrumentState = RemoteUnit.InstrumentState;
            for(loopCounter = 0; loopCounter < RemoteUnit.SensorsInfo.numberOfSensors; loopCounter++)
            {
                RemoteUnit.SensorsInfo.sensorArray[loopCounter].InternalSensorStatus = RemoteUnit.SensorsInfo.sensorArray[loopCounter].SensorStatus;
            }
            
            commEvt = GetEventMessageFromPool();
            if(commEvt != NULL)
            {
                GetCurrentTimeAndDate(&commEvt->dateTimeInfo);
                
                commEvt->commEvtType = INSTRUMENT_DATA_UPLOAD;
                commEvt->InstrumentState = (INSTRUMENT_STATUS_t)RemoteUnit.InstrumentState;
                commEvt->instSensorInfo.numberOfSensors = RemoteUnit.SensorsInfo.numberOfSensors;
                
                for(loopCounter = 0; loopCounter < RemoteUnit.SensorsInfo.numberOfSensors; loopCounter++)
                {
                    commEvt->instSensorInfo.sensorArray[loopCounter].componentCode        = RemoteUnit.SensorsInfo.sensorArray[loopCounter].componentCode;
                    commEvt->instSensorInfo.sensorArray[loopCounter].DecimalPlaces        = RemoteUnit.SensorsInfo.sensorArray[loopCounter].DecimalPlaces;
                    commEvt->instSensorInfo.sensorArray[loopCounter].SensorMeasuringUnits = RemoteUnit.SensorsInfo.sensorArray[loopCounter].SensorMeasuringUnits;
                    commEvt->instSensorInfo.sensorArray[loopCounter].SensorReadingHigh    = RemoteUnit.SensorsInfo.sensorArray[loopCounter].SensorReadingHigh;
                    commEvt->instSensorInfo.sensorArray[loopCounter].SensorReadingLow     = RemoteUnit.SensorsInfo.sensorArray[loopCounter].SensorReadingLow;
                    commEvt->instSensorInfo.sensorArray[loopCounter].SensorStatus         = RemoteUnit.SensorsInfo.sensorArray[loopCounter].SensorStatus;
                    commEvt->instSensorInfo.sensorArray[loopCounter].SensorType           = RemoteUnit.SensorsInfo.sensorArray[loopCounter].SensorType;
                }
                
                commEvt->sequenceNumber = GetNextSequence();
                commEvt->GPSLocationInfo.isGpsValid = GPSReceivedCoordinates.isGpsValid;
                commEvt->GPSLocationInfo.latitude = GPSReceivedCoordinates.latitude;
                commEvt->GPSLocationInfo.latitudeDir = GPSReceivedCoordinates.latitudeDir;
                commEvt->GPSLocationInfo.longitude = GPSReceivedCoordinates.longitude;
                commEvt->GPSLocationInfo.longitudeDir = GPSReceivedCoordinates.longitudeDir;
                
                
                //Add comm Event to event Queue
                OSQPost(&eventMessagesQueue, commEvt, sizeof(ComEvent_t), OS_OPT_POST_FIFO + OS_OPT_POST_ALL + OS_OPT_POST_NO_SCHED, &err);
                
                // Post message to cellualr Task to send Event to iNet
                msg = (CellMsg_t*)GetTaskMessageFromPool();
                if(msg != NULL)
                {
                    eventsCreated++;
                    msg->msgId = CELL_SEND_EVENT_TO_INET;
                    msg->msgInfo = 1;
                    msg->ptrData = NULL;
                    OSTaskQPost(&CellTaskTCB, (void *)msg, sizeof(SysMsg_t), OS_OPT_POST_FIFO, &err);
                }
            }
        }
        
        if(RemoteUnit.InstrumentState == 4)
        {
            // Post message to cellualr Task to send Event to iNet
            sysMsg = GetTaskMessageFromPool();
            if(sysMsg != NULL)
            {
                eventsCreated++;
                sysMsg->msgId = DEVICE_SHUTDOWN_MSG;
                sysMsg->msgInfo = 1;
                sysMsg->ptrData = NULL;
                OSTaskQPost(&SYSTaskTCB, (void *)sysMsg, sizeof(SysMsg_t), OS_OPT_POST_FIFO, &err);
            }
        } 
        
    }
    
}


//==============================================================================
//
//   static void ProcessFirmwareHeader(uint8_t *IncomingBuffer, uint8_t StartingIndex)
//
//   Author:   Dilawar Ali
//   Date:     2018/09/13
//
//!  This function process the header data of firmware file sent from instrument
//
//==============================================================================
static void ProcessFirmwareHeader(uint8_t *IncomingBuffer, uint8_t StartingIndex)
{
    uint8_t CalculatedChecksum = 0u;
    uint8_t ReceivedChecksum = 0u;
    uint8_t payLoadSize = 0;
    
    fwFile.returnMessage = NACK;
    
    
    fwFile.currentSequenceNumber = 0;
    fwFile.previousSequenceNumber = 0;
    
    //Get Payload size from message
    payLoadSize = IncomingBuffer[StartingIndex + 5];
    //Get checksum sent in message
    ReceivedChecksum = IncomingBuffer[StartingIndex + 5 + payLoadSize];
    //Calculate checksum of payload in message
    CalculatedChecksum = (CalculateMessageChecksum(IncomingBuffer, (unsigned short)(StartingIndex+2), (unsigned short)(StartingIndex + 4 + payLoadSize + 1)) - 1 );
    //check either checksum match or not
    if(ReceivedChecksum == CalculatedChecksum)
    {
        // Get Device type from message for which file is sent
        fwFile.deviceType = (WIRELESS_DEVICE_TYPE_t) IncomingBuffer[4];
        // Check is cellular file is sent or not
        if(fwFile.deviceType == WIRELESS_DEVICE_CELLULAR)
        {
            // File sent is for cellular then acknowledge the message
            fwFile.returnMessage        = ACK;
            fwFile.errorType            = NO_ERROR;
            // Get the typr of file sent from instrument
            fwFile.fileType             = (FIRMWARE_FILE_TYPE_ENUM) IncomingBuffer[6];
            // Get the total number of packet which will sent during firmware update
            fwFile.totalDataPackets     = (uint32_t) (((uint8_t)IncomingBuffer[7])  | (((uint8_t)IncomingBuffer[8]) << 8)  | (((uint8_t)IncomingBuffer[9]) << 16)  | (((uint8_t)IncomingBuffer[10]) << 24));
            // Get CRC of compelete file being sent from instrument
            fwFile.fileCRC              = (uint32_t) (((uint8_t)IncomingBuffer[11]) | (((uint8_t)IncomingBuffer[12]) << 8) | (((uint8_t)IncomingBuffer[13]) << 16) | (((uint8_t)IncomingBuffer[14]) << 24));
            fwFile.remainingDataPackets = fwFile.totalDataPackets;
            fwFile.expectedPacketType   = MAIN_HEADER_PACKET;
        }
        else
        {
            // Declare file not supported if it is not for cellular
            fwFile.errorType = DEVICE_NOT_SUPPORTED;
        }
        
    }
    else
    {
        // through incalid CRC error if CRC is not matched
        fwFile.errorType = INVALID_CRC;
    }
}

//==============================================================================
//
//   static void ProcessFirmwareData(uint8_t *IncomingBuffer, uint8_t StartingIndex)
//
//   Author:   Dilawar Ali
//   Date:     2018/09/13
//
//!  This function process the binary data of firmware file sent from instrument
//
//==============================================================================
static void ProcessFirmwareData(uint8_t *IncomingBuffer, uint8_t StartingIndex)
{
    uint8_t CalculatedChecksum = 0u;
    uint8_t ReceivedChecksum = 0u;
    uint8_t payLoadSize = 0;
    uint8_t loopCounter = 0;
    
    static uint32_t ReceivedDataPackets = 0;
    
    SysMsg_t *msg = NULL;
    RTOS_ERR  err;
    
    fwFile.returnMessage = NACK;

    //Get payload Size
    payLoadSize = IncomingBuffer[StartingIndex + 5];
    // Get sent checksum of message
    ReceivedChecksum = IncomingBuffer[StartingIndex + 5 + payLoadSize];
    //Calculate checksum of payload sent
    CalculatedChecksum = (CalculateMessageChecksum(IncomingBuffer, (unsigned short)(StartingIndex+2), (unsigned short)(StartingIndex + 4 + payLoadSize + 1)) - 1 );
    // Verify both checksum match
    if(ReceivedChecksum == CalculatedChecksum)
    {
        // check deveice type sent
        fwFile.deviceType = (WIRELESS_DEVICE_TYPE_t) IncomingBuffer[4];
        
        // Verify is it for cellular battery pack or not
        if(fwFile.deviceType == WIRELESS_DEVICE_CELLULAR)
        {
            // Acknowledge the Message
            fwFile.returnMessage         = ACK;
            fwFile.errorType             = NO_ERROR;
            
            // Get file type being sent now
            fwFile.fileType              = (FIRMWARE_FILE_TYPE_ENUM) IncomingBuffer[6];
            // Get currnet sequence number being sent
            fwFile.currentSequenceNumber = (uint32_t) (((uint8_t)IncomingBuffer[7])  | (((uint8_t)IncomingBuffer[8]) << 8)  | (((uint8_t)IncomingBuffer[9]) << 16)  | (((uint8_t)IncomingBuffer[10]) << 24));
            
            // If received sequence number is greater than previous then valid data packet is received
            if(fwFile.currentSequenceNumber > fwFile.previousSequenceNumber)
            {
                ReceivedDataPackets++;
                fwFile.bufferDataSize = 0;
                // Copy data from message to buffer so that it can write to flash
                for(loopCounter = 0; ((loopCounter < (payLoadSize - 5)) && (loopCounter < FW_BUFFER_SIZE)); loopCounter++)
                {
                    fwFile.fwBuffer[loopCounter] = IncomingBuffer[11 + loopCounter];
                    fwFile.bufferDataSize++;
                }
                
                //Update Sequence number
                fwFile.previousSequenceNumber = fwFile.currentSequenceNumber;
                
                // Create the message for systask to write data to data flash
                msg = (SysMsg_t*)GetTaskMessageFromPool();
                if(msg != NULL)
                {
                    msg->msgId = WRITE_DATA_TO_FLASH;
                    msg->msgInfo = 1;
                    msg->ptrData = NULL;
                    // Post message to Systask
                    OSTaskQPost(&SYSTaskTCB, (void *)msg, sizeof(SysMsg_t), OS_OPT_POST_FIFO, &err);
                }
            }
            
        }
        else
        {
            // Declare file is not supported
            fwFile.errorType = DEVICE_NOT_SUPPORTED;
        }
        
    }
    else
    {
        // Declare CRC is not valid
        fwFile.errorType = INVALID_CRC;
    }
}


//==============================================================================
//
//   static void ProcessFirmwareFooter(uint8_t *IncomingBuffer, uint8_t StartingIndex)
//
//   Author:   Dilawar Ali
//   Date:     2018/09/13
//
//!  This function process the Footer data of firmware file sent from instrument
//
//==============================================================================
static void ProcessFirmwareFooter(uint8_t *IncomingBuffer, uint8_t StartingIndex)
{
    uint8_t CalculatedChecksum = 0u;
    uint8_t ReceivedChecksum = 0u;
    uint8_t payLoadSize = 0;
    
    fwFile.returnMessage = NACK;
    
    
    fwFile.currentSequenceNumber = 0;
    fwFile.previousSequenceNumber = 0;
    
    //Get Payload size from message
    payLoadSize = IncomingBuffer[StartingIndex + 5];
    //Get checksum sent in message
    ReceivedChecksum = IncomingBuffer[StartingIndex + 5 + payLoadSize];
    //Calculate checksum of payload in message
    CalculatedChecksum = (CalculateMessageChecksum(IncomingBuffer, (unsigned short)(StartingIndex+2), (unsigned short)(StartingIndex + 4 + payLoadSize + 1)) - 1 );
    //check either checksum match or not
    if(ReceivedChecksum == CalculatedChecksum)
    {
        // Get Device type from message for which file is sent
        fwFile.deviceType = (WIRELESS_DEVICE_TYPE_t) IncomingBuffer[4];
        // Check is cellular file is sent or not
        if(fwFile.deviceType == WIRELESS_DEVICE_CELLULAR)
        {
            // File sent is for cellular then acknowledge the message
            fwFile.returnMessage        = ACK;
            fwFile.errorType            = NO_ERROR;
            // Get the typr of file sent from instrument
            fwFile.fileType             = (FIRMWARE_FILE_TYPE_ENUM) IncomingBuffer[6];
            // Get the total number of packet which will sent during firmware update
            fwFile.totalDataPackets     = (uint32_t) (((uint8_t)IncomingBuffer[7])  | (((uint8_t)IncomingBuffer[8]) << 8)  | (((uint8_t)IncomingBuffer[9]) << 16)  | (((uint8_t)IncomingBuffer[10]) << 24));
            // Get CRC of compelete file being sent from instrument
            fwFile.fileCRC              = (uint32_t) (((uint8_t)IncomingBuffer[11]) | (((uint8_t)IncomingBuffer[12]) << 8) | (((uint8_t)IncomingBuffer[13]) << 16) | (((uint8_t)IncomingBuffer[14]) << 24));
        }
        else
        {
            // Declare file not supported if it is not for cellular
            fwFile.errorType = DEVICE_NOT_SUPPORTED;
        }
        
    }
    else
    {
        // through incalid CRC error if CRC is not matched
        fwFile.errorType = INVALID_CRC;
    }
}

//==============================================================================
//
//  static void BuildFirmwareAckNackMessage(uint8_t *OutgoingBuffer)
//
//   Author:   Tayyab Tahir
//   Date:     2018/08/41
//
//!  This function writes the firmware ACK NACK message to the outgoing buffer   
//
//==============================================================================
static uint8_t BuildFirmwareAckNackMessage(uint8_t *OutgoingBuffer)
{
  
  uint8_t Index = 0u;
  uint8_t returnLength = 0u;
  
  // Byte 0-1 Start framing Character of the message 
  OutgoingBuffer[Index++] = START_FRAMING_CHARACTER;
  OutgoingBuffer[Index++] = START_FRAMING_CHARACTER;
  
  // Byte 2 Message Type: Firmware ACK NACK message
  OutgoingBuffer[Index++] = RESPOND_ACK_NACK_FIRMWARE_MESSAGE;
  
  // Byte 3 ACK/NACK
  OutgoingBuffer[Index++] = fwFile.returnMessage;
  
  // Byte 4 Error Type
  OutgoingBuffer[Index++] = fwFile.errorType;
  
  // Byte 5 Payload Length
  OutgoingBuffer[Index++] = 16;
    
  // Byte 6 File Type
  OutgoingBuffer[Index++] = fwFile.fileType;
  
  // Byte 7-10 Sequence Number
  OutgoingBuffer[Index++] = (fwFile.currentSequenceNumber)      & 0xFF;
  OutgoingBuffer[Index++] = (fwFile.currentSequenceNumber >> 8) & 0xFF;
  OutgoingBuffer[Index++] = (fwFile.currentSequenceNumber >> 16)& 0xFF;
  OutgoingBuffer[Index++] = (fwFile.currentSequenceNumber >> 24)& 0xFF;
  
  // Byte 11-14 Sequence Number
  OutgoingBuffer[Index++] = 0;
  OutgoingBuffer[Index++] = 0;
  OutgoingBuffer[Index++] = 0;
  OutgoingBuffer[Index++] = fwFile.errorType;
  
  // Byte 15-18 Sequence Number
  OutgoingBuffer[Index++] = (fwFile.currentSequenceNumber)      & 0xFF;
  OutgoingBuffer[Index++] = (fwFile.currentSequenceNumber >> 8) & 0xFF;
  OutgoingBuffer[Index++] = (fwFile.currentSequenceNumber >> 16)& 0xFF;
  OutgoingBuffer[Index++] = (fwFile.currentSequenceNumber >> 24)& 0xFF;
  
  // Byte 19-20 CheckSum of the message
  OutgoingBuffer[Index] = CalculateMessageChecksum(OutgoingBuffer, (unsigned short)2, (unsigned short)(Index-1));
  Index++;
  OutgoingBuffer[Index++] = 0x00;  
  
  // Byte 21-22 End framing Character of the message 
  OutgoingBuffer[Index++] = END_FRAMING_CHARACTER;
  OutgoingBuffer[Index++] = END_FRAMING_CHARACTER; 
  
  returnLength = Index-1;
  return returnLength;
}