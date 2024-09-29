#ifndef __EVENT_H
#define __EVENT_H
#include <stdbool.h>

#include <SysTask.h>
#include <SPI_Comm.h>
#include <main.h>


#define MAX_ALLOWED_INSTRUMENT_SERIAL_NUMBER            16u
#define MAX_ALLOWED_USER_NAME                           16u
#define MAX_ALLOWED_SITE_NAME                           16u
#define MAX_ALLOWED_MSG_SIZE                            128u
#define NETWORK_PASSWORD_LENGTH                         20u

#define PUT_EVENT_TO_COMM_QUEUE(ptrToEvent)             PutOneEventToQueueTail(eventQueueHandle,ptrToEvent);
#define PUT_EVENT_TO_COMM_QUEUE_HEAD(ptrToEvent)        PutOneEventToQueueHead(eventQueueHandle,ptrToEvent);

#define READ_EVENT_FROM_COMM_QUEUE()                    RaedOneEventFromQueue(eventQueueHandle)
#define GET_EVENT_FROM_COMM_QUEUE()                     GetOneEventFromQueue(eventQueueHandle)

#define MAX_SENSORS_PER_INSTRUMENT              7u


#define MAX_ALLOWED_INSTRUMENT_SERIAL_NUMBER    16u
#define MAX_ALLOWED_USER_NAME                   16u
#define MAX_ALLOWED_SITE_NAME                   16u
#define MAX_ALLOWED_MSG_SIZE                    128u



typedef enum
{
    INSTRUMENT_NORMAL   = 0,
    PANIC               = 1,
    SHUTDOWN            = 2,
    MANDOWN             = 3,
    PUMP_FAULT          = 4,
    LOW_BATTERY         = 5,
    INSTRUMENT_TYPE     = 6,//(Instrument type; Local = 0, Remote = 1)
    INSTRUMENT_CHARGING = 7  
}
INSTRUMENT_STATUS_t;

typedef enum
{
    INSTRUMENT_TYPE_DEFAULT     = 0u,
    INSTRUMENT_TYPE_VPRO        = 0x12u,
    INSTRUMENT_TYPE_SAFECORE    = 0x13u
}INSTRUMENT_TYPE_t;

typedef enum 
{
    MEAS_UNITS_INVALID          = 0,//!< This constant is used for invalid gas
    MEAS_UNITS_PPM              = 1,//!< This constant is used for gas in ppm units 
    MEAS_UNITS_VOL              = 2,//!< This constant is used for gas in percentage volume units 
    MEAS_UNITS_LEL              = 3,//!< This constant is used for gas in perecntage lel units 
    
}MEASUREMENT_UNITS_t;

typedef struct
{
    uint8_t SerialNumber[SERIAL_NUMBER_LENGTH + 1];
    uint8_t ManufacturingDate[DATE_LENGTH + 1];
    uint8_t PartNumber[PART_NUMBER_LENGTH + 1];
    uint8_t TechniciansInitials[TECH_INITIALS_LENGTH + 1];
    uint8_t JobNumber[JOB_NUMBER_LENGTH + 1];
    uint8_t HardwareVersion;

    uint8_t BatteryType;
    
    uint8_t MessgaeVersion;
    SMART_BATTERY_STATUS_t SmartBatteryStatus;
    INET_STATUS_t InetStatus;
    uint8_t CloudMessageStatus;
    uint8_t IsINETMessageAvalibleforVpro;
    uint8_t INETCustomMessage[CUSTOM_MSG_LENGTH];
    uint64_t LastPeriodicMessageTimeStamp;
}InstInfo_t;

typedef struct
{
    WIRELESS_DEVICE_TYPE_t devType;
    uint8_t CellOperator[32];
    int8_t WirelessRSSI;
}WirelessInfo_t;
/*
typedef struct
{
uint8_t sensorStatus;
float sensorReading;
MEASUREMENT_UNITS_t measurementUnit;
uint8_t componentCode;
uint8_t gasCode;
}SensorInfo_t;
*/
/*
typedef struct 
{
uint8_t numberOfSensors;
SensorInfo_t sensorArray[MAX_SENSORS_PER_INSTRUMENT];
}InstSensorInfo_t;
*/
typedef enum
{
    EVENT_NONE = 0,
    EVENT_COMM_EVT,
}EVENT_ID_t;

typedef struct
{
    //    Queue_Elem elem;
    EVENT_ID_t id;
}Event_t;

typedef enum
{
    GET_INET_TOKEN        = 0,
    REGISTER_INST_ON_INET = 1,
    INSTRUMENT_DATA_UPLOAD,
    LAST_IINVALID_EVENT,
}COMM_EVT_TYPE_t;


typedef struct
{
    Event_t evtObj;
    COMM_EVT_TYPE_t commEvtType;
    uint8_t sequenceNumber;
    INSTRUMENT_STATUS_t InstrumentState;
    InstSensorInfo_t instSensorInfo;
    GPSInfo_t GPSLocationInfo;
    DateTimeInfo_t dateTimeInfo;
}ComEvent_t;


typedef struct
{
    Event_t evtObj;
}SysEvent_t;

typedef struct
{
    Event_t evtObj;
    uint8_t Msg[MAX_ALLOWED_MSG_SIZE];
}MsgEvent_t;

typedef Event_t*  PTR_EVENT_t;

typedef ComEvent_t* PTR_COMM_EVT_t;


extern GPSInfo_t GPSReceivedCoordinates;;
uint8_t GetNextSequence(void);




void CreatMemmoryPoleForQueue(void);

int8_t intializeEvent(PTR_EVENT_t ptrEvent,EVENT_ID_t id);

extern InstInfo_t InstrumentInfo;


//------------------------------------------------------------------------------
//  SysMsg_t* GetTaskMessageFromPool(void)
//
//   Author:  Dilawar Ali
//   Date:    2018/05/22
//
//!  This function Get the memory for TASK message from its memory pool
//
//------------------------------------------------------------------------------
SysMsg_t* GetTaskMessageFromPool(void);

//------------------------------------------------------------------------------
//   void ReturnTaskMessageToPool(SysMsg_t* msg)
//
//   Author:  Dilawar Ali
//   Date:    2018/05/22
//
//!  This function Returns the TASK message memory to its memory pool
//
//------------------------------------------------------------------------------
void ReturnTaskMessageToPool(SysMsg_t* msg);

//------------------------------------------------------------------------------
//  ComEvent_t* GetEventMessageFromPool(void)
//
//   Author:  Dilawar Ali
//   Date:    2018/05/22
//
//!  This function Get the memory for Event message from its memory pool
//
//------------------------------------------------------------------------------
ComEvent_t* GetEventMessageFromPool(void);

//------------------------------------------------------------------------------
//  void ReturnEventMessageToPool(ComEvent_t* msg, BOOLEAN isEventSent)
//
//   Author:  Dilawar Ali
//   Date:    2018/05/22
//
//!  This function returns the Event message memory to its memory pool
//
//------------------------------------------------------------------------------
void ReturnEventMessageToPool(ComEvent_t* msg, BOOLEAN isEventSent);



#endif /* __EVENT_H */