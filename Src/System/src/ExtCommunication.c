//==============================================================================
//
//  ExtCommunication.c
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
//  Source:        ExtCommunication.c
//
//  Project:       Cornell
//
//  Author:        Dilawar Ali
//
//  Date:          2018/04/23
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

#include <stdio.h>
#include <time.h>
#include <string.h>
#include <Math.h>

#include "ExtCommunication.h"
#include "Event.h"
#include "jsmn.h"
#include "SPI_Comm.h"
#include "Timer.h"
#include "Cellular.h"

//==============================================================================
//  CONSTANTS, TYPEDEFS AND MACROS 
//==============================================================================
const uint8_t inetwasdev1Cert[] =
//https://inetnowstg.indsci.com
"-----BEGIN CERTIFICATE-----\n"
"MIIDxTCCAq2gAwIBAgIBADANBgkqhkiG9w0BAQsFADCBgzELMAkGA1UEBhMCVVMx\n"
"EDAOBgNVBAgTB0FyaXpvbmExEzARBgNVBAcTClNjb3R0c2RhbGUxGjAYBgNVBAoT\n"
"EUdvRGFkZHkuY29tLCBJbmMuMTEwLwYDVQQDEyhHbyBEYWRkeSBSb290IENlcnRp\n"
"ZmljYXRlIEF1dGhvcml0eSAtIEcyMB4XDTA5MDkwMTAwMDAwMFoXDTM3MTIzMTIz\n"
"NTk1OVowgYMxCzAJBgNVBAYTAlVTMRAwDgYDVQQIEwdBcml6b25hMRMwEQYDVQQH\n"
"EwpTY290dHNkYWxlMRowGAYDVQQKExFHb0RhZGR5LmNvbSwgSW5jLjExMC8GA1UE\n"
"AxMoR28gRGFkZHkgUm9vdCBDZXJ0aWZpY2F0ZSBBdXRob3JpdHkgLSBHMjCCASIw\n"
"DQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBAL9xYgjx+lk09xvJGKP3gElY6SKD\n"
"E6bFIEMBO4Tx5oVJnyfq9oQbTqC023CYxzIBsQU+B07u9PpPL1kwIuerGVZr4oAH\n"
"/PMWdYA5UXvl+TW2dE6pjYIT5LY/qQOD+qK+ihVqf94Lw7YZFAXK6sOoBJQ7Rnwy\n"
"DfMAZiLIjWltNowRGLfTshxgtDj6AozO091GB94KPutdfMh8+7ArU6SSYmlRJQVh\n"
"GkSBjCypQ5Yj36w6gZoOKcUcqeldHraenjAKOc7xiID7S13MMuyFYkMlNAJWJwGR\n"
"tDtwKj9useiciAF9n9T521NtYJ2/LOdYq7hfRvzOxBsDPAnrSTFcaUaz4EcCAwEA\n"
"AaNCMEAwDwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8EBAMCAQYwHQYDVR0OBBYE\n"
"FDqahQcQZyi27/a9BUFuIMGU2g/eMA0GCSqGSIb3DQEBCwUAA4IBAQCZ21151fmX\n"
"WWcDYfF+OwYxdS2hII5PZYe096acvNjpL9DbWu7PdIxztDhC2gV7+AJ1uP2lsdeu\n"
"9tfeE8tTEH6KRtGX+rcuKxGrkLAngPnon1rpN5+r5N9ss4UXnT3ZJE95kTXWXwTr\n"
"gIOrmgIttRD02JDHBHNA7XIloKmf7J6raBKZV8aPEjoJpL1E/QYVN8Gb5DKj7Tjo\n"
"2GTzLH4U/ALqn83/B2gX2yKQOC16jdFU8WnjXzPKej17CuPKf1855eJ1usV2GDPO\n"
"LPAvTK33sefOT6jEm0pUBsV/fdUID+Ic/n4XuKxe9tQWskMJDE32p2u0mYRlynqI\n"
"4uJEvlz36hz1\n"
"-----END CERTIFICATE-----\n"
;

//==============================================================================
//  LOCAL DATA DECLARATIONS
//==============================================================================
static uint8_t iNetEventId[25];
//==============================================================================
//  LOCAL FUNCTION PROTOTYPES
//==============================================================================
static int32_t JSONCreateTokenAccess(uint8_t urlBuffer[], uint32_t urlBufferSize, uint8_t dataBuffer[], uint32_t dataBufferSize, PTR_COMM_EVT_t evt);
static int32_t JSONCreateInstrumentDataUpload(uint8_t urlBuffer[], uint32_t urlBufferSize, uint8_t dataBuffer[], uint32_t dataBufferSize, PTR_COMM_EVT_t evt);
static int32_t JSONCreateInstrumentRegister(uint8_t urlBuffer[], uint32_t urlBufferSize, uint8_t dataBuffer[], uint32_t dataBufferSize, PTR_COMM_EVT_t evt);
static int32_t JParseGetToken(uint8_t js_data[], uint32_t len, uint8_t tokenBuffer[], uint32_t tokenBufferLength, PTR_COMM_EVT_t evt);
static int32_t JParseInstrumentRegister(uint8_t js_data[], uint32_t len, uint8_t ResponseBuffer[], uint32_t ResponseBufferLength, PTR_COMM_EVT_t evt);
static int32_t JParseInstrumentDataUpload(uint8_t js_data[], uint32_t len, uint8_t ResponseBuffer[], uint32_t ResponseBufferLength, PTR_COMM_EVT_t evt);
static void JSONGpsDataConversion(float *latitude, float *longitude, char *latitudeDirection, char *longitudeDirection);


//==============================================================================
//  GLOBAL DATA DECLARATIONS
//==============================================================================
JasonCreatorAndParser_t jsonCreatorAndParser[LAST_IINVALID_EVENT] = 
{
    {
        JSONCreateTokenAccess,
        JParseGetToken,
    },
    {
        JSONCreateInstrumentRegister,
        JParseInstrumentRegister,
    },
    {
        JSONCreateInstrumentDataUpload,
        JParseInstrumentDataUpload,
    }
};
//==============================================================================
//  LOCAL FUNCTIONS IMPLEMENTATION
//==============================================================================

//------------------------------------------------------------------------------
//  int32_t JSONCreateTokenAccess(uint8_t urlBuffer[], uint32_t urlBufferSize, uint8_t dataBuffer[], uint32_t dataBufferSize)
//
//   Author:  Dilawar Ali
//   Date:    2018/04/20
//
//!  This function create JSON to get Token from iNet server
//
//------------------------------------------------------------------------------
static int32_t JSONCreateTokenAccess(uint8_t urlBuffer[], uint32_t urlBufferSize, uint8_t dataBuffer[], uint32_t dataBufferSize, PTR_COMM_EVT_t evt)
{
    int32_t ret = -1;
    //Update URL buffer of communication interface
    ret = snprintf((char *)urlBuffer, urlBufferSize, "/oauth2/endpoint/iNet/token\0");
    // Update data buffer of communication with event correspoding JSON data
    ret = snprintf((char *)dataBuffer, dataBufferSize, "grant_type=password&client_id=%s&client_secret=%s&username=%s&password=%s$\0", CLIENT_ID, CLIENT_SECRET, USERNAME, PASSWORD);
    
    return ret;
}

//------------------------------------------------------------------------------
//  static int32_t JSONCreateInstrumentDataUpload(uint8_t urlBuffer[], uint32_t urlBufferSize, uint8_t dataBuffer[], uint32_t dataBufferSize)
//
//   Author:  Dilawar Ali
//   Date:    2018/04/20
//
//!  This function create JSON For Instrument Join and Sensor Update event
//
//------------------------------------------------------------------------------
static int32_t JSONCreateInstrumentDataUpload(uint8_t urlBuffer[], uint32_t urlBufferSize, uint8_t dataBuffer[], uint32_t dataBufferSize, PTR_COMM_EVT_t evt)
{
    uint8_t time[30] = {0};
    int32_t size = -1;
    uint32_t index = 1, loopCounter = 0;
    uint8_t gasCode[10] = {0};
    uint8_t sensorCode[10] = {0};
    float gasReading = 0;
    int16_t gasUnitReading = 0;
    static uint8_t sequenceNumber = 0;
    PTR_COMM_EVT_t commEvt = (PTR_COMM_EVT_t)evt;
    //Buffer for sensor details
    uint8_t sensorsdata[Max_LEN_FOR_SENSOR_JSON * MAX_NUMBER_OF_SENSORS] = {0};
    
    //Update URL buffer of communication interface
    size = snprintf((char *)urlBuffer, urlBufferSize, "/iNetAPI/v1/live/create\0");
    
    //Update time buffer from event time
    snprintf((char *)time, 30, "%02u-%02u-%02uT%02u:%02u:%02u.000+0000", commEvt->dateTimeInfo.date.year, commEvt->dateTimeInfo.date.month, commEvt->dateTimeInfo.date.day, commEvt->dateTimeInfo.time.hours, commEvt->dateTimeInfo.time.minutes, commEvt->dateTimeInfo.time.seconds);
    
    
    
    sensorsdata[0] = '[';
    
    for(loopCounter = 0; loopCounter < commEvt->instSensorInfo.numberOfSensors; loopCounter++)
    {
        gasUnitReading = 0;
        // Get gas code of sensor
        sprintf((char *)gasCode, "G%04u", commEvt->instSensorInfo.sensorArray[loopCounter].SensorType);
        // Get Component code of server
        commEvt->instSensorInfo.sensorArray[loopCounter].componentCode = (commEvt->instSensorInfo.sensorArray[loopCounter].SensorMeasuringUnits) >> 4;
        sprintf((char *)sensorCode, "S%04u", commEvt->instSensorInfo.sensorArray[loopCounter].componentCode);
        
        gasUnitReading |= (int16_t)commEvt->instSensorInfo.sensorArray[loopCounter].SensorReadingHigh;
        gasUnitReading |= gasUnitReading << 8;
        gasUnitReading |= (int16_t)commEvt->instSensorInfo.sensorArray[loopCounter].SensorReadingLow;
        
        gasReading = (float)((gasUnitReading)/(pow(10, commEvt->instSensorInfo.sensorArray[loopCounter].DecimalPlaces)));
        
        // Update buffer with sensors details in JSON format
        //      index += sprintf((char *)&sensorsdata[index], "{\"componentCode\":\"%s\",\"gasCode\":\"%s\",\"uom\":%u,\"status\":%u,\"gasReading\":%2.4f},\0",sensorCode, gasCode, \
        //        commEvt->instSensorInfo.sensorArray[loopCounter].SensorMeasuringUnits, commEvt->instSensorInfo.sensorArray[loopCounter].SensorStatus, gasReading);
        index += sprintf((char *)&sensorsdata[index], "{\"gasCode\":\"%s\",\"uom\":%u,\"status\":%u,\"gasReading\":%2.4f},\0", gasCode, \
            commEvt->instSensorInfo.sensorArray[loopCounter].SensorMeasuringUnits, commEvt->instSensorInfo.sensorArray[loopCounter].SensorStatus, gasReading);
        
    }
    
    if (commEvt->instSensorInfo.numberOfSensors > 0)
    {
        // Remove last ','
        index--;
    }
    
    sensorsdata[index] = ']';
    
    // Update buffer with instrument details
    size = snprintf((char *)dataBuffer, dataBufferSize, "{\"device\":\"cellular\",\"sn\":\"%s\",\"time\":\"%s\",\"sequence\":%d,\"status\":%d,\"equipmentCode\": \"VPRO\",\"user\":\"%s\",\"site\":\"%s\"" ,RemoteUnit.SerialNumber, time, sequenceNumber++, commEvt->InstrumentState, RemoteUnit.UserName, RemoteUnit.SiteName);
    //@todo: device and equipment Code
    
    // When Valid gps co-ordinates are attached
    if(commEvt->GPSLocationInfo.isGpsValid == true)
    {
        // convert GPS data to accepted format of iNet
        JSONGpsDataConversion(&commEvt->GPSLocationInfo.latitude, &commEvt->GPSLocationInfo.longitude, (char *)&commEvt->GPSLocationInfo.latitudeDir, (char *)&commEvt->GPSLocationInfo.longitudeDir);
        // Add GPS position in JSON Data
        size += snprintf((char *)&dataBuffer[size], (dataBufferSize - size), ",\"position\":{\"latitude\":%4.6f,\"longitude\":%4.6f,\"accuracy\":%4.6f}", commEvt->GPSLocationInfo.latitude, commEvt->GPSLocationInfo.longitude,commEvt->GPSLocationInfo.horizantalDilution);
    }
    
    // Add Sensor Data in jSON data
    size += snprintf((char *)&dataBuffer[size], (dataBufferSize - size), ",\"sensors\":%s}$\0", sensorsdata );
    
    return size;
}

//------------------------------------------------------------------------------
//  static int32_t JSONCreateInstrumentRegister(uint8_t urlBuffer[], uint32_t urlBufferSize, uint8_t dataBuffer[], uint32_t dataBufferSize)
//
//   Author:  Dilawar Ali
//   Date:    2018/04/20
//
//!  This function create JSON For Instrument Join and Sensor Update event
//
//------------------------------------------------------------------------------
static int32_t JSONCreateInstrumentRegister(uint8_t urlBuffer[], uint32_t urlBufferSize, uint8_t dataBuffer[], uint32_t dataBufferSize, PTR_COMM_EVT_t evt)
{
    int32_t size = -1;
    //Update URL buffer of communication interface
    size = snprintf((char *)urlBuffer, urlBufferSize, "/iNetAPI/v1/live/%s/register", RemoteUnit.SerialNumber);
    
    return size;
}

//------------------------------------------------------------------------------
//  static void JSONGpsDataConversion(float *latitude, float *longitude, char *latitudeDirection, char *longitudeDirection)
//
//   Author:  Dilawar Ali
//   Date:    2018/04/20
//
//!  This function convert gps data received from settelite to gps data needed to iNet
//
//------------------------------------------------------------------------------
static void JSONGpsDataConversion(float *latitude, float *longitude, char *latitudeDirection, char *longitudeDirection)
{
    int localLatitude,localLongitude;
    float localF, localL;
    localF = *latitude/100.0f;
    localL = *longitude/100.0f;
    localLatitude = (int) localF;
    localLongitude = (int) localL;
    *latitude = (float) localLatitude +(100.0f * ( (localF - (float)localLatitude)/60.0f));
    *longitude = (float) localLongitude+(100.0f * ((localL - (float)localLongitude)/60.0f));
    // Conversion added
    if( *latitudeDirection == 'S' )
    {
        // Convert latitude
        *latitude =  ( (*latitude )* (-1) );
    }
    // Check logitude Direction
    if( *longitudeDirection == 'W' )
    {
        // Convert Longitude sign
        *longitude =  ( (*longitude )* (-1) );
    }
}


//------------------------------------------------------------------------------
//  void GetCurrentTimeAndDate(void)
//
//   Author:  Dilawar Ali
//   Date:    2018/04/23
//
//!  This function Get current time from RTC Clock
//
//------------------------------------------------------------------------------
void GetCurrentTimeAndDate(DateTimeInfo_t *currentTime)
{
    uint32_t secondsSince1970 = 0u;
    secondsSince1970 = RTCDRV_GetWallClock();
    
    struct tm *tms;
    tms = gmtime((time_t*)&secondsSince1970);
    
    // Synchronize system clock
    currentTime->date.month     = tms->tm_mon + 1; // struct tm monthId(0-11)
    currentTime->date.day       = tms->tm_mday;
    currentTime->date.year      = tms->tm_year + 1900;
    currentTime->time.hours     = tms->tm_hour;
    currentTime->time.minutes   = tms->tm_min;
    currentTime->time.seconds   = tms->tm_sec;
    
}

//------------------------------------------------------------------------------
//  static int32_t JParseGetToken(uint8_t js_data[], uint32_t len, uint8_t tokenBuffer[], uint32_t tokenBufferLength, PTR_COMM_EVT_t commEvt);
//
//   Author:  Dilawar Ali
//   Date:    2018/04/23
//
//!  This function Parse server response incase of Token event and update token buffer with valid received token
//
//------------------------------------------------------------------------------
static int32_t JParseGetToken(uint8_t js_data[], uint32_t len, uint8_t tokenBuffer[], uint32_t tokenBufferLength, PTR_COMM_EVT_t evt)
{
    jsmn_parser js;
    jsmntok_t js_tok[MAX_JSON_TOKEN_NUM];
    jsmn_init(&js);
    uint32_t size=0;
    int32_t status = -1;
    
    int numOfToken=(sizeof(js_tok)/sizeof(js_tok[0]));
    
    status=jsmn_parse(&js, (char const*)js_data, len, js_tok, numOfToken);
    if(status<0)
    {
        
    }
    else
    {
        for(uint32_t i=1; i<status;++i)
        {
            size=(js_tok[i].end-js_tok[i].start);
            if((0==strncmp((char const*)&js_data[js_tok[i].start],"access_token",size)) && (size !=0))
            {
                size=(js_tok[i+1].end-js_tok[i+1].start);
                snprintf((char *)tokenBuffer,tokenBufferLength,"Bearer %.*s",size,(char *)&js_data[js_tok[i+1].start]);
            }
            else if (( 0 == strncmp((char const*)&js_data[js_tok[i].start],"expires_in",size)) && (size != 0))
            {
                status = 0;
            }
            else
            {
                //Do Nothing
            }
        }
    }
    return status;
}

//------------------------------------------------------------------------------
//  static int32_t JParseInstrumentRegister(uint8_t js_data[], uint32_t len, uint8_t ResponseBuffer[], uint32_t ResponseBufferLength, PTR_COMM_EVT_t commEvt)
//
//   Author:  Dilawar Ali
//   Date:    2018/04/23
//
//!  This function Parse server response incase of Token event and update token buffer with valid received token
//
//------------------------------------------------------------------------------
static int32_t JParseInstrumentRegister(uint8_t js_data[], uint32_t len, uint8_t ResponseBuffer[], uint32_t ResponseBufferLength, PTR_COMM_EVT_t evt)
{
    return 0;
}

//------------------------------------------------------------------------------
//  static int32_t JParseInstrumentDataUpload(uint8_t js_data[], uint32_t len, uint8_t ResponseBuffer[], uint32_t ResponseBufferLength, PTR_COMM_EVT_t commEvt)
//
//   Author:  Dilawar Ali
//   Date:    2018/04/23
//
//!  This function Parse server response incase of Token event and update token buffer with valid received token
//
//------------------------------------------------------------------------------
static int32_t JParseInstrumentDataUpload(uint8_t js_data[], uint32_t len, uint8_t ResponseBuffer[], uint32_t ResponseBufferLength, PTR_COMM_EVT_t evt)
{
    uint8_t *start = NULL;
    uint8_t loopCounter = 0;
    start = (uint8_t *)strstr((char const*)js_data, ":");
    if(start != NULL)
    {
        start++;
        start++;
        while((*start != '\"') && (loopCounter < 25))
        {
            iNetEventId[loopCounter++] = *start++;
        }
    }
    
    return 0;
}

//------------------------------------------------------------------------------
//  int32_t CreateHttpHeader(uint8_t *buffer, uint32_t buffLen)
//
//   Author:  Dilawar Ali
//   Date:    2018/06/19
//
//!  This function Parse server response incase of Token event and update token buffer with valid received token
//
//------------------------------------------------------------------------------
int32_t CreateHttpHeader(COMM_EVT_TYPE_t evt, uint8_t *buffer, uint32_t buffLen, uint32_t contentLength)
{
    int32_t ret = -1;
    if(evt == GET_INET_TOKEN)
    {
        ret = snprintf((char *)buffer, buffLen, "POST %s HTTP/1.1\r\n"
                       "Host: %s\r\n"
                           "Connection: close\r\n"
                               "Content-Length: %d\r\n"
                                   "Content-Type: %s\r\n"
                                       "Accept: */*\r\n\r\n",
                                       httpUrlBuffer,
                                       INET_HOST,
                                       contentLength,
                                       CONTENT_TYPE_URL_ENCODED);
        
        
    }
    else
    {
        ret = snprintf((char *)buffer, buffLen, "POST %s HTTP/1.1\r\n"
                       "Host: %s\r\n"
                           "Connection: close\r\n"
                               "Content-Length: %d\r\n"
                                   "Content-Type: %s\r\n"
                                       "Authorization: %s\r\n"
                                           "Accept: */*\r\n\r\n",
                                           httpUrlBuffer,
                                           INET_HOST,
                                           contentLength,
                                           CONTENT_TYPE_JSON,
                                           tokenBuffer);
    }
    
    return ret;
}