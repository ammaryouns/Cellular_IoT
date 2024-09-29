#ifndef __SPI_COMM_H
#define __SPI_COMM_H

#include <stdint.h>
#include <spidrv.h>

#include "Main.h"

#define Board_Slave_SPI                 CC3220SF_LAUNCHXL_SPI1
#define VPRO_SPI_TRANSFER_TIME_OUT      100u     //system ticks count 
#define VPRO_SPI_BIT_RATE               1000000//125000u //set 1 mega hz
#define VPRO_SPI_DATA_TRANSFER_SIZE     8u 
#define SPI_SLAVE_BUFFER_LENGTH         95u
#define SUCCESSFULL                     1u
#define UNSUCCESSFULL                   0u


//Requests by the Master device 
#define COMMAND_SEND_DATA_TO_WIFI                               0x18
#define COMMAND_SEND_CANNED_MSG_TO_CLOUD                        0x1A
#define COMMAND_SEND_PROXIMITY_ALARM_TO_CLOUD                   0x1B
#define COMMAND_SEND_RADIO_CONFIGURE_MESSAGE                    0x41
#define COMMAND_GET_RADIO_CONFIGURE_MESSAGE                     0x51
#define COMMAND_SEND_WASSUP_MESSAGE                             0xD0
#define COMMAND_SEND_WASSUP_AGAIN_MESSAGE                       0xDA

#define COMMAND_GET_FW_HEADER                                   0x5A
#define COMMAND_GET_FW_DATA                                     0x5E
#define COMMAND_GET_FW_FOOTER                                   0x5C

//Response from Slave to master regarding firmwrae upgrade 
#define RESPOND_ACK_NACK_FIRMWARE_MESSAGE                       0x5F

//Response from Slave to master
#define RESPOND_NO_MORE_MESSAGES                                0xDD
#define RESPOND_TEXT_INET_MESSAGE                               0x9A
#define RESPOND_GET_RADIO_CONFIGURE_MESSAGE                     0x51

//Messgae Types 
#define SEND_CANNED_MESSAGE_TO_CLOUD                            0x1A
#define SEND_PROXIMITY_ALARM_TO_CLOUD                           0x1B
#define SEND_GET_RADIO_CONFIGURE_MESSAGE                        0x51
#define SEND_READ_STATUS_MESSAGE                                0x90
#define SEND_READ_QUICK_STATUS_MESSAGE                          0x91
#define SEND_MESSAGE_REQUEST                                    0xAD

//Additional required defines 
#define ONE_BYTE_LENGTH                                         0x01u
#define MAX_SENSOR_SUPPORTED                                    0x07u
#define GPS_LAT_LONG_LENGTH                                     0x04u
#define NETWORK_CONFG_LENGTH_BYTE                               0x05u
#define LENGTH_BYTE                                             5u


//#define NETWORK_PASSWORD_LENGTH                                 32u
#define CUSTOM_MSG_LENGTH                                       45u

#define I_AM_OK_MESSAGE_SIZE                                    51U          //!< Size of I am OK Message


#define NETWOK_ID_LENGTH                                        16u
#define STATUS_MESSAGE_LENGTH_BYTE                              3u
#define START_FRAMING_CHARACTER                                 0x24u 
#define END_FRAMING_CHARACTER                                   0x23u
#define MSG_VERSION_ONE                                         0x1u

#define ONE_MINUTE                                              15u //Seconds
#define PERIODIC_INET_MESSSAGE_INTERVAL                         ONE_MINUTE
#define IS_PERIODIC_EVENT_ENABLED                               true

#define EventLogDebugPrint(format,...)      SYSTEM_PRINT(EVENT_LOG_TASK_DEBUG_LOG, format, ##__VA_ARGS__)
#define EventLogDebugFlush()                SYSTEM_FLUSH(EVENT_LOG_TASK_DEBUG_LOG)

#define CHECKBIT(var,bit)           (((var) >> (bit)) & (int8_t) 0x01)   //!< Macro to read a particular bit of a number
#define SETBIT(var,bit)             ((var) |= ((int8_t) 0x01 << (bit)))  //!< Macro to set a particular bit of a number

/* Todo TT
#define HIBYTE_WORD32(param)        (int8_t)(param) >> 24               //!< Macro to get higher byte from a 32 bit word
#define MIDBYTE_A_WORD32(param)     (int8_t)((param) >> 16)              //!< Macro to get mid higher byte from a 32 bit word
#define MIDBYTE_B_WORD32(param)     (int8_t)((param) >> 8)               //!< Macro to get mid lower byte from a 32 bit word
#define LOBYTE_WORD32(param)        (int8_t)(param)                      //!< Macro to get lower byte from a 32 bit word
*/

#define HIBYTE_WORD16(param)        ( ((int8_t)(param) >> 8) & 0x00FF )               //!< Macro to get higher byte (8 MSB bits) from a 16 bit word
#define LOBYTE_WORD16(param)        ( ((int8_t)(param) )  & 0x00FF)                      //!< Macro to get lower byte (8 LSB bits) from a 16 bit word

#define HINIBBLE_WORD8(param)        ( ((int8_t)((param) >> 4)) & 0x0F )               //!< Macro to get higher nibble (4 MSB bits) from a 8 bit word
#define LONIBBLE_WORD8(param)        ( ((int8_t)(param)) & 0x0F)                      //!< Macro to get lower nibble (4 LSB bits) from a 8 bit word





typedef enum
{
    INVALID_UNITS       = 0,
    PARTS_PER_MILLION   = 1,
    PERCENTAGE_VOLUME   = 2,
    PERCANTAGE_LEL      = 3  
}GAS_MEASUREMENT_UNITS_t;

typedef enum 
{
    SENSOR_NORMAL       =0X00,
    LOW_ALARM           =0X01,
    HIGH_ALARM          =0X02,
    NEGATIVE_OR         =0X03,
    OR                  =0X04,
    CALIBRATION_FAULT   =0X05,
    ZERO_FAULT          =0X06,
    USER_DISABLED       =0X08,
    BUMP_FAULT          =0X09,
    CALIBRATION_OVERDUE =0X0B,
    DATA_FAIL           =0X0D,
    TWA_ALARM           =0X0E,
    STEL_ALARM          =0X0F
        
}SENSOR_STATUS_t;

typedef enum
{
    SERIAL_NUMBER               = 0u,
    MANUFACTURING_DATE          = 1u,
    INST_PART_NUMBER            = 2,
    TECHNICIAN_INITIALS         = 3u,
    JOB_NUMBER                  = 4u,
    HARDWARE_VERSION            = 5u,
    NETWORK_ID                  = 6u,
    NETWORK_PASSWORD            = 7u,
    NETWORK_ENCRYPTION_TYPE     = 8u,
    BATTERY_TYPE                = 9u,       //Wi-Fi set to 1; Cell set to 2
//    BATTERY_TYPE                = 10u,
    
//    BATTERY_TYPE                = 11u,
//    BATTERY_TYPE                = 12u,
//    BATTERY_TYPE                = 13u,
//    BATTERY_TYPE                = 14u,
//    BATTERY_TYPE                = 15u,
//    BATTERY_TYPE                = 16u,
//    BATTERY_TYPE                = 17u,
//    BATTERY_TYPE                = 18u,
//    BATTERY_TYPE                = 19u,
    SIM_CARRIER                 = 20u,
    
    SIM_ICCID                   = 21u,
    CELL_PHONE_NUMBER           = 22u,
    MODULE_IMEI                 = 23u,
    CELL_ANTENNA_TYPE           = 24u,
    SIM_APN                     = 25u,
    CELL_NUMBER_1               = 26u,
    CELL_NUMBER_2               = 27u,
//    BATTERY_TYPE                = 28u,
//    BATTERY_TYPE                = 29u,
//    BATTERY_TYPE                = 30u,
//    
//    BATTERY_TYPE                = 31u,
//    BATTERY_TYPE                = 32u,
    
    NO_PARAMETER                = 33u       //Defined for our own understanding can be changes     
}RADIO_CONFIGURATION_PARAMETER_t;

typedef enum 
{
    SENSOR_GAS_TYPE_INVALID                           = 0u, //!< Sensor type code - Invalid
    SENSOR_GAS_TYPE_CO                                = 1u, //!< Sensor type code - Carbon Monoxide
    SENSOR_GAS_TYPE_H2S                               = 2u, //!< Sensor type code - Hydrogen Sulphide
    SENSOR_GAS_TYPE_SO2                               = 3u, //!< Sensor type code - Sulphur Dioxide
    SENSOR_GAS_TYPE_NO2                               = 4u, //!< Sensor type code - Nitrogen Dioxide
    SENSOR_GAS_TYPE_CL2                               = 5u, //!< Sensor type code - Chlorine
    SENSOR_GAS_TYPE_CLO2                              = 6u, //!< Sensor type code - Chlorine Dioxide
    SENSOR_GAS_TYPE_HCN                               = 7u, //!< Sensor type code - Hydrogen Cyanide
    SENSOR_GAS_TYPE_PH3                               = 8u, //!< Sensor type code - Phosphine
    SENSOR_GAS_TYPE_H2                                = 9u, //!< Sensor type code - Hydrogen
    SENSOR_GAS_TYPE_CO2                              = 11u, //!< Sensor type code - Carbon Dioxide (IR)
    SENSOR_GAS_TYPE_NO                               = 12u, //!< Sensor type code - Nitric Oxide
    SENSOR_GAS_TYPE_NH3                              = 13u, //!< Sensor type code - Ammonia
    SENSOR_GAS_TYPE_HCL                              = 14u, //!< Sensor type code - Hydrogen Chloride
    SENSOR_GAS_TYPE_O3                               = 15u, //!< Sensor type code - Ozone
    SENSOR_GAS_TYPE_PHOSGENE                         = 16u, //!< Sensor type code - Phosgene
    SENSOR_GAS_TYPE_HF                               = 17u, //!< Sensor type code - HF sensor 
    SENSOR_GAS_TYPE_CH4_IR                           = 18u, //!< Sensor type code - Methane IR
    SENSOR_GAS_TYPE_CO_H2_NULL                       = 19u, //!< Sensor type code - CO H2 Null sensor 
    SENSOR_GAS_TYPE_O2                               = 20u, //!< Sensor type code - Oxygen
    SENSOR_GAS_TYPE_CH4                              = 21u, //!< Sensor type code - Methane (LEL & IR)
    SENSOR_GAS_TYPE_HEXANE                           = 23u, //!< Sensor type code - Hexane (LEL)
    SENSOR_GAS_TYPE_PENTANE                          = 26u, //!< Sensor type code - Pentane (LEL)
    SENSOR_GAS_TYPE_PROPANE                          = 27u, //!< Sensor type code - Propane (LEL & IR)
    SENSOR_GAS_TYPE_14BUTANEDIOL                     = 28u, //!< Sensor type code - Butanediol,1,4-
    SENSOR_GAS_TYPE_14DIOXANE                        = 29u, //!< Sensor type code - Dioxane,1,4-
    SENSOR_GAS_TYPE_124TRIMETH                       = 30u, //!< Sensor type code - Trimethylbenzene,1,2,4-
    SENSOR_GAS_TYPE_123TRIMETH                       = 31u, //!< Sensor type code - Trimethylbenzene,1,2,3-
    SENSOR_GAS_TYPE_12DIBROMO                        = 32u, //!< Sensor type code - Dibromoethane,1,2-
    SENSOR_GAS_TYPE_12DICHLORO                       = 33u, //!< Sensor type code - Dichlorobenzene,1,2-
    SENSOR_GAS_TYPE_135TRIMETH                       = 34u, //!< Sensor type code - Trimethylbenzene,1,3,5-
    SENSOR_GAS_TYPE_1BUTANOL                         = 35u, //!< Sensor type code - Butanol,1-
    SENSOR_GAS_TYPE_1METH2PROP                       = 36u, //!< Sensor type code - Methoxypropanol
    SENSOR_GAS_TYPE_1PROPANOL                        = 37u, //!< Sensor type code - Propanol,1-
    SENSOR_GAS_TYPE_METHACETATE                      = 38u, //!< Sensor type code - Methyl Acetate
    SENSOR_GAS_TYPE_METHACRYL                        = 39u, //!< Sensor type code - Methyl Acrylate
    SENSOR_GAS_TYPE_METHACETO                        = 40u, //!< Sensor type code - Methyl Acetoacetate
    SENSOR_GAS_TYPE_METHBENZO                        = 41u, //!< Sensor type code - Methyl Benzoate
    SENSOR_GAS_TYPE_METHMETHACR                      = 42u, //!< Sensor type code - Methyl Methacrylate
    SENSOR_GAS_TYPE_2BUTANONE                        = 43u, //!< Sensor type code - Butanone
    SENSOR_GAS_TYPE_DIMTHFORM                        = 44u, //!< Sensor type code - Dimethylformamide
    SENSOR_GAS_TYPE_2METHOXYETH                      = 45u, //!< Sensor type code - Methoxy Ethanol,2-
    SENSOR_GAS_TYPE_2PENTANONE                       = 46u, //!< Sensor type code - Pentanone,2-
    SENSOR_GAS_TYPE_2PICOLINE                        = 47u, //!< Sensor type code - Picoline,2-
    SENSOR_GAS_TYPE_2PROPANOL                        = 48u, //!< Sensor type code - Propanol,2-
    SENSOR_GAS_TYPE_NNDIMETHFORM                     = 49u, //!< Sensor type code - Dimethylformamide
    SENSOR_GAS_TYPE_NNDIMETHACET                     = 50u, //!< Sensor type code - Dimethyl Acetamide
    SENSOR_GAS_TYPE_3PICOLINE                        = 51u, //!< Sensor type code - Picoline,3-
    SENSOR_GAS_TYPE_DIACETONE                        = 52u, //!< Sensor type code - Diacetone Alcohol
    SENSOR_GAS_TYPE_ACETALDEHYDE                     = 53u, //!< Sensor type code - Acetaldehyde
    SENSOR_GAS_TYPE_ACETONE                          = 54u, //!< Sensor type code - Acetone
    SENSOR_GAS_TYPE_ACETOPHENONE                     = 55u, //!< Sensor type code - Acetophenone
    SENSOR_GAS_TYPE_ALLYL_ALC                        = 56u, //!< Sensor type code - Allyl Alcohol
    SENSOR_GAS_TYPE_AMYLACETATE                      = 58u, //!< Sensor type code - Amyl Acetate
    SENSOR_GAS_TYPE_BENZENE                          = 59u, //!< Sensor type code - Benzene
    SENSOR_GAS_TYPE_METHBROMIDE                      = 60u, //!< Sensor type code - Methyl Bromide
    SENSOR_GAS_TYPE_BUTADIENE                        = 61u, //!< Sensor type code - Butadiene
    SENSOR_GAS_TYPE_BUTOXYETH                        = 62u, //!< Sensor type code - Butoxy Ethanol
    SENSOR_GAS_TYPE_BUTYLACETATE                     = 63u, //!< Sensor type code - Butyl Acetate
    SENSOR_GAS_TYPE_TETRACHLORO                      = 64u, //!< Sensor type code - Tetrachloroethylene
    SENSOR_GAS_TYPE_11DICHLORO                       = 65u, //!< Sensor type code - Dichloroethene,t-1,2
    SENSOR_GAS_TYPE_ETHYLBENZENE                     = 66u, //!< Sensor type code - Ethyl Benzene
    SENSOR_GAS_TYPE_TRICHLOROETH                     = 67u, //!< Sensor type code - Trichloroethylene
    SENSOR_GAS_TYPE_ETHYLACETO                       = 68u, //!< Sensor type code - Ethyl Acetoacetate
    SENSOR_GAS_TYPE_CHLOROBENZ                       = 69u, //!< Sensor type code - Chlorobenzene
    SENSOR_GAS_TYPE_CUMENE                           = 70u, //!< Sensor type code - Cumene
    SENSOR_GAS_TYPE_CYCLOHEXANE                      = 71u, //!< Sensor type code - Cyclohexane
    SENSOR_GAS_TYPE_CYCHEXANONE                      = 72u, //!< Sensor type code - Cyclohexanone
    SENSOR_GAS_TYPE_DECANE                           = 73u, //!< Sensor type code - Decane
    SENSOR_GAS_TYPE_DIETHYLAMINE                     = 74u, //!< Sensor type code - Diethylamine
    SENSOR_GAS_TYPE_DIMETHOXY                        = 75u, //!< Sensor type code - Dimethoxymethane
    SENSOR_GAS_TYPE_EPICHLORO                        = 76u, //!< Sensor type code - Epichlorohydrin
    SENSOR_GAS_TYPE_ETHANOL                          = 77u, //!< Sensor type code - Etahnol
    SENSOR_GAS_TYPE_ETHGLYCOL                        = 78u, //!< Sensor type code - Ethylene Glycol
    SENSOR_GAS_TYPE_ETHACETATE                       = 79u, //!< Sensor type code - Ethyl Acetate
    SENSOR_GAS_TYPE_ETHYLENE                         = 80u, //!< Sensor type code - Ethylene
    SENSOR_GAS_TYPE_ETO                              = 81u, //!< Sensor type code - Ethylene Oxide
    SENSOR_GAS_TYPE_BUTYROLACT                       = 82u, //!< Sensor type code - Butyrolactone
    SENSOR_GAS_TYPE_HEPTANE                          = 84u, //!< Sensor type code - Heptane
    SENSOR_GAS_TYPE_HYDRAZINE                        = 86u, //!< Sensor type code - Hydrazine
    SENSOR_GAS_TYPE_ISOAMYLACET                      = 87u, //!< Sensor type code - Isoamyl Acetate
    SENSOR_GAS_TYPE_ISOPROPAMINE                     = 88u, //!< Sensor type code - Isopropylamine
    SENSOR_GAS_TYPE_ISOPROPETHER                     = 89u, //!< Sensor type code - Isopropyl Ether
    SENSOR_GAS_TYPE_ISOBUTANOL                       = 90u, //!< Sensor type code - Isobutanol
    SENSOR_GAS_TYPE_ISOBUTYLENE                      = 91u, //!< Sensor type code - Isobutylene
    SENSOR_GAS_TYPE_ISOOCTANE                        = 92u, //!< Sensor type code - Isooctane
    SENSOR_GAS_TYPE_ISOPHORONE                       = 93u, //!< Sensor type code - Isophorone
    SENSOR_GAS_TYPE_ISOPROPANOL                      = 94u, //!< Sensor type code - Isopropanol
    SENSOR_GAS_TYPE_JETAFUEL                         = 95u, //!< Sensor type code - Jet A Fuel
    SENSOR_GAS_TYPE_JETA1FUEL                        = 96u, //!< Sensor type code - Jet A1 Fuel
    SENSOR_GAS_TYPE_MEK                              = 98u, //!< Sensor type code - MEK (Methylethyl Ketone)
    SENSOR_GAS_TYPE_MESITYLOXIDE                     = 99u, //!< Sensor type code - Mesityl Oxide
    SENSOR_GAS_TYPE_MIBK                             = 100u,//!< Sensor type code - MIBK (Methyl-Isobutyl Ketone)
    SENSOR_GAS_TYPE_MONOMETHYL                       = 101u,//!< Sensor type code - Monomethylamine
    SENSOR_GAS_TYPE_MTBE                             = 102u,//!< Sensor type code - MTBE (Methyl-tertbutyl Ether)
    SENSOR_GAS_TYPE_METHBENZYL                       = 103u,//!< Sensor type code - Methylbenzyl Alcohol
    SENSOR_GAS_TYPE_MXYLENE                          = 104u,//!< Sensor type code - Xylene,m-
    SENSOR_GAS_TYPE_NMETHPYRROL                      = 105u,//!< Sensor type code - Methylpyrrolidone,n-
    SENSOR_GAS_TYPE_OCTANE                           = 106u,//!< Sensor type code - Octane
    SENSOR_GAS_TYPE_OXYLENE                          = 107u,//!< Sensor type code - Xylene,o-
    SENSOR_GAS_TYPE_PHENELETHYL                      = 108u,//!< Sensor type code - Phenylethyl Alcohol
    SENSOR_GAS_TYPE_PHENOL                           = 109u,//!< Sensor type code - Phenol
    SENSOR_GAS_TYPE_PROPYLENE                        = 111u,//!< Sensor type code - Propylene
    SENSOR_GAS_TYPE_PROPYLOXIDE                      = 112u,//!< Sensor type code - Propylene Oxide
    SENSOR_GAS_TYPE_PXYLENE                          = 113u,//!< Sensor type code - Xylene,p-
    SENSOR_GAS_TYPE_PYRIDINE                         = 114u,//!< Sensor type code - Pyridine
    SENSOR_GAS_TYPE_QUINOLINE                        = 115u,//!< Sensor type code - Quinoline
    SENSOR_GAS_TYPE_STYRENE                          = 116u,//!< Sensor type code - Styrene
    SENSOR_GAS_TYPE_TBUTYLAMINE                      = 117u,//!< Sensor type code - tert-Butylamine
    SENSOR_GAS_TYPE_TRISDICHLETH                     = 118u,//!< Sensor type code - Dichloroethene,t-1,2
    SENSOR_GAS_TYPE_TBUTYLMERCAP                     = 119u,//!< Sensor type code - tert-Butyl Mercaptan
    SENSOR_GAS_TYPE_TBUTYLALCO                       = 120u,//!< Sensor type code - tert-Butyl Alcohol
    SENSOR_GAS_TYPE_THF                              = 121u,//!< Sensor type code - THF (Tetrahydrofuran)
    SENSOR_GAS_TYPE_THIOPHENE                        = 122u,//!< Sensor type code - Thiophene
    SENSOR_GAS_TYPE_TOLUENE                          = 123u,//!< Sensor type code - Toluene
    SENSOR_GAS_TYPE_TURPENTINE                       = 124u,//!< Sensor type code - Turpentine
    SENSOR_GAS_TYPE_VINYLCYCLO                       = 125u,//!< Sensor type code - Vinylcyclohexone
    SENSOR_GAS_TYPE_VINYLACETATE                     = 126u,//!< Sensor type code - Vinyl Acetate
    SENSOR_GAS_TYPE_VINYLCHLOR                       = 127u,//!< Sensor type code - Vinyl Chloride
    SENSOR_GAS_TYPE_ACETYLENE                        = 200u,//!< Sensor type code - Acetylene
    SENSOR_GAS_TYPE_BUTANE                           = 201u,//!< Sensor type code - Butane
    SENSOR_GAS_TYPE_ETHANE                           = 202u,//!< Sensor type code - Ethane
    SENSOR_GAS_TYPE_METHANOL                         = 203u,//!< Sensor type code - Methanol
    SENSOR_GAS_TYPE_JP4                              = 205u,//!< Sensor type code - JP-4
    SENSOR_GAS_TYPE_JP5                              = 206u,//!< Sensor type code - JP-5
    SENSOR_GAS_TYPE_JP8                              = 207u,//!< Sensor type code - JP-8
    SENSOR_GAS_TYPE_ACETIC_ACID                      = 208u,//!< Sensor type code - Acetic Acid
    SENSOR_GAS_TYPE_ACETIC_ANHYD                     = 209u,//!< Sensor type code - Acetic Anhydride
    SENSOR_GAS_TYPE_ARSINE                           = 210u,//!< Sensor type code - Arsine
    SENSOR_GAS_TYPE_BROMINE                          = 211u,//!< Sensor type code - Bromine
    SENSOR_GAS_TYPE_CARBON_DISUL                     = 212u,//!< Sensor type code - Carbon Disulfide
    SENSOR_GAS_TYPE_CYCLOHEXENE                      = 213u,//!< Sensor type code - Cyclohexene
    SENSOR_GAS_TYPE_DIESEL_FUEL                      = 214u,//!< Sensor type code - Diesel Fuel
    SENSOR_GAS_TYPE_DIMETHSULFOX                     = 215u,//!< Sensor type code - Dimethyl Sulfoxide
    SENSOR_GAS_TYPE_ETHYL_ETHER                      = 216u,//!< Sensor type code - Ethyl Ether
    SENSOR_GAS_TYPE_IODINE                           = 217u,//!< Sensor type code - Iodine
    SENSOR_GAS_TYPE_METHYL_MERC                      = 218u,//!< Sensor type code - Methyl Mercaptan
    SENSOR_GAS_TYPE_NAPHTHALENE                      = 219u,//!< Sensor type code - Naphthalene
    SENSOR_GAS_TYPE_NITROBENZENE                     = 220u,//!< Sensor type code - Nitrobenzene
    SENSOR_GAS_TYPE_2MTHOXETHOXE                     = 221u,//!< Sensor type code - Methoxyethoxyethanol,2-
    SENSOR_GAS_TYPE_NONANE                           = 222u,//!< Sensor type code - Nonane
    SENSOR_GAS_TYPE_HC		                     = 248u,//!< Sensor type code - Hydrocarben sensor
    // Custom response factors are not supported in VPRO
    
}SENSOR_TYPES_t;

typedef enum
{
    RUNNING_NORMAL                      = 0u,
    NO_CONNECTION_AVALIBLE              = 1u,
    CREDENTIALS_REJECTED_BY_NETWORK     = 2u,
    NO_CELL_SERVICE                     = 3u,
    ERROR_UNDEFINED                     = 4u
}SMART_BATTERY_STATUS_t;

typedef enum 
{
    INET_ACCEPTS_INSTRUMENT_AND_MONITORING      = 0u,
    INET_NOT_MONITORING_INSTRUMENT              = 1u,
    NO_CELL_OR_WIFI                             = 2u,
    CAN_NOT_CONNECT_TO_INET                     = 3u,
    INVALID_INET_CREDANTIALS                    = 4u
}INET_STATUS_t;

typedef enum
{
    NO_MESSAGE                    =0u,
    MESSAGE_FROM_INET             =1u,
    EMERGENCY_MESSAGE_FROM_INET   =2u,
    ALARM_MESSAGE_FROM_INET       =3u
}CLOUD_MESSAGE_BYTE_t;

typedef enum
{
    NO_MESSAGE_LEFT             = 0u,
    RESPOND_STATUS_MSG          = 1u,
    RESPOND_QUICK_STATUS_MSG    = 2u,
    RESPOND_RADIO_CONFIGURATION = 3u,
    PASS_INET_MESSAGE           = 4u,
    RENSPOND_FW_ACK_NACK_MESSAGE,
    
    
}MASTER_REQUESTS_t;

typedef struct 
{
    SENSOR_TYPES_t  SensorType;
    GAS_MEASUREMENT_UNITS_t   SensorMeasuringUnits;
    char   SensorReadingHigh;
    char   SensorReadingLow;   
    SENSOR_STATUS_t SensorStatus;
    unsigned char DecimalPlaces;
    uint8_t componentCode;
    unsigned char   InternalSensorStatus;
}SensorInfo_t;

typedef struct 
{
    uint8_t numberOfSensors;
    SensorInfo_t sensorArray[MAX_SENSOR_SUPPORTED];
}InstSensorInfo_t;

typedef struct
{
    unsigned char SerialNumber[SERIAL_NUMBER_LENGTH+1];
    unsigned char UserName[SERIAL_NUMBER_LENGTH+1];
    unsigned char SiteName[SERIAL_NUMBER_LENGTH+1];
    unsigned char InstrumentType;
    unsigned char MessageCounter;
    unsigned char InstrumentState;
    unsigned char InternalInstrumentState;
    unsigned char UserSecurityLevel;
    unsigned char SiteSecurityLevel;
    InstSensorInfo_t  SensorsInfo;
    unsigned char UserCustomMessage[CUSTOM_MSG_LENGTH];
}Remote_Instrument_t;


extern SPIDRV_Handle_t SPIHandle;
extern Remote_Instrument_t RemoteUnit;
void VproSlaveStateMachine (uint16_t EventType);
int32_t SPISlaveConfigure(void);

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
uint8_t ManageOutgoingMessage(void);

//==============================================================================
//
//  void ProcessIncomingmessage(unsigned char *IncomingBuffer, unsigned char StartingIndex)
//
//   Author:   Tayyab Tahir
//   Date:     2018/04/23
//
//!  This function handeles data extraction from incoming message according to 
//!  their type.
//
//==============================================================================
void ProcessIncomingmessage(unsigned char *IncomingBuffer, unsigned char StartingIndex);

#endif/*__SPI_COMM_H*/