/*
 * Copyright (C) OpenTX
 *
 * Based on code named
 *   th9x - http://code.google.com/p/th9x
 *   er9x - http://code.google.com/p/er9x
 *   gruvin9x - http://code.google.com/p/gruvin9x
 *
 * License GPLv2: http://www.gnu.org/licenses/gpl-2.0.html
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _CROSSFIRE_H_
#define _CROSSFIRE_H_

#include <inttypes.h>
#include "crc_crsf.h"
#include "../LoRa/config.h"



#ifndef PACK
#define PACK( __Declaration__ ) __Declaration__ __attribute__((__packed__))
#endif


#define  PCBI6 1
#define deviceId 0xEE
#define handsetId 0xEF

#define  TRACE(...) while(0){}
#define DIM(arr) (sizeof((arr))/sizeof((arr)[0]))
enum TelemetryProtocol
{
  TELEM_PROTO_FRSKY_D,
  TELEM_PROTO_FRSKY_SPORT,
  TELEM_PROTO_CROSSFIRE,
  TELEM_PROTO_SPEKTRUM,
  TELEM_PROTO_LUA,
  TELEM_PROTO_FLYSKY_IBUS,
};



#define TELEMETRY_RX_PACKET_SIZE       128

#define TELEMETRY_OUTPUT_FIFO_SIZE     16

extern uint8_t telemetryRxBuffer[TELEMETRY_RX_PACKET_SIZE];
extern uint8_t telemetryRxBufferCount;
extern uint8_t outputTelemetryBuffer[TELEMETRY_OUTPUT_FIFO_SIZE] ;
extern uint8_t outputTelemetryBufferSize;
extern uint8_t outputTelemetryBufferTrigger;
extern uint8_t telemetryProtocol;
extern uint32_t telemetryErrors;


int setTelemetryValue(TelemetryProtocol protocol, uint16_t id, uint8_t subId, uint8_t instance, int32_t value, uint32_t unit, uint32_t prec);

inline void telemetryOutputPushByte(uint8_t byte)
{
  outputTelemetryBuffer[outputTelemetryBufferSize++] = byte;
}

inline void telemetryOutputSetTrigger(uint8_t byte)
{
  outputTelemetryBufferTrigger = byte;
}

// Device address
#define BROADCAST_ADDRESS              0x00
#define RADIO_ADDRESS                  0xEA
#define MODULE_ADDRESS                 0xEE

// Frame id
#define GPS_ID                         0x02
#define CF_VARIO_ID                    0x07
#define BATTERY_ID                     0x08
#define LINK_ID                        0x14
#define CHANNELS_ID                    0x16
#define LINK_RX_ID                     0x1C
#define LINK_TX_ID                     0x1D
#define ATTITUDE_ID                    0x1E
#define FLIGHT_MODE_ID                 0x21
#define PING_DEVICES_ID                0x28
#define DEVICE_INFO_ID                 0x29
#define REQUEST_SETTINGS_ID            0x2A
#define COMMAND_ID                     0x32
#define RADIO_ID                       0x3A

#define UART_SYNC                      0xC8
#define SUBCOMMAND_CRSF                0x10
#define COMMAND_MODEL_SELECT_ID        0x05



//enum TelemetryUnit {
//  UNIT_RAW,
//  UNIT_VOLTS,
//  UNIT_AMPS,
//  UNIT_MILLIAMPS,
//  UNIT_KTS,
//  UNIT_METERS_PER_SECOND,
//  UNIT_FEET_PER_SECOND,
//  UNIT_KMH,
//  UNIT_MPH,
//  UNIT_METERS,
//  UNIT_FEET,
//  UNIT_CELSIUS,
//  UNIT_FAHRENHEIT,
//  UNIT_PERCENT,
//  UNIT_MAH,
//  UNIT_WATTS,
//  UNIT_MILLIWATTS,
//  UNIT_DB,
//  UNIT_RPMS,
//  UNIT_G,
//  UNIT_DEGREE,
//  UNIT_RADIANS,
//  UNIT_MILLILITERS,
//  UNIT_FLOZ,  
//  UNIT_HERTZ,
//  UNIT_DBM,
//  UNIT_HOURS,
//  UNIT_MINUTES,
//  UNIT_SECONDS,
//  // FrSky format used for these fields, could be another format in the future
//  UNIT_FIRST_VIRTUAL,
//  UNIT_CELLS = UNIT_FIRST_VIRTUAL,
//  UNIT_DATETIME,
//  UNIT_GPS,
//  UNIT_BITFIELD,
//  UNIT_TEXT,
//  // Internal units (not stored in sensor unit)
//  UNIT_GPS_LONGITUDE,
//  UNIT_GPS_LATITUDE,
//  UNIT_DATETIME_YEAR,
//  UNIT_DATETIME_DAY_MONTH,
//  UNIT_DATETIME_HOUR_MIN,
//  UNIT_DATETIME_SEC
//};

enum CrossfireSensorIndexes {
  RX_RSSI1_INDEX,
  RX_RSSI2_INDEX,
  RX_QUALITY_INDEX,
  RX_SNR_INDEX,
  RX_ANTENNA_INDEX,
  RF_MODE_INDEX,
  TX_POWER_INDEX,
  TX_RSSI_INDEX,
  TX_QUALITY_INDEX,
  TX_SNR_INDEX,
  RX_RSSI_PERC_INDEX,
  RX_RF_POWER_INDEX,
  TX_RSSI_PERC_INDEX,
  TX_RF_POWER_INDEX,
  TX_FPS_INDEX,
  BATT_VOLTAGE_INDEX,
  BATT_CURRENT_INDEX,
  BATT_CAPACITY_INDEX,
  BATT_REMAINING_INDEX,
  GPS_LATITUDE_INDEX,
  GPS_LONGITUDE_INDEX,
  GPS_GROUND_SPEED_INDEX,
  GPS_HEADING_INDEX,
  GPS_ALTITUDE_INDEX,
  GPS_SATELLITES_INDEX,
  ATTITUDE_PITCH_INDEX,
  ATTITUDE_ROLL_INDEX,
  ATTITUDE_YAW_INDEX,
  FLIGHT_MODE_INDEX,
  VERTICAL_SPEED_INDEX,
  UNKNOWN_INDEX,
};

enum CrossfireFrames{
  CRSF_FRAME_CHANNEL,
  CRSF_FRAME_MODELID,
  CRSF_FRAME_MODELID_SENT
};

//struct CrossfireSensor {
//  const uint8_t id;
//  const uint8_t subId;
//  const char * name;
//  const TelemetryUnit unit;
//  const uint8_t precision;
//};



typedef  uint_fast32_t tmr10ms_t;
typedef  uint8_t event_t;

 extern int32_t g_elrs_update_rate;
  extern int32_t        g_elrs_lag ;


#define MIN_REFRESH_RATE      1750 /* us */
#define MAX_REFRESH_RATE     50000 /* us */

#define CROSSFIRE_FRAME_MAXLEN 64


#define CROSSFIRE_CHANNELS_COUNT        16
#ifdef  HIGH_UART 
#define CROSSFIRE_BAUDRATE       400000
#define CROSSFIRE_PERIOD         4 /* us; 250 Hz */
#else
#define CROSSFIRE_BAUDRATE       115200
#define CROSSFIRE_PERIOD         8  /* us; 62 Hz */
#endif


#if 0
// Module pulse synchronization
struct ModuleSyncStatus
{
  // feedback input: last received values
  uint16_t  refreshRate; // in us
  int16_t   inputLag;    // in us

  uint32_t lastUpdate;  // in 10ms
  int16_t   currentLag;  // in us
  
   bool isValid() ;

  // Set feedback from RF module
  void update(uint16_t newRefreshRate, int16_t newInputLag);

  // Get computed settings for scheduler
  uint16_t getAdjustedRefreshRate();

  // Status string for the UI
  void getRefreshString(char* refreshText);

  ModuleSyncStatus();
};
#endif



PACK(typedef struct  t_CrossfirePulsesData{
  uint8_t pulses[CROSSFIRE_FRAME_MAXLEN];
  uint8_t length;
} ) CrossfirePulsesData;


extern int g_crossfile_baudrate_index ; 
extern CrossfirePulsesData g_crossfire;

//void registerCrossfireTelemetryCallback(void (*callback)(uint8_t, uint8_t*, uint8_t));

void runCrossfireTelemetryCallback(uint8_t command, uint8_t* data, uint8_t length);

int processCrossfireTelemetryData(uint8_t data);
//void crossfireSetDefault(int index, uint8_t id, uint8_t subId);
bool isCrossfireOutputBufferAvailable();
uint8_t createCrossfireModelIDFrame(uint8_t * frame);
bool crossfireTelemetryPush(uint8_t command, uint8_t *data, uint8_t length);
uint8_t createCrossfireChannelsFrame(uint8_t* frame, int16_t* pulses);
int setupPulsesCrossfire();
void crossfireTelemetryPush4(const uint8_t cmd, const uint8_t third, const uint8_t fourth) ;
void crossfireTelemetryPing();
void crsf_init();
void crsf_shutdown();
void crsf_action();
//uint32_t crsf_current_period();
void resetElrsMenuVariables();

#endif // _CROSSFIRE_H_
