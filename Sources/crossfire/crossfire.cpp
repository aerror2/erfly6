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

#include "crossfire.h"
#include "../er9x.h"
#include <string.h>
#include "../en.h"
#include "../iface_a7105.h"


uint8_t outputTelemetryBuffer[TELEMETRY_OUTPUT_FIFO_SIZE] ;
uint8_t outputTelemetryBufferSize = 0;
uint8_t outputTelemetryBufferTrigger = 0;

uint8_t telemetryStreaming = 0;
uint8_t telemetryRxBuffer[TELEMETRY_RX_PACKET_SIZE];   // Receive buffer. 9 bytes (full packet), worst case 18 bytes with byte-stuffing (+1)
uint8_t telemetryRxBufferCount = 0;

CrossfirePulsesData g_crossfire={{0},0};
uint32_t telemetryErrors=0;
//bool g_crsf_inited =0;
//int g_crossfile_baudrate_index =4; 



int32_t g_elrs_update_rate;
int32_t g_elrs_lag;




////#if SPORT_MAX_BAUDRATE < 400000
//const uint32_t CROSSFIRE_BAUDRATES[] = {
//  3750000,
//  1870000,
//  921600,
//  400000,
//  115200,
//};

//const uint8_t CROSSFIRE_PERIODS[] = {
//  4,
//  4,
//  4,
//  4,
//  16,
//};

//const CrossfireSensor crossfireSensors[] = {
//    {LINK_ID, 0, ZSTR_RX_RSSI1, UNIT_DB, 0},
//    {LINK_ID, 1, ZSTR_RX_RSSI2, UNIT_DB, 0},
//    {LINK_ID, 2, ZSTR_RX_QUALITY, UNIT_PERCENT, 0},
//    {LINK_ID, 3, ZSTR_RX_SNR, UNIT_DB, 0},
//    {LINK_ID, 4, ZSTR_ANTENNA, UNIT_RAW, 0},
//    {LINK_ID, 5, ZSTR_RF_MODE, UNIT_RAW, 0},
//    {LINK_ID, 6, ZSTR_TX_POWER, UNIT_MILLIWATTS, 0},
//    {LINK_ID, 7, ZSTR_TX_RSSI, UNIT_DB, 0},
//    {LINK_ID, 8, ZSTR_TX_QUALITY, UNIT_PERCENT, 0},
//    {LINK_ID, 9, ZSTR_TX_SNR, UNIT_DB, 0},
//    {LINK_RX_ID, 0, ZSTR_RX_RSSI_PERC, UNIT_PERCENT, 0},
//    {LINK_RX_ID, 1, ZSTR_RX_RF_POWER, UNIT_DBM, 0},
//    {LINK_TX_ID, 0, ZSTR_TX_RSSI_PERC, UNIT_PERCENT, 0},
//    {LINK_TX_ID, 1, ZSTR_TX_RF_POWER, UNIT_DBM, 0},
//    {LINK_TX_ID, 2, ZSTR_TX_FPS, UNIT_HERTZ, 0},
//    {BATTERY_ID, 0, ZSTR_BATT, UNIT_VOLTS, 1},
//    {BATTERY_ID, 1, ZSTR_CURR, UNIT_AMPS, 1},
//    {BATTERY_ID, 2, ZSTR_CAPACITY, UNIT_MAH, 0},
//    {BATTERY_ID, 3, ZSTR_BATT_PERCENT, UNIT_PERCENT, 0},
//    {GPS_ID, 0, ZSTR_GPS, UNIT_GPS_LATITUDE, 0},
//    {GPS_ID, 0, ZSTR_GPS, UNIT_GPS_LONGITUDE, 0},
//    {GPS_ID, 2, ZSTR_GSPD, UNIT_KMH, 1},
//    {GPS_ID, 3, ZSTR_HDG, UNIT_DEGREE, 3},
//    {GPS_ID, 4, ZSTR_ALT, UNIT_METERS, 0},
//    {GPS_ID, 5, ZSTR_SATELLITES, UNIT_RAW, 0},
//    {ATTITUDE_ID, 0, ZSTR_PITCH, UNIT_RADIANS, 3},
//    {ATTITUDE_ID, 1, ZSTR_ROLL, UNIT_RADIANS, 3},
//    {ATTITUDE_ID, 2, ZSTR_YAW, UNIT_RADIANS, 3},
//    {FLIGHT_MODE_ID, 0, ZSTR_FLIGHT_MODE, UNIT_TEXT, 0},
//    {CF_VARIO_ID, 0, ZSTR_VSPD, UNIT_METERS_PER_SECOND, 2},
//    {0, 0, "UNKNOWN", UNIT_RAW, 0},
//};

//const CrossfireSensor &getCrossfireSensor(uint8_t id, uint8_t subId) {
//  if (id == LINK_ID)
//    return crossfireSensors[RX_RSSI1_INDEX + subId];
//  else if (id == LINK_RX_ID)
//    return crossfireSensors[RX_RSSI_PERC_INDEX + subId];
//  else if (id == LINK_TX_ID)
//    return crossfireSensors[TX_RSSI_PERC_INDEX + subId];
//  else if (id == BATTERY_ID)
//    return crossfireSensors[BATT_VOLTAGE_INDEX + subId];
//  else if (id == GPS_ID)
//    return crossfireSensors[GPS_LATITUDE_INDEX + subId];
//  else if (id == CF_VARIO_ID)
//    return crossfireSensors[VERTICAL_SPEED_INDEX];
//  else if (id == ATTITUDE_ID)
//    return crossfireSensors[ATTITUDE_PITCH_INDEX + subId];
//  else if (id == FLIGHT_MODE_ID)
//    return crossfireSensors[FLIGHT_MODE_INDEX];
//  else
//    return crossfireSensors[UNKNOWN_INDEX];
//}



#define CROSSFIRE_CH_CENTER 0x3E0
#define CROSSFIRE_CH_BITS 11


#define CROSSFIRE_CENTER_CH_OFFSET(ch) (0)


//uint8_t createCrossfireModelIDFrame(uint8_t* frame) {
//  uint8_t* buf = frame;
//  *buf++ = UART_SYNC;                               /* device address */
//  *buf++ = 8;                                       /* frame length */
//  *buf++ = COMMAND_ID;                              /* cmd type */
//  *buf++ = MODULE_ADDRESS;                          /* Destination Address */
//  *buf++ = RADIO_ADDRESS;                           /* Origin Address */
//  *buf++ = SUBCOMMAND_CRSF;                         /* sub command */
//  *buf++ = COMMAND_MODEL_SELECT_ID;                 /* command of set model/receiver id */
//  *buf++ = (~GetChipID()) &0xFF; /* model ID */

//  *buf++ = crc8_BA(frame + 2, 6);
//  *buf++ = crc8(frame + 2, 7);
//  return buf - frame;
//}

// Range for pulses (channels output) is [-1024:+1024]
uint8_t createCrossfireChannelsFrame(uint8_t* frame, int16_t* pulses) {
  uint8_t* buf = frame;
  *buf++ = MODULE_ADDRESS;
  *buf++ = 24;  // 1(ID) + 22 + 1(CRC)
  uint8_t* crc_start = buf;
  *buf++ = CHANNELS_ID;
  uint32_t bits = 0;
  uint8_t bitsavailable = 0;
  for (int i = 0; i < CROSSFIRE_CHANNELS_COUNT; i++) {
    uint32_t val = limit(0, CROSSFIRE_CH_CENTER + (((pulses[i]) * 4) / 5), 2 * CROSSFIRE_CH_CENTER);
    bits |= val << bitsavailable;
    bitsavailable += CROSSFIRE_CH_BITS;
    while (bitsavailable >= 8) {
      *buf++ = bits;
      bits >>= 8;
      bitsavailable -= 8;
    }
  }

  *buf++ = crc8(crc_start, 23);
  return buf - frame;
}

int setupPulsesCrossfire() {
  uint8_t* pulses = g_crossfire.pulses;
  
    if (outputTelemetryBufferSize > 0) {
      memcpy(pulses, outputTelemetryBuffer, outputTelemetryBufferSize);
      g_crossfire.length = outputTelemetryBufferSize;
      outputTelemetryBufferSize = 0;
      outputTelemetryBufferTrigger = 0;
      return 1;
    } else {
      g_crossfire.length = createCrossfireChannelsFrame(
          pulses,
          g_chans512);
          return 0;
    }
}



#if 0
static ModuleSyncStatus moduleSyncStatus;

ModuleSyncStatus &getModuleSyncStatus(uint8_t moduleIdx)
{
  return moduleSyncStatus;
}

bool ModuleSyncStatus::isValid()
{
    // 2 seconds
    return (g_tmr10ms- lastUpdate < 200);
 }
  
ModuleSyncStatus::ModuleSyncStatus()
{
  memset(this, 0, sizeof(ModuleSyncStatus));
}

void ModuleSyncStatus::update(uint16_t newRefreshRate, int16_t newInputLag)
{
  if (!newRefreshRate)
    return;
  
  if (newRefreshRate < MIN_REFRESH_RATE)
    newRefreshRate = newRefreshRate * (MIN_REFRESH_RATE / (newRefreshRate + 1));
  else if (newRefreshRate > MAX_REFRESH_RATE)
    newRefreshRate = MAX_REFRESH_RATE;

  refreshRate = newRefreshRate;
  inputLag    = newInputLag;
  currentLag  = newInputLag;
  lastUpdate  = g_tmr10ms;

  TRACE("[SYNC] update rate = %dus; lag = %dus",refreshRate,currentLag);
}

uint16_t ModuleSyncStatus::getAdjustedRefreshRate()
{
  int16_t lag = currentLag;
  int32_t newRefreshRate = refreshRate;

  if (lag == 0) {
    return refreshRate;
  }
  
  newRefreshRate += lag;
  
  if (newRefreshRate < MIN_REFRESH_RATE) {
      newRefreshRate = MIN_REFRESH_RATE;
  }
  else if (newRefreshRate > MAX_REFRESH_RATE) {
    newRefreshRate = MAX_REFRESH_RATE;
  }

  currentLag -= newRefreshRate - refreshRate;
  TRACE("[SYNC] mod rate = %dus; lag = %dus",newRefreshRate,currentLag);
  
  return (uint16_t)newRefreshRate;
}

#endif 


//int setTelemetryValue(TelemetryProtocol protocol, uint16_t id, uint8_t subId, uint8_t instance, int32_t value, uint32_t unit, uint32_t prec)
//{
//    //TODO
//    return 0;
//}

//int setTelemetryText(TelemetryProtocol protocol, uint16_t id, uint8_t subId, uint8_t instance, const char* text) {
//  //TODO
//  return 0;
//}

 static  uint8_t idx_to_afhds2a [] ={
FST_IDX_ERR, //RX_RSSI1_INDEX,
FST_IDX_NOISE,// RX_RSSI2_INDEX,
FST_IDX_RSSI, // RX_QUALITY_INDEX,
FST_IDX_SNR, // RX_SNR_INDEX,
 0xff,// RX_ANTENNA_INDEX,
FST_IDX_S86, // RF_MODE_INDEX,
FST_IDX_S87, // TX_POWER_INDEX,
FST_IDX_TSSI, // TX_RSSI_INDEX,
FST_IDX_S88, // TX_QUALITY_INDEX,
FST_IDX_S89,// TX_SNR_INDEX,
 0xff,// RX_RSSI_PERC_INDEX,
FST_IDX_TX_V, // RX_RF_POWER_INDEX,
 0xff,// TX_RSSI_PERC_INDEX,
FST_IDX_S8a, // TX_RF_POWER_INDEX,
 FST_IDX_S85,// TX_FPS_INDEX,
 FST_IDX_INTV,// BATT_VOLTAGE_INDEX,
 FST_IDX_BAT_CURR,// BATT_CURRENT_INDEX,
 FST_IDX_THRCAP,// BATT_CAPACITY_INDEX,
 FST_IDX_FUEL,// BATT_REMAINING_INDEX,
 FST_IDX_GPS_LAT,// GPS_LATITUDE_INDEX,
 FST_IDX_GPS_LON,// GPS_LONGITUDE_INDEX,
 FST_IDX_GROUND_SPEED,// GPS_GROUND_SPEED_INDEX,
 FST_IDX_CMP_HEAD,// GPS_HEADING_INDEX,
 FST_IDX_GPS_ALT,// GPS_ALTITUDE_INDEX,
 FST_IDX_GPS_STATUS,// GPS_SATELLITES_INDEX,
 FST_IDX_PITCH,// ATTITUDE_PITCH_INDEX,
 FST_IDX_ROLL, // ATTITUDE_ROLL_INDEX,
 FST_IDX_YAW,// ATTITUDE_YAW_INDEX,
 FST_IDX_FLIGHT_MODE,// FLIGHT_MODE_INDEX,
 FST_IDX_VERTICAL_SPEED,// VERTICAL_SPEED_INDEX,
 // UNKNOWN_INDEX,
 };

void processCrossfireTelemetryValue(uint8_t index, int32_t value) {
  int tidx = 0;
   if(index < UNKNOWN_INDEX && (tidx =idx_to_afhds2a[index]) !=0xff)
   {
        AFHDS2A_tel_data[tidx] = value;
       AFHDS2A_tel_status |= 1<<tidx;
   }
  //const CrossfireSensor &sensor = crossfireSensors[index];
  //setTelemetryValue(TELEM_PROTO_CROSSFIRE, sensor.id, 0, sensor.subId, value, sensor.unit, sensor.precision);

}

bool checkCrossfireTelemetryFrameCRC() {
  uint8_t len = telemetryRxBuffer[1];
  uint8_t crc = crc8(&telemetryRxBuffer[2], len - 1);

  return (crc == telemetryRxBuffer[len + 1]);
}



bool getCrossfireTelemetryValue(uint8_t index, int32_t &value, int N) {
  bool result = false;
  uint8_t *byte = &telemetryRxBuffer[index];
  value = (*byte & 0x80) ? -1 : 0;
  for (uint8_t i = 0; i < N; i++) {
    value <<= 8;
    if (*byte != 0xff) {
      result = true;
    }
    value += *byte++;
  }
  return result;
}


void processCrossfireTelemetryFrame() {


  if (!checkCrossfireTelemetryFrameCRC()) {
    TRACE("[XF] CRC error");
    telemetryErrors++;
    return;
  }

          uint8_t id = telemetryRxBuffer[2];
  int32_t value;
  switch (id) {
#if 1
    case CF_VARIO_ID:
      if (getCrossfireTelemetryValue(3, value,2))
          processCrossfireTelemetryValue(VERTICAL_SPEED_INDEX, value);
     break;

    case GPS_ID:
      if (getCrossfireTelemetryValue(3, value,4))
        processCrossfireTelemetryValue(GPS_LATITUDE_INDEX, value / 10);
      if (getCrossfireTelemetryValue(7, value,4))
        processCrossfireTelemetryValue(GPS_LONGITUDE_INDEX, value / 10);
      if (getCrossfireTelemetryValue(11, value,2))
        processCrossfireTelemetryValue(GPS_GROUND_SPEED_INDEX, value);
      if (getCrossfireTelemetryValue(13, value,2))
        processCrossfireTelemetryValue(GPS_HEADING_INDEX, value);
      if (getCrossfireTelemetryValue(15, value,2))
        processCrossfireTelemetryValue(GPS_ALTITUDE_INDEX, value - 1000);
      if (getCrossfireTelemetryValue(17, value,1))
        processCrossfireTelemetryValue(GPS_SATELLITES_INDEX, value);
      break;

    case LINK_ID:
      for (unsigned int i = 0; i <= TX_SNR_INDEX; i++) {
        if (getCrossfireTelemetryValue(3 + i, value,1)) {
          if (i == TX_POWER_INDEX) {
            static const int32_t power_values[] = {0, 10, 25, 100, 500, 1000, 2000, 250, 50};
            value = ((unsigned)value < DIM(power_values) ? power_values[value] : 0);
          }
          processCrossfireTelemetryValue(i, value);
          //if (i == RX_QUALITY_INDEX) {

          //  if (value) {
          //   //  telemetryData.rssi.set(value);
          //   //  telemetryStreaming = TELEMETRY_TIMEOUT10ms;
          //  } else {
          //   // telemetryData.rssi.reset();
          //  //  telemetryStreaming = 0;
          //  }
     
          //}
        }
      }
      break;

    case LINK_RX_ID:
      if (getCrossfireTelemetryValue(4, value,1))
        processCrossfireTelemetryValue(RX_RSSI_PERC_INDEX, value);
      if (getCrossfireTelemetryValue(7, value,1))
        processCrossfireTelemetryValue(TX_RF_POWER_INDEX, value);
      break;

    case LINK_TX_ID:
      if (getCrossfireTelemetryValue(4, value,1))
        processCrossfireTelemetryValue(TX_RSSI_PERC_INDEX, value);
      if (getCrossfireTelemetryValue(7, value,1))
        processCrossfireTelemetryValue(RX_RF_POWER_INDEX, value);
      if (getCrossfireTelemetryValue(8, value,1))
        processCrossfireTelemetryValue(TX_FPS_INDEX, value * 10);
      break;

    case BATTERY_ID:
      if (getCrossfireTelemetryValue(3, value,2))
        processCrossfireTelemetryValue(BATT_VOLTAGE_INDEX, value*10);
      if (getCrossfireTelemetryValue(5, value,2))
        processCrossfireTelemetryValue(BATT_CURRENT_INDEX, value*10);
      if (getCrossfireTelemetryValue(7, value,3))
        processCrossfireTelemetryValue(BATT_CAPACITY_INDEX, value*10);
      if (getCrossfireTelemetryValue(10, value,1))
        processCrossfireTelemetryValue(BATT_REMAINING_INDEX, value*10);
      break;

    case ATTITUDE_ID:
      if (getCrossfireTelemetryValue(3, value,2))
        processCrossfireTelemetryValue(ATTITUDE_PITCH_INDEX, value / 10);
      if (getCrossfireTelemetryValue(5, value,2))
        processCrossfireTelemetryValue(ATTITUDE_ROLL_INDEX, value / 10);
      if (getCrossfireTelemetryValue(7, value,2))
        processCrossfireTelemetryValue(ATTITUDE_YAW_INDEX, value / 10);
      break;

    case FLIGHT_MODE_ID: {
    #if 0   //TODO
      const CrossfireSensor &sensor = crossfireSensors[FLIGHT_MODE_INDEX];
      int textLength = min<int>(16, telemetryRxBuffer[1]);
      telemetryRxBuffer[textLength] = '\0';
      setTelemetryText(TELEM_PROTO_CROSSFIRE, sensor.id, 0, sensor.subId, (const char *)telemetryRxBuffer + 3);
   #endif
     break;
    }
 #endif

    case RADIO_ID:
      if (telemetryRxBuffer[3] == 0xEA     // radio address
          && telemetryRxBuffer[5] == 0x10  // timing correction frame
      ) {
        getCrossfireTelemetryValue(6, (int32_t &)g_elrs_update_rate,4);
        getCrossfireTelemetryValue(10, g_elrs_lag,4);
      }
      break;

    default:
      // <Device address 0><Frame length 1><Type 2><Payload 3><CRC>
      // destination address and CRC are skipped
      runCrossfireTelemetryCallback(telemetryRxBuffer[2], telemetryRxBuffer + 2, telemetryRxBuffer[1] - 1);
      break;
  }

}

bool isCrossfireOutputBufferAvailable() {
  return outputTelemetryBufferSize == 0;
}

int processCrossfireTelemetryData(uint8_t data) {
  if (telemetryRxBufferCount == 0 && data != RADIO_ADDRESS) {
    TRACE("[XF] address 0x%02X error", data);
    telemetryErrors++;
    return 0;
  }

  if (telemetryRxBufferCount == 1 && (data < 2 || data > TELEMETRY_RX_PACKET_SIZE - 2)) {
    TRACE("[XF] length 0x%02X error", data);
    telemetryRxBufferCount = 0;
    telemetryErrors++;
    return 0 ;
  }

  if (telemetryRxBufferCount < TELEMETRY_RX_PACKET_SIZE) {
    telemetryRxBuffer[telemetryRxBufferCount++] = data;
  } else {
    TRACE("[XF] array size %d error", telemetryRxBufferCount);
    telemetryRxBufferCount = 0;
    telemetryErrors++;
  }

  if (telemetryRxBufferCount > 4) {
    uint8_t length = telemetryRxBuffer[1];
    if (length  > 0 && length + 2 == telemetryRxBufferCount) {
      processCrossfireTelemetryFrame();
      telemetryRxBufferCount = 0;

      //g_elrs_update_rate = length;
  
      return 1;
    }
  }

  return 0;
}

//void crossfireSetDefault(int index, uint8_t id, uint8_t subId) {

//#if 0

//  TelemetrySensor &telemetrySensor = g_model.telemetrySensors[index];

//  telemetrySensor.id = id;
//  telemetrySensor.instance = subId;

//  const CrossfireSensor &sensor = getCrossfireSensor(id, subId);
//  TelemetryUnit unit = sensor.unit;
//  if (unit == UNIT_GPS_LATITUDE || unit == UNIT_GPS_LONGITUDE)
//    unit = UNIT_GPS;
//  uint8_t prec = min<uint8_t>(2, sensor.precision);
//  telemetrySensor.init(sensor.name, unit, prec);
//  if (id == LINK_ID) {
//    telemetrySensor.logs = true;
//  }

//  storageDirty(EE_MODEL);
//#endif

//}

/**
 * Skip luaInputTelemetryFifo and luaCrossfireTelemetryPop() to save RAM and provide synchronous API instead
 */
/*
void (*crossfireTelemetryCallback)(uint8_t, uint8_t*, uint8_t);

void registerCrossfireTelemetryCallback(void (*callback)(uint8_t, uint8_t*, uint8_t)) {
  crossfireTelemetryCallback = callback;
}
*/


//inline void runCrossfireTelemetryCallback(uint8_t command, uint8_t* data, uint8_t length) {

//  if (command == 0x29) {
//    parseDeviceInfoMessage(data);
//  } else if (command == 0x2B) {
//    parseParameterInfoMessage(data, length);
//    if (allParamsLoaded < 1 || statusComplete == 0) {
//      fieldTimeout = 0; 
//    }
//  } else if (command == 0x2E) {
//    parseElrsInfoMessage(data);
//  }
//  //if (crossfireTelemetryCallback != 0) {
//  //  crossfireTelemetryCallback(command, data, length);
//  //}
//}


bool crossfireTelemetryPush(uint8_t command, uint8_t *data, uint8_t length) {
  // TRACE("crsfPush %x", command);
  if (isCrossfireOutputBufferAvailable()) {
    telemetryOutputPushByte(MODULE_ADDRESS);
    telemetryOutputPushByte(2 + length);  // 1(COMMAND) + data length + 1(CRC)
    telemetryOutputPushByte(command);     // COMMAND
    for (int i = 0; i < length; i++) {
      telemetryOutputPushByte(data[i]);
    }

    telemetryOutputPushByte(crc8(outputTelemetryBuffer + 2, 1 + length));
    telemetryOutputSetTrigger(command);
    return true;
  } else {
    return false;
  }
}




//uint32_t crsf_current_period()
//{
//    return CROSSFIRE_PERIODS[g_crossfile_baudrate_index];
//}

void crsf_init()
{

  g_crossfire.length = 0;
  setup_crsf_serial_port(CROSSFIRE_BAUDRATE,processCrossfireTelemetryData);
  SetPRTTimPeriod(PROTO_ELRS2);
  //g_crsf_inited = true;

 
}

void crsf_shutdown()
{
  //if(g_crsf_inited)
  //{
  //  g_crsf_inited = false;
    shutdown_crsf_serial_port();
 // }
}

void crsf_action()
{

    uint8_t dat;

    /*
    if (crsf_read_data(&data,1)>0) {
    //  LOG_TELEMETRY_WRITE_START();
      do {
        processCrossfireTelemetryData(data);
       // LOG_TELEMETRY_WRITE_BYTE(data);
      } while (crsf_read_data(&data,1)>0);
    }
    */
    #if USE_IE_UART_TX ||USE_DMA_UART
    if(crsf_is_sending())
    {
        return;
    }
    #endif

    setupPulsesCrossfire();
    if (g_crossfire.length > 0) {
      crsf_send_data(
          g_crossfire.pulses,
          g_crossfire.length);

           //crsf_wait_and_read();
    }

   //

}

