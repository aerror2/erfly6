 /*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Multiprotocol is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Multiprotocol.  If not, see <http://www.gnu.org/licenses/>.
 */
// Last sync with hexfet new_protocols/flysky_a7105.c dated 2015-09-28
#include "er9x.h"
#include "iface_a7105.h"
#include "menus.h"


#define AFHDS2A_TXPACKET_SIZE	37
#define AFHDS2A_RXPACKET_SIZE	37

 const AFHDS2A_telemetry AFHDS2A_tel[] = {
     {FST_TYPE_SNR, 3,0},
     {FST_TYPE_NOISE,3,0},
     {FST_TYPE_RSSI,3,0},
     {FST_TYPE_ERR,3,0},
     {FST_TYPE_TSSI,3,0},
     {FST_TYPE_INTV,4,2},
     {FST_TYPE_TEM,4,2},
     {FST_TYPE_RPM,5,0},
     {FST_TYPE_EXTV,4,2},
     {FST_TYPE_CELL,4,2},
     {FST_TYPE_BAT_CURR,4,2},
     {FST_TYPE_FUEL,4,2},
     {FST_TYPE_THRCAP,4,0},
     {FST_TYPE_CMP_HEAD,4,0},
     {FST_TYPE_CLIMB_RATE,4,0},
     {FST_TYPE_COG,4,0},
     {FST_TYPE_GPS_STATUS,4,0},
     {FST_TYPE_ACC_X,4,2},
     {FST_TYPE_ACC_Y,4,2},
     {FST_TYPE_ACC_Z,4,2},
     {FST_TYPE_ROLL,5,2},
     {FST_TYPE_PITCH,5,2},
     {FST_TYPE_YAW,5,2},
     {FST_TYPE_VERTICAL_SPEED,4,2},
     {FST_TYPE_GROUND_SPEED,4,2},
     {FST_TYPE_GPS_DIST,5,0},
     {FST_TYPE_ARMED,2,0},
     {FST_TYPE_FLIGHT_MODE,2,0},

     {FST_TYPE_PRES,4,2},
     {FST_TYPE_ODO1,5,0},
     {FST_TYPE_ODO2,5,0},
     {FST_TYPE_SPE,5,0},
     {FST_TYPE_TX_V,4,2},

//4 byte sensors
     {FST_TYPE_GPS_LAT,8,5},
     {FST_TYPE_GPS_LON,8,5},
     {FST_TYPE_GPS_ALT,8,2},
     {FST_TYPE_ALT,8,2},
     {FST_TYPE_ALT_MAX,8,2},
     {FST_TYPE_S85,10,0},
     {FST_TYPE_S86,10,0},
     {FST_TYPE_S87,10,0},
     {FST_TYPE_S88,10,0},
     {FST_TYPE_S89,10,0},
     {FST_TYPE_S8a,10,0}};

 int32_t AFHDS2A_tel_data[43];
 uint64_t AFHDS2A_tel_status = 0;
//int32_t AFHDS2A_tel_statuses[2]={0,0};
//#define  SET_TEL_STATUS(x) do{ if(x<32) AFHDS2A_tel_statuses[0]|=1<<x; else AFHDS2A_tel_statuses[1]|=1<<(x-32);}while(0);
//#define  CLR_TEL_STATUS(x) do{ if(x<32) AFHDS2A_tel_statuses[0]&=~(1<<x); else AFHDS2A_tel_statuses[1]&=~(1<<(x-32);}while(0));
//inline  bool IS_TEL_STATUS_SET(int  x) {if(x<32) return AFHDS2A_tel_statuses[0]&(1<<x); else return AFHDS2A_tel_statuses[1] &1<<(x-32);}

uint8_t AFHDS2A_tel_res_cnt[43] = {0};
 
 int16_t AltOffset;
 int16_t GAltOffset;
 


static void AFHDS2A_calc_channels() {
     uint8_t idx = 0;
     uint32_t rnd = ID.MProtocol_id;
     uint8_t i;
     while (idx < AFHDS2A_NUMFREQ)
     {
         uint8_t band_no = ((((idx << 1) | ((idx >> 1) & 0b01)) + ID.rx_tx_addr[3]) & 0b11);
         rnd = rnd * 0x0019660D + 0x3C6EF35F; // Randomization

         uint8_t next_ch = band_no * 41 + 1 + ((rnd >> idx) % 41); // Channel range: 1..164

         for (i = 0; i < idx; i++)
         {
             // Keep the distance 5 between the channels
             uint8_t distance;
             if (next_ch > hopping_frequency[i])
                 distance = next_ch - hopping_frequency[i];
             else
                 distance = hopping_frequency[i] - next_ch;

             if (distance < 5)
                 break;
         }

         if (i != idx)
             continue;

         hopping_frequency[idx++] = next_ch;
     }
 }
#define MAX_TEL_INDEX  (sizeof(AFHDS2A_tel) / sizeof(AFHDS2A_tel[0]))
 /*---------------------------------------------------------------------------*/
static uint8_t AFHDS2A_get_telemetry_index(uint8_t telID) {
   for (uint32_t i = 0; i < MAX_TEL_INDEX; i++)
     if (AFHDS2A_tel[i].id == telID) {
       return (uint8_t)i;
     }
   return 0xFF;
 }
 /*---------------------------------------------------------------------------*/
static void AFHDS2A_check_telemetry_status(void) {
   for (uint32_t i = 0; i < TEL_NUMBER; i++) {
     if (AFHDS2A_tel_res_cnt[i])
       AFHDS2A_tel_res_cnt[i]--;
     else
       AFHDS2A_tel_status &= ~((uint64_t)1 << i);
   }
 }
 /*---------------------------------------------------------------------------*/
static void processFlySkySensor(const uint8_t *packet, uint8_t type) {
   uint8_t buffer[8];
   uint16_t id = packet[0];
   const uint8_t instance = packet[1];
   int32_t value;

   // Load most likely value
   if (type == 0xAA)
     value = (packet[3] << 8) | packet[2];
   else
     value = (packet[6] << 24) | (packet[5] << 16) | (packet[4] << 8) | packet[3];

   if (id == FST_TYPE_GPS_FULL) {
     //(AC FRAME)[ID][inst][size][fix][sats][LAT]x4[LON]x4[ALT]x4
     AFHDS2A_tel_data[FST_IDX_GPS_STATUS] = packet[4];
     AFHDS2A_tel_status |= ((uint64_t)1 << FST_IDX_GPS_STATUS);
     AFHDS2A_tel_res_cnt[FST_IDX_GPS_STATUS] = TEL_COUNTER_MAX;
     for (uint8_t sensorID = FST_TYPE_GPS_LAT; sensorID <= FST_TYPE_GPS_ALT; sensorID++) {
       int index = 5 + (sensorID - FST_TYPE_GPS_LAT) * 4;
       buffer[0] = sensorID;
       buffer[1] = instance;
       buffer[2] = 4;
       memcpy(buffer + 3, packet + index, 4);
       processFlySkySensor(buffer, 0xAC);
     }
     return;
   } else if (id == FST_TYPE_VOLT_FULL) {
     //(AC FRAME)[ID][inst][size][ACC_X]x2[ACC_Y]x2[ACC_Z]x2[ROLL]x2[PITCH]x2[YAW]x2
     for (uint8_t sensorID = FST_TYPE_EXTV; sensorID <= FST_TYPE_RPM; sensorID++) {
       int index = 3 + (sensorID - FST_TYPE_EXTV) * 2;
       buffer[0] = sensorID;
       buffer[1] = instance;
       buffer[2] = packet[index];
       buffer[3] = packet[index + 1];
       processFlySkySensor(buffer, 0xAA);
     }
     return;
   } else if (id == FST_TYPE_ACC_FULL) {
     //(AC FRAME)[ID][inst][size]
     for (uint8_t sensorID = FST_TYPE_ACC_X; sensorID <= FST_TYPE_YAW; sensorID++) {
       int index = 3 + (sensorID - FST_TYPE_ACC_X) * 2;
       buffer[0] = sensorID;
       buffer[1] = instance;
       buffer[2] = packet[index];
       buffer[3] = packet[index + 1];
       processFlySkySensor(buffer, 0xAA);
     }
     return;
   }
   uint8_t tel_index = AFHDS2A_get_telemetry_index(id);
   AFHDS2A_tel_data[tel_index] = value;
   AFHDS2A_tel_status |= ((uint64_t)1 << tel_index);
   AFHDS2A_tel_res_cnt[tel_index] = TEL_COUNTER_MAX;

   switch (tel_index) {
   case FST_IDX_INTV:
     AFHDS2A_tel_data[tel_index] += g_model.IntVoffs;
     break;
   case FST_IDX_EXTV:
     AFHDS2A_tel_data[tel_index] += g_model.ExtVoffs;
     break;
   case FST_IDX_RPM:
     AFHDS2A_tel_data[tel_index] /= g_model.numBlades;
     break;
   case FST_IDX_ALT:
     AFHDS2A_tel_data[tel_index] += AltOffset;
     break;
   case FST_IDX_GPS_LAT:
   case FST_IDX_GPS_LON:
     AFHDS2A_tel_data[tel_index] /= 10; //6 DIGITS TO 5
     break;
   case FST_IDX_GPS_ALT:
     AFHDS2A_tel_data[tel_index] += GAltOffset;
     break;
   default:
     break;
   }
 }
/*---------------------------------------------------------------------------*/
static void processFlySkyPacket(const uint8_t * packet)
{
  const uint8_t * buffer = packet + 1;
  int sesnor = 0;
  while (sesnor++ < 7) {
    if (*buffer == FST_TYPE_UNKNOWN) break;
    processFlySkySensor(buffer, 0xAA);
    buffer += 4;
  }
}
/*---------------------------------------------------------------------------*/
static void processFlySkyPacketAC(const uint8_t * packet)
{
  const uint8_t * buffer = packet + 1;
  while (buffer - packet < 26) 
  {
    if (*buffer == FST_TYPE_UNKNOWN) break;
    uint8_t size = buffer[2];
    processFlySkySensor(buffer, 0xAC);
    buffer += size + 3;
  }
}
/*---------------------------------------------------------------------------*/
static void AFHDS2A_update_telemetry() {
#define K 100
  static uint32_t Dacc = 60 * K;
  uint8_t in = A7105_ReadReg(A7105_1D_RSSI_THOLD);
  if (in < 60)
    in = 60;
  Dacc = Dacc + in - AFHDS2A_tel_data[FST_IDX_TSSI];
  AFHDS2A_tel_data[FST_IDX_TSSI] = Dacc / K;
  AFHDS2A_tel_status |= ((uint64_t)1 << FST_IDX_TSSI);
  AFHDS2A_tel_res_cnt[FST_IDX_TSSI] = TEL_COUNTER_MAX;

  // AA | TXID | rx_id | sensor id | sensor # | value 16 bit big endian | sensor id ......
  // AC | TXID | rx_id | sensor id | sensor # | length | bytes | sensor id ......
  if (packet[0] == 0xAA)
    processFlySkyPacket(&packet[8]);
  else if (packet[0] == 0xAC)
    processFlySkyPacketAC(&packet[8]);
}
/*---------------------------------------------------------------------------*/
static void AFHDS2A_build_bind_packet(void) {
   uint8_t ch;
   uint8_t phase = RadioState & 0x0F;
   memcpy(&packet[1], ID.rx_tx_addr, 4);
   memset(&packet[5], 0xff, 4);
   packet[10] = 0x00;
   for (ch = 0; ch < AFHDS2A_NUMFREQ; ch++)
     packet[11 + ch] = hopping_frequency[ch];
   memset(&packet[27], 0xff, 10);
   packet[37] = 0x00;
   switch (phase) {
   case AFHDS2A_BIND1:
     packet[0] = 0xbb;
     packet[9] = 0x01;
     break;
   case AFHDS2A_BIND2:
   case AFHDS2A_BIND3:
   case AFHDS2A_BIND4:
     packet[0] = 0xbc;
     if (phase == AFHDS2A_BIND4) {
       memcpy(&packet[5], &g_model.rxID, 4);
       memset(&packet[11], 0xff, 16);
     }
     packet[9] = phase - 1;
     if (packet[9] > 0x02)
       packet[9] = 0x02;
     packet[27] = 0x01;
     packet[28] = 0x80;
     break;
   }
 }
/*---------------------------------------------------------------------------*/
void AFHDS2A_build_packet(uint8_t type) {
   memcpy(&packet[1], ID.rx_tx_addr, 4);
   memcpy(&packet[5], g_model.rxID, 4);
   switch (type) {
   case AFHDS2A_PACKET_STICKS:
     packet[0] = 0x58;
     for (uint8_t ch = 0; ch < 14; ch++) {
       uint16_t channelMicros;
       if (g_model.failsafeRepeat)
         channelMicros = convert_failsafe_ppm(ch);
       else
         channelMicros = g_chans512[ch] / 2 + RADIO_PPM_CENTER;
       packet[9 + ch * 2] = channelMicros & 0xFF;
       packet[10 + ch * 2] = (channelMicros >> 8) & 0xFF;
     }
     break;
   case AFHDS2A_PACKET_FAILSAFE:
     packet[0] = 0x56;
     for (uint8_t ch = 0; ch < 14; ch++) {
       if (g_model.failsafeMode) { // Failsafe values
         uint16_t failsafeMicros = convert_failsafe_ppm(ch);
         packet[9 + ch * 2] = failsafeMicros & 0xff;
         packet[10 + ch * 2] = (failsafeMicros >> 8) & 0xff;
       } else { // no values
         packet[9 + ch * 2] = 0xff;
         packet[10 + ch * 2] = 0xff;
       }
     }
     break;
   case AFHDS2A_PACKET_SETTINGS:
     packet[0] = 0xaa;
     packet[9] = 0xfd;
     packet[10] = 0xff;
     if (g_model.ServoFreq < 50 || g_model.ServoFreq > 400)
       g_model.ServoFreq = 50; // default is 50Hz
     packet[11] = g_model.ServoFreq;
     packet[12] = g_model.ServoFreq >> 8;
     if (g_model.PPMOut)
       packet[13] = 0x01; // PPM output enabled
     else
       packet[13] = 0x00;
     packet[14] = 0x00;
     for (uint8_t i = 15; i < 37; i++)
       packet[i] = 0xff;
     packet[18] = 0x05; // ?
     packet[19] = 0xdc; // ?
     packet[20] = 0x05; // ?
     if (g_model.IS_BUS)
       packet[21] = 0xdd; // SBUS output enabled
     else
       packet[21] = 0xde; // IBUS
     break;
   }
   if (hopping_frequency_no >= AFHDS2A_NUMFREQ)
     packet[37] = 0x00;
   else
     packet[37] = 0; // hopping_frequency_no+2;
 }
/*---------------------------------------------------------------------------*/
void ActionAFHDS2A(void) {
  uint8_t Channel;
  static uint8_t packet_type;
  //static uint16_t telem_counter;
  static uint16_t packet_counter = 0;
  RadioActive = true; 
  A7105_AdjustLOBaseFreq();

  if (IS_BIND_DONE)
    RadioState = (RadioState & 0xF0) | AFHDS2A_DATA;
  switch (RadioState) {
  case ((TIM_CALL << CALLER) | (SEND << SEND_RES) | (AFHDS2A_BIND1)):
  case ((TIM_CALL << CALLER) | (SEND << SEND_RES) | (AFHDS2A_BIND2)):
  case ((TIM_CALL << CALLER) | (SEND << SEND_RES) | (AFHDS2A_BIND3)):
  case ((TIM_CALL << CALLER) | (RES << SEND_RES) | (AFHDS2A_BIND1)):
  case ((TIM_CALL << CALLER) | (RES << SEND_RES) | (AFHDS2A_BIND2)):
  case ((TIM_CALL << CALLER) | (RES << SEND_RES) | (AFHDS2A_BIND3)):
    goto SendBIND_;

  case ((TIM_CALL << CALLER) | (SEND << SEND_RES) | (AFHDS2A_BIND4)):
    goto SendBIND4_;

  case ((GPIO_CALL << CALLER) | (SEND << SEND_RES) | (AFHDS2A_BIND1)):
  case ((GPIO_CALL << CALLER) | (SEND << SEND_RES) | (AFHDS2A_BIND2)):
  case ((GPIO_CALL << CALLER) | (SEND << SEND_RES) | (AFHDS2A_BIND3)):
    goto EndSendBIND123_;

  case ((GPIO_CALL << CALLER) | (RES << SEND_RES) | (AFHDS2A_BIND1)):
  case ((GPIO_CALL << CALLER) | (RES << SEND_RES) | (AFHDS2A_BIND2)):
  case ((GPIO_CALL << CALLER) | (RES << SEND_RES) | (AFHDS2A_BIND3)):
    goto ResBIND123_;

  case ((TIM_CALL << CALLER) | (SEND << SEND_RES) | (AFHDS2A_DATA)):
  case ((TIM_CALL << CALLER) | (RES << SEND_RES) | (AFHDS2A_DATA)):
    goto SendData_;

  case ((GPIO_CALL << CALLER) | (SEND << SEND_RES) | (AFHDS2A_DATA)):
    goto EndSendData_;

  case ((GPIO_CALL << CALLER) | (RES << SEND_RES) | (AFHDS2A_DATA)):
    goto ResData_;

  default:
    goto Exit_;
  }
//--------------------------------------------------------------------------
SendBIND4_: //--------------------------------------------------------------
  bind_phase++;
  if (bind_phase >= 4) {
    hopping_frequency_no = 1;
    RadioState = (RadioState & 0xF0) | AFHDS2A_DATA;
    SETBIT(RadioState, SEND_RES, SEND);
    BIND_DONE;
    goto Exit_;
  }
SendBIND_: //---------------------------------------------------------------
  AFHDS2A_build_bind_packet();
  Channel = (packet_count % 2 ? 0x0d : 0x8c);
  SETBIT(RadioState, SEND_RES, SEND);
  goto Send_;
EndSendBIND123_: //---------------------------------------------------------
  A7105_SetPower();
  A7105_SetTxRxMode(TXRX_OFF); // Turn LNA off since we are in near range and we want to prevent swamping
  A7105_Strobe(A7105_RX);
  EnableGIO();
  RadioState++;
  if ((RadioState & 0x0F) > AFHDS2A_BIND3)
    RadioState = (RadioState & 0xF0) | AFHDS2A_BIND1;
  SETBIT(RadioState, SEND_RES, RES);
    goto Exit_;
ResBIND123_: //-----------------------------------------------------------
  A7105_ReadData(AFHDS2A_RXPACKET_SIZE);
  if ((packet[0] == 0xbc) & (packet[9] == 0x01)) {
    for (uint8_t i = 0; i < 4; i++) {
      g_model.rxID[i] = packet[5 + i];
    }
    RadioState = (RadioState & 0xF0) | AFHDS2A_BIND4;
    bind_phase = 0;
    SETBIT(RadioState, SEND_RES, SEND);
  }
    goto Exit_;
SendData_: //--------------------------------------------------------------
  Channel = hopping_frequency[hopping_frequency_no++];
  AFHDS2A_build_packet(packet_type);
  SETBIT(RadioState, SEND_RES, SEND);
  if (hopping_frequency_no >= AFHDS2A_NUMFREQ) {
    hopping_frequency_no = 0;
    goto SendNoAntSwitch_;
  }
  goto Send_;
EndSendData_: //-----------------------------------------------------------
  A7105_SetPower();
  A7105_SetTxRxMode(RX_EN);
  A7105_Strobe(A7105_RX);
  if (!(packet_counter % 1313))
    packet_type = AFHDS2A_PACKET_SETTINGS;
  else if (!(packet_counter % 1569))
    packet_type = AFHDS2A_PACKET_FAILSAFE;
  else
    packet_type = AFHDS2A_PACKET_STICKS;
  SETBIT(RadioState, SEND_RES, RES);
  EnableGIO();
  AFHDS2A_check_telemetry_status();
  SETBIT(RadioState, SEND_RES, RES);
    goto Exit_;
ResData_: //-----------------------------------------------------------
  if ((A7105_ReadReg(A7105_00_MODE) & 0x20) == 0) {  // test CRC&CRF bits
  A7105_ReadData(AFHDS2A_RXPACKET_SIZE);
    if (packet[0] == 0xAA && packet[9] == 0xFC)      // RX is asking for settings
      packet_type = AFHDS2A_PACKET_SETTINGS;
    else if (packet[0] == 0xAA && packet[9] == 0xFD) // RX is asking for FailSafe
      packet_type = AFHDS2A_PACKET_FAILSAFE;
    else if (packet[0] == 0xAA || packet[0] == 0xAC) {
      if ((!memcmp(&packet[1], ID.rx_tx_addr, 4)) &
          (!memcmp(&packet[5], g_model.rxID, 4))) {  // Validate TX RX address
        AFHDS2A_update_telemetry();
      }
    }
  }
  SETBIT(RadioState, SEND_RES, SEND);
    goto Exit_;
Send_: //---------------------------------------------------------------
  A7105_AntSwitch();
SendNoAntSwitch_:
  A7105_WriteData(AFHDS2A_TXPACKET_SIZE, Channel);
  EnableGIO();
  packet_count++;
  packet_counter++;
//  return;
Exit_: //---------------------------------------------------------------
RadioActive = false;
}
/*---------------------------------------------------------------------------*/
void initAFHDS2A(void) {
  RadioState      = ((TIM_CALL << CALLER) | (SEND << SEND_RES) | (AFHDS2A_DATA));
  ID.MProtocol_id = GetChipID();
  AFHDS2A_calc_channels();
  A7105_Init();
  SetPRTTimPeriod(PROTO_AFHDS2A);
  packet_count         = 0;
  hopping_frequency_no = 0;
}