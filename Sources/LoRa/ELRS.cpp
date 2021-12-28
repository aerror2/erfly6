#include "ELRS.h"
#include "OTA.h"
#include "../er9x.h"
#include "SX127x.h"
#include "common.h"
#include "crc.h"
#include "config.h"
#include "FHSS.h"
#include "POWERMGNT.h"
#include "telemetry_protocol.h"
#include "LQCALC.h"
#include "stubborn_receiver.h"
#include "stubborn_sender.h"

#if defined(Regulatory_Domain_AU_915) || defined(Regulatory_Domain_EU_868) || defined(Regulatory_Domain_IN_866) || defined(Regulatory_Domain_FCC_915) || defined(Regulatory_Domain_AU_433) || defined(Regulatory_Domain_EU_433)
#include "SX127xDriver.h"
SX127xDriver Radio;
#elif defined(Regulatory_Domain_ISM_2400)
#include "SX1280Driver.h"
SX1280Driver Radio;
#endif
//// CONSTANTS ////
#define MSP_PACKET_SEND_INTERVAL 10LU

#ifndef TLM_REPORT_INTERVAL_MS
#define TLM_REPORT_INTERVAL_MS 320LU // Default to 320ms
#endif

GENERIC_CRC14 ota_crc(ELRS_CRC14_POLY);
//CRSF crsf;
POWERMGNT POWERMGNT;
//MSP msp;
//ELRS_EEPROM eeprom;
tx_config_t config;

CRSF crsf;


volatile uint8_t NonceTX;

#if defined(PLATFORM_ESP8266) || defined(PLATFORM_ESP32)
unsigned long rebootTime = 0;
extern bool webserverPreventAutoStart;
#endif
//// MSP Data Handling ///////
bool NextPacketIsMspData = false;  // if true the next packet will contain the msp data

////////////SYNC PACKET/////////
/// sync packet spamming on mode change vars ///
#define syncSpamAResidualTimeMS 500 // we spam some more after rate change to help link get up to speed
#define syncSpamAmount 3
volatile uint8_t syncSpamCounter = 0;
uint32_t rfModeLastChangedMS = 0;
uint32_t SyncPacketLastSent = 0;
////////////////////////////////////////////////

volatile uint32_t LastTLMpacketRecvMillis = 0;
uint32_t TLMpacketReported = 0;

LQCALC<10> LQCalc;

volatile bool busyTransmitting;
static volatile bool ModelUpdatePending;
volatile bool connectionHasModelMatch = true;

//bool InBindingMode = false;
uint8_t MSPDataPackage[5];
static uint8_t BindingSendCount;
bool RxWiFiReadyToSend = false;
#if defined(USE_TX_BACKPACK)
bool TxBackpackWiFiReadyToSend = false;
bool VRxBackpackWiFiReadyToSend = false;
#endif

static TxTlmRcvPhase_e TelemetryRcvPhase = ttrpTransmitting;
StubbornReceiver TelemetryReceiver(ELRS_TELEMETRY_MAX_PACKAGES);
StubbornSender MspSender(ELRS_MSP_MAX_PACKAGES);
//uint8_t CRSFinBuffer[CRSF_MAX_PACKET_LEN+1];

#define   InBindingMode (pxxFlag & PXX_BIND)

//////////// DYNAMIC TX OUTPUT POWER ////////////

#if !defined(DYNPOWER_THRESH_UP)
  #define DYNPOWER_THRESH_UP              15
#endif
#if !defined(DYNPOWER_THRESH_DN)
  #define DYNPOWER_THRESH_DN              21
#endif
#define DYNAMIC_POWER_MIN_RECORD_NUM       5 // average at least this number of records
#define DYNAMIC_POWER_BOOST_LQ_THRESHOLD  20 // If LQ is dropped suddenly for this amount (relative), immediately boost to the max power configured.
#define DYNAMIC_POWER_BOOST_LQ_MIN        50 // If LQ is below this value (absolute), immediately boost to the max power configured.
#define DYNAMIC_POWER_MOVING_AVG_K         8 // Number of previous values for calculating moving average. Best with power of 2.
static int32_t dynamic_power_rssi_sum;
static int32_t dynamic_power_rssi_n;
static int32_t dynamic_power_avg_lq;
static bool dynamic_power_updated;


void ICACHE_RAM_ATTR HandlePrepareForTLM()
{
  uint8_t modresult = (NonceTX + 1) % TLMratioEnumToValue(ExpressLRS_currAirRate_Modparams->TLMinterval);
  // If next packet is going to be telemetry, start listening to have a large receive window (time-wise)
  if (ExpressLRS_currAirRate_Modparams->TLMinterval != TLM_RATIO_NO_TLM && modresult == 0)
  {
    Radio.RXnb();
    TelemetryRcvPhase = ttrpInReceiveMode;
  }
}


void ICACHE_RAM_ATTR HandleFHSS()
{
  uint8_t modresult = (NonceTX + 1) % ExpressLRS_currAirRate_Modparams->FHSShopInterval;
  // If the next packet should be on the next FHSS frequency, do the hop
  if (!InBindingMode && modresult == 0)
  {
    Radio.SetFrequencyReg(FHSSgetNextFreq());
  }
}



void ICACHE_RAM_ATTR ProcessTLMpacket()
{
  uint16_t inCRC = (((uint16_t)Radio.RXdataBuffer[0] & 0b11111100) << 6) | Radio.RXdataBuffer[7];

  Radio.RXdataBuffer[0] &= 0b11;
  uint16_t calculatedCRC = ota_crc.calc(Radio.RXdataBuffer, 7, CRCInitializer);

  uint8_t type = Radio.RXdataBuffer[0] & TLM_PACKET;
  uint8_t TLMheader = Radio.RXdataBuffer[1];

  if ((inCRC != calculatedCRC))
  {
    DBGLN("TLM crc error");
    return;
  }

  if (type != TLM_PACKET)
  {
    DBGLN("TLM type error %d", type);
    return;
  }

  if (connectionState != connected)
  {
    connectionState = connected;
    DBGLN("got downlink conn");
  }

  LastTLMpacketRecvMillis = g_tmr10ms;
  LQCalc.add();

    switch(TLMheader & ELRS_TELEMETRY_TYPE_MASK)
    {
        case ELRS_TELEMETRY_TYPE_LINK:

          #if 0
            // Antenna is the high bit in the RSSI_1 value
            // RSSI received is signed, inverted polarity (positive value = -dBm)
            // OpenTX's value is signed and will display +dBm and -dBm properly
            crsf.LinkStatistics.uplink_RSSI_1 = -(Radio.RXdataBuffer[2] & 0x7f);
            crsf.LinkStatistics.uplink_RSSI_2 = -(Radio.RXdataBuffer[3] & 0x7f);
            crsf.LinkStatistics.uplink_SNR = Radio.RXdataBuffer[4];
            crsf.LinkStatistics.uplink_Link_quality = Radio.RXdataBuffer[5];
            crsf.LinkStatistics.downlink_SNR = Radio.LastPacketSNR;
            crsf.LinkStatistics.downlink_RSSI = Radio.LastPacketRSSI;
            crsf.LinkStatistics.active_antenna = Radio.RXdataBuffer[2] >> 7;
         #endif
            connectionHasModelMatch = Radio.RXdataBuffer[3] >> 7;
            // -- uplink_TX_Power is updated when sending to the handset, so it updates when missing telemetry
            // -- rf_mode is updated when we change rates
            // -- downlink_Link_quality is updated before the LQ period is incremented
            MspSender.ConfirmCurrentPayload(Radio.RXdataBuffer[6] == 1);
       
            dynamic_power_updated = true;
            break;

        case ELRS_TELEMETRY_TYPE_DATA:
           
            TelemetryReceiver.ReceiveData(TLMheader >> ELRS_TELEMETRY_SHIFT, Radio.RXdataBuffer + 2);
           
            break;
    }
}


uint8_t adjustPacketRateForBaud(uint8_t rate)
{
  #if defined(Regulatory_Domain_ISM_2400)
    // Packet rate limited to 250Hz if we are on 115k baud
    if (crsf.GetCurrentBaudRate() == 115200) {
      while(rate < RATE_MAX) {
        expresslrs_mod_settings_s *const ModParams = get_elrs_airRateConfig(rate);
        if (ModParams->enum_rate >= RATE_250HZ) {
          break;
        }
        rate++;
      }
    }
  #endif
  return rate;
}


void ICACHE_RAM_ATTR SetRFLinkRate(uint8_t index) // Set speed of RF link (hz)
{
  index = adjustPacketRateForBaud(index);
  expresslrs_mod_settings_s *const ModParams = get_elrs_airRateConfig(index);
  expresslrs_rf_pref_params_s *const RFperf = get_elrs_RFperfParams(index);
  bool invertIQ = UID[5] & 0x01;
  if ((ModParams == ExpressLRS_currAirRate_Modparams)
    && (RFperf == ExpressLRS_currAirRate_RFperfParams)
    && (invertIQ == Radio.IQinverted))
    return;

  DBGLN("set rate %u", index);

#if 0
  hwTimer.updateInterval(ModParams->interval);
 #endif
  
  Radio.Config(ModParams->bw, ModParams->sf, ModParams->cr, GetInitialFreq(), ModParams->PreambleLen, invertIQ, ModParams->PayloadLength);

  ExpressLRS_currAirRate_Modparams = ModParams;
  ExpressLRS_currAirRate_RFperfParams = RFperf;
  
  
 // crsf.LinkStatistics.rf_Mode = (uint8_t)RATE_4HZ - (uint8_t)ExpressLRS_currAirRate_Modparams->enum_rate;
  //crsf.setSyncParams(ModParams->interval);


  connectionState = disconnected;
  rfModeLastChangedMS = g_tmr10ms;
}



void ICACHE_RAM_ATTR RXdoneISR()
{
  // There isn't enough time to receive two packets during one telemetry slot
  // Stop receiving to prevent a second packet preamble from starting a second receive
  Radio.SetTxIdleMode();
  ProcessTLMpacket();
  busyTransmitting = false;
}

void ICACHE_RAM_ATTR TXdoneISR()
{
  busyTransmitting = false;
  HandleFHSS();
  HandlePrepareForTLM();
}

static void ChangeRadioParams()
{
  ModelUpdatePending = false;

  SetRFLinkRate(config.model_config.rate);
  POWERMGNT.CurrentPower = ((PowerLevels_e)config.model_config.power);
  OtaSetSwitchMode((OtaSwitchMode_e)config.model_config.switchMode);
  // TLM interval is set on the next SYNC packet
}


void ICACHE_RAM_ATTR GenerateSyncPacketData()
{
  const uint8_t SwitchEncMode = config.model_config.switchMode & 0b11;
  uint8_t Index;
  if (syncSpamCounter)
  {
    Index = (config.model_config.rate & 0b11);
  }
  else
  {
    Index = (ExpressLRS_currAirRate_Modparams->index & 0b11);
  }

  // TLM ratio is boosted for one sync cycle when the MspSender goes active

  ExpressLRS_currAirRate_Modparams->TLMinterval = (expresslrs_tlm_ratio_e)config.model_config.tlm;
  uint8_t TLMrate = (ExpressLRS_currAirRate_Modparams->TLMinterval & 0b111);

  Radio.TXdataBuffer[0] = SYNC_PACKET & 0b11;
  Radio.TXdataBuffer[1] = FHSSgetCurrIndex();
  Radio.TXdataBuffer[2] = NonceTX;
  Radio.TXdataBuffer[3] = (Index << 6) + (TLMrate << 3) + (SwitchEncMode << 1);
  Radio.TXdataBuffer[4] = UID[3];
  Radio.TXdataBuffer[5] = UID[4];
  Radio.TXdataBuffer[6] = UID[5];
  // For model match, the last byte of the binding ID is XORed with the inverse of the modelId
  if (!InBindingMode //&& config.GetModelMatch()
  )
  {
    Radio.TXdataBuffer[6] ^= (~GetChipID()) & MODELMATCH_MASK;
  }

  SyncPacketLastSent = g_tmr10ms;
  if (syncSpamCounter)
    --syncSpamCounter;
}



void ICACHE_RAM_ATTR SendRCdataToRF()
{
  uint32_t now = g_tmr10ms;
  static uint8_t syncSlot;
#if defined(NO_SYNC_ON_ARM)
  uint32_t SyncInterval = 250;
  bool skipSync = IsArmed() || InBindingMode;
#else
  uint32_t SyncInterval = (connectionState == connected) ? ExpressLRS_currAirRate_RFperfParams->SyncPktIntervalConnected : ExpressLRS_currAirRate_RFperfParams->SyncPktIntervalDisconnected;
  bool skipSync = InBindingMode;
#endif

  uint8_t NonceFHSSresult = NonceTX % ExpressLRS_currAirRate_Modparams->FHSShopInterval;
  bool WithinSyncSpamResidualWindow = now - rfModeLastChangedMS < syncSpamAResidualTimeMS/10;

  // Sync spam only happens on slot 1 and 2 and can't be disabled
  if ((syncSpamCounter || WithinSyncSpamResidualWindow) && (NonceFHSSresult == 1 || NonceFHSSresult == 2))
  {
    GenerateSyncPacketData();
    syncSlot = 0; // reset the sync slot in case the new rate (after the syncspam) has a lower FHSShopInterval
  }
  // Regular sync rotates through 4x slots, twice on each slot, and telemetry pushes it to the next slot up
  // But only on the sync FHSS channel and with a timed delay between them
  else if ((!skipSync) && ((syncSlot / 2) <= NonceFHSSresult) && (now - SyncPacketLastSent > SyncInterval/10) && (Radio.currFreq == GetInitialFreq()))
  {
    GenerateSyncPacketData();
    syncSlot = (syncSlot + 1) % (ExpressLRS_currAirRate_Modparams->FHSShopInterval * 2);
  }
  else
  {

    if (NextPacketIsMspData && MspSender.IsActive())
    {
      uint8_t *data;
      uint8_t maxLength;
      uint8_t packageIndex;
      MspSender.GetCurrentPayload(&packageIndex, &maxLength, &data);
      Radio.TXdataBuffer[0] = MSP_DATA_PACKET & 0b11;
      Radio.TXdataBuffer[1] = packageIndex;
      Radio.TXdataBuffer[2] = maxLength > 0 ? *data : 0;
      Radio.TXdataBuffer[3] = maxLength >= 1 ? *(data + 1) : 0;
      Radio.TXdataBuffer[4] = maxLength >= 2 ? *(data + 2) : 0;
      Radio.TXdataBuffer[5] = maxLength >= 3 ? *(data + 3): 0;
      Radio.TXdataBuffer[6] = maxLength >= 4 ? *(data + 4): 0;
      // send channel data next so the channel messages also get sent during msp transmissions
      NextPacketIsMspData = false;
      // counter can be increased even for normal msp messages since it's reset if a real bind message should be sent
      BindingSendCount++;
      // If the telemetry ratio isn't already 1:2, send a sync packet to boost it
      // to add bandwidth for the reply
      if (ExpressLRS_currAirRate_Modparams->TLMinterval != TLM_RATIO_1_2)
        syncSpamCounter = 1;
    }
    else

    {
      // always enable msp after a channel package since the slot is only used if MspSender has data to send
      NextPacketIsMspData = true;
      for (uint8_t ch = 0; ch < 14; ch++) {
       uint16_t channelMicros;
    
       channelMicros =fmap( g_chans512[ch] + 1024 , 0,2048,CRSF_CHANNEL_VALUE_MIN,CRSF_CHANNEL_VALUE_MAX);
       crsf.ChannelDataIn[ch] = channelMicros;

      }

      crsf.ChannelDataIn[14]  = CRSF_CHANNEL_VALUE_MIN;
      crsf.ChannelDataIn[15]  = CRSF_CHANNEL_VALUE_MIN;

      PackChannelData(Radio.TXdataBuffer, &crsf, TelemetryReceiver.GetCurrentConfirm(),
        NonceTX, TLMratioEnumToValue(ExpressLRS_currAirRate_Modparams->TLMinterval));
    }
  }

  // artificially inject the low bits of the nonce on data packets, this will be overwritten with the CRC after it's calculated
  if (Radio.TXdataBuffer[0] == RC_DATA_PACKET && OtaSwitchModeCurrent == smHybridWide)
    Radio.TXdataBuffer[0] |= NonceFHSSresult << 2;

  ///// Next, Calculate the CRC and put it into the buffer /////
  uint16_t crc = ota_crc.calc(Radio.TXdataBuffer, 7, CRCInitializer);
  Radio.TXdataBuffer[0] = (Radio.TXdataBuffer[0] & 0b11) | ((crc >> 6) & 0b11111100);
  Radio.TXdataBuffer[7] = crc & 0xFF;

  Radio.TXnb();
}




void elrs_init()
{

  FHSSrandomiseFHSSsequence(uidMacSeedGet());

  Radio.RXdoneCallback = &RXdoneISR;
  Radio.TXdoneCallback = &TXdoneISR;

  //config.Load();
  Radio.currFreq = GetInitialFreq(); //set frequency first or an error will occur!!! 
  bool init_success = Radio.Begin();
 
  if(!init_success)
      return;
       
 // POWERMGNT.init();
  POWERMGNT.FanEnableThreshold  =((PowerLevels_e)config.powerFanThreshold);
  ChangeRadioParams();

  
}

void elrs_action()
{
  #ifdef FEATURE_OPENTX_SYNC
  // Sync OpenTX to this point
  crsf.JustSentRFpacket();
  #endif

  // Nonce advances on every timer tick
  if (!InBindingMode)
    NonceTX++;

  // If HandleTLM has started Receive mode, TLM packet reception should begin shortly
  // Skip transmitting on this slot
  if (TelemetryRcvPhase == ttrpInReceiveMode)
  {
    TelemetryRcvPhase = ttrpWindowInProgress;
  //  crsf.LinkStatistics.downlink_Link_quality = LQCalc.getLQ();
    LQCalc.inc();
    return;
  }
  // TLM packet reception was the previous slot, transmit this slot (below)
  if (TelemetryRcvPhase == ttrpWindowInProgress)
  {
    // Stop Receive mode if it is still active
    Radio.SetTxIdleMode();
    TelemetryRcvPhase = ttrpTransmitting;
  }

  // Do not send a stale channels packet to the RX if one has not been received from the handset
  // *Do* send data if a packet has never been received from handset and the timer is running
  //     this is the case when bench testing and TXing without a handset
  //uint32_t lastRcData = crsf.GetRCdataLastRecv();
 // if (!lastRcData || (micros() - lastRcData < 1000000))
  {
    busyTransmitting = true;
    SendRCdataToRF();
  }
}