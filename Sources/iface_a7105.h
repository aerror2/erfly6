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
#ifndef _IFACE_A7105_H_
#define _IFACE_A7105_H_
#define _BV(bit) (1 << (bit))
/*------------------------Telemetry type--------------------------------------*/
#define FST_TYPE_INTV			0x00    // Internal Voltage
#define FST_TYPE_TEM			0x01    // Temperature
#define FST_TYPE_RPM			0x02    // RPM
#define FST_TYPE_EXTV			0x03    // External Voltage
#define FST_TYPE_CELL			0x04    // Avg Cell voltage
#define FST_TYPE_BAT_CURR		0x05    // battery current A * 100
#define FST_TYPE_FUEL			0x06	// remaining battery percentage / mah drawn otherwise or fuel level no unit!
#define FST_TYPE_THRCAP			0x07	// throttle value / battery capacity
#define FST_TYPE_CMP_HEAD		0x08 	//Heading  0..360 deg, 0=north 2bytes
#define FST_TYPE_CLIMB_RATE 		0x09  	//2 bytes m/s *100
#define FST_TYPE_COG			0x0a 	//2 bytes  Course over ground(NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. unknown max uint
#define FST_TYPE_GPS_STATUS		0x0b 	//2 bytes
#define FST_TYPE_ACC_X			0x0c 	//2 bytes m/s *100 signed
#define FST_TYPE_ACC_Y			0x0d 	//2 bytes m/s *100 signed
#define FST_TYPE_ACC_Z			0x0e 	//2 bytes m/s *100 signed
#define FST_TYPE_ROLL			0x0f 	//2 bytes deg *100 signed
#define FST_TYPE_PITCH			0x10 	//2 bytes deg *100 signed
#define FST_TYPE_YAW			0x11 	//2 bytes deg *100 signed
#define FST_TYPE_VERTICAL_SPEED	        0x12 	//2 bytes m/s *100
#define FST_TYPE_GROUND_SPEED		0x13 	//2 bytes m/s *100 different unit than build-in sensor
#define FST_TYPE_GPS_DIST		0x14 	//2 bytes dist from home m unsigned
#define FST_TYPE_ARMED			0x15 	//2 bytes
#define FST_TYPE_FLIGHT_MODE		0x16 	//2 bytes simple index listed below


#define FST_TYPE_PRES			0x41    // Pressure
#define FST_TYPE_ODO1			0x7c    // Odometer1
#define FST_TYPE_ODO2			0x7d    // Odometer2
#define FST_TYPE_SPE			0x7e    // Speed	//2byte km/h
#define FST_TYPE_TX_V			0x7f    // TX Voltage


//4 byte sensors
#define FST_TYPE_GPS_LAT		0x80 //4bytes signed WGS84 in degrees * 1E7
#define FST_TYPE_GPS_LON		0x81 //4bytes signed WGS84 in degrees * 1E7
#define FST_TYPE_GPS_ALT		0x82 //4bytes signed!!! GPS alt m*100
#define FST_TYPE_ALT			0x83 //4bytes signed!!! Alt m*100
#define FST_TYPE_ALT_MAX		0x84 //4bytes signed MaxAlt m*100
#define FST_TYPE_S85			0x85
#define FST_TYPE_S86			0x86
#define FST_TYPE_S87			0x87
#define FST_TYPE_S88			0x88
#define FST_TYPE_S89			0x89
#define FST_TYPE_S8a			0x8a


//#define FST_TYPE_ALT_FLYSKY		0xf9    // Altitude			//2 bytes signed in m
#define FST_TYPE_SNR			0xfa    // SNR
#define FST_TYPE_NOISE			0xfb    // Noise
#define FST_TYPE_RSSI			0xfc    // RSSI
#define FST_TYPE_ERR			0xfe    // Error rate
#define FST_TYPE_UNKNOWN		0xff
#define FST_TYPE_TSSI			0x30    // TSSI


#define FST_TYPE_GPS_FULL		0xfd
#define FST_TYPE_VOLT_FULL		0xf0
#define FST_TYPE_ACC_FULL		0xef
/*------------------------Telemetry index--------------------------------------*/
#define FST_IDX_SNR			0    // SNR
#define FST_IDX_NOISE			1    // Noise
#define FST_IDX_RSSI			2    // RSSI
#define FST_IDX_ERR			3    // Error rate
#define FST_IDX_TSSI			4    // TSSI

#define FST_IDX_INTV			5    // Internal Voltage
#define FST_IDX_TEM			6    // Temperature
#define FST_IDX_RPM			7    // RPM
#define FST_IDX_EXTV			8    // External Voltage
#define FST_IDX_CELL			9    // Avg Cell voltage
#define FST_IDX_BAT_CURR		10    // battery current A * 100
#define FST_IDX_FUEL			11    // remaining battery percentage / mah drawn otherwise or fuel level no unit!
#define FST_IDX_THRCAP			12    // throttle value / battery capacity
#define FST_IDX_CMP_HEAD		13    // Heading  0..360 deg, 0=north 2bytes
#define FST_IDX_CLIMB_RATE 		14    //2 bytes m/s *100
#define FST_IDX_COG			15   //2 bytes  Course over ground(NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. unknown max uint
#define FST_IDX_GPS_STATUS		16   //2 bytes
#define FST_IDX_ACC_X			17   //2 bytes m/s *100 signed
#define FST_IDX_ACC_Y			18   //2 bytes m/s *100 signed
#define FST_IDX_ACC_Z			19   //2 bytes m/s *100 signed
#define FST_IDX_ROLL			20   //2 bytes deg *100 signed
#define FST_IDX_PITCH			21   //2 bytes deg *100 signed
#define FST_IDX_YAW			22   //2 bytes deg *100 signed
#define FST_IDX_VERTICAL_SPEED	        23   //2 bytes m/s *100
#define FST_IDX_GROUND_SPEED		24   //2 bytes m/s *100 different unit than build-in sensor
#define FST_IDX_GPS_DIST		25   //2 bytes dist from home m unsigned
#define FST_IDX_ARMED			26   //2 bytes
#define FST_IDX_FLIGHT_MODE		27   //2 bytes simple index listed below

#define FST_IDX_PRES			28    // Pressure
#define FST_IDX_ODO1			29    // Odometer1
#define FST_IDX_ODO2			30    // Odometer2
#define FST_IDX_SPE			31    // Speed	//2byte km/h
#define FST_IDX_TX_V			32    // TX Voltage

//4 byte sensors
#define FST_IDX_GPS_LAT			33 //4bytes signed WGS84 in degrees * 1E7
#define FST_IDX_GPS_LON			34 //4bytes signed WGS84 in degrees * 1E7
#define FST_IDX_GPS_ALT			35 //4bytes signed!!! GPS alt m*100
#define FST_IDX_ALT			36 //4bytes signed!!! Alt m*100
#define FST_IDX_ALT_MAX			37 //4bytes signed MaxAlt m*100
#define FST_IDX_S85			38
#define FST_IDX_S86			39
#define FST_IDX_S87			40
#define FST_IDX_S88			41
#define FST_IDX_S89			42
#define FST_IDX_S8a			43

/*---------------------------------------------------------------------------*/

#define TX_EN            0b00000010
#define RX_EN            0b00000001
#define TXRX_OFF         0b00000011
#define AFHDS2A_NUMFREQ	 16
#define RADIO_PPM_CENTER 1500

//#define FORCE_AFHDS2A_TUNING 0
enum A7105_POWER
{
	A7105_POWER_0 = 0x00<<3 | 0x00,	// TXPOWER_100uW  = -23dBm == PAC=0 TBG=0
	A7105_POWER_1 = 0x00<<3 | 0x01,	// TXPOWER_300uW  = -20dBm == PAC=0 TBG=1
	A7105_POWER_2 = 0x00<<3 | 0x02,	// TXPOWER_1mW    = -16dBm == PAC=0 TBG=2
	A7105_POWER_3 = 0x00<<3 | 0x04,	// TXPOWER_3mW    = -11dBm == PAC=0 TBG=4
	A7105_POWER_4 = 0x01<<3 | 0x05,	// TXPOWER_10mW   =  -6dBm == PAC=1 TBG=5
	A7105_POWER_5 = 0x02<<3 | 0x07,	// TXPOWER_30mW   =   0dBm == PAC=2 TBG=7
	A7105_POWER_6 = 0x03<<3 | 0x07,	// TXPOWER_100mW  =   1dBm == PAC=3 TBG=7
	A7105_POWER_7 = 0x03<<3 | 0x07	// TXPOWER_150mW  =   1dBm == PAC=3 TBG=7
};

#define A7105_HIGH_POWER	A7105_POWER_7
#define	A7105_LOW_POWER		A7105_POWER_3
#define	A7105_RANGE_POWER	A7105_POWER_0
#define	A7105_BIND_POWER	A7105_POWER_0



extern uint8_t frskyUsrStreaming;
extern uint8_t protocol_flags;
extern bool RadioActive;

typedef struct AFHDS2A_telemetry_t {
uint8_t id;
uint8_t dig: 5;   
uint8_t prec:3;
}__attribute__((packed)) AFHDS2A_telemetry;

extern int32_t  AFHDS2A_tel_data [43];
extern uint64_t AFHDS2A_tel_status;
extern const AFHDS2A_telemetry AFHDS2A_tel[];



extern uint8_t  packet[40];
#define NUM_CHN 16

// Protocol variables
typedef union {
	uint32_t MProtocol_id; //tx id,
	uint8_t rx_tx_addr[4];
}ID_t;

extern ID_t ID;

extern uint8_t  packet_count;
extern uint8_t  phase;
extern uint8_t  bind_phase;
extern uint8_t  hopping_frequency[AFHDS2A_NUMFREQ];
extern uint8_t  hopping_frequency_no;
extern volatile uint8_t RadioState;

#define BIND_IN_PROGRESS	protocol_flags &= ~_BV(7)
#define BIND_DONE		protocol_flags |= _BV(7)
#define BIND_START		protocol_flags |= _BV(6)
#define BIND_STOP       	protocol_flags &= ~_BV(6)

#define IS_BIND_DONE		( ( protocol_flags & _BV(7) ) !=0 )
#define IS_BIND_IN_PROGRESS	( ( protocol_flags & _BV(7) ) ==0 )
#define IS_BIND_START		( ( protocol_flags & _BV(6) ) !=0 )
#define IS_BIND_STOP       	( ( protocol_flags & _BV(6) ) ==0 )


#define RANGE_FLAG_on		protocol_flags |= _BV(3)
#define RANGE_FLAG_off		protocol_flags &= ~_BV(3)
#define IS_RANGE_FLAG_on	( ( protocol_flags & _BV(3) ) !=0 )


enum A7105_State {
    A7105_SLEEP     = 0x80,
    A7105_IDLE      = 0x90,
    A7105_STANDBY   = 0xA0,
    A7105_PLL       = 0xB0,
    A7105_RX        = 0xC0,
    A7105_TX        = 0xD0,
    A7105_RST_WRPTR = 0xE0,
    A7105_RST_RDPTR = 0xF0,
};

enum {
    A7105_00_MODE         = 0x00,
    A7105_01_MODE_CONTROL = 0x01,
    A7105_02_CALC         = 0x02,
    A7105_03_FIFOI        = 0x03,
    A7105_04_FIFOII       = 0x04,
    A7105_05_FIFO_DATA    = 0x05,
    A7105_06_ID_DATA      = 0x06,
    A7105_07_RC_OSC_I     = 0x07,
    A7105_08_RC_OSC_II    = 0x08,
    A7105_09_RC_OSC_III   = 0x09,
    A7105_0A_CK0_PIN      = 0x0A,
    A7105_0B_GPIO1_PIN1   = 0x0B,
    A7105_0C_GPIO2_PIN_II = 0x0C,
    A7105_0D_CLOCK        = 0x0D,
    A7105_0E_DATA_RATE    = 0x0E,
    A7105_0F_PLL_I        = 0x0F,
    A7105_10_PLL_II       = 0x10,
    A7105_11_PLL_III      = 0x11,
    A7105_12_PLL_IV       = 0x12,
    A7105_13_PLL_V        = 0x13,
    A7105_14_TX_I         = 0x14,
    A7105_15_TX_II        = 0x15,
    A7105_16_DELAY_I      = 0x16,
    A7105_17_DELAY_II     = 0x17,
    A7105_18_RX           = 0x18,
    A7105_19_RX_GAIN_I    = 0x19,
    A7105_1A_RX_GAIN_II   = 0x1A,
    A7105_1B_RX_GAIN_III  = 0x1B,
    A7105_1C_RX_GAIN_IV   = 0x1C,
    A7105_1D_RSSI_THOLD   = 0x1D,
    A7105_1E_ADC          = 0x1E,
    A7105_1F_CODE_I       = 0x1F,
    A7105_20_CODE_II      = 0x20,
    A7105_21_CODE_III     = 0x21,
    A7105_22_IF_CALIB_I   = 0x22,
    A7105_23_IF_CALIB_II  = 0x23,
    A7105_24_VCO_CURCAL   = 0x24,
    A7105_25_VCO_SBCAL_I  = 0x25,
    A7105_26_VCO_SBCAL_II = 0x26,
    A7105_27_BATTERY_DET  = 0x27,
    A7105_28_TX_TEST      = 0x28,
    A7105_29_RX_DEM_TEST_I  = 0x29,
    A7105_2A_RX_DEM_TEST_II = 0x2A,
    A7105_2B_CPC          = 0x2B,
    A7105_2C_XTAL_TEST    = 0x2C,
    A7105_2D_PLL_TEST     = 0x2D,
    A7105_2E_VCO_TEST_I   = 0x2E,
    A7105_2F_VCO_TEST_II  = 0x2F,
    A7105_30_IFAT         = 0x30,
    A7105_31_RSCALE       = 0x31,
    A7105_32_FILTER_TEST  = 0x32,
};
#define A7105_0F_CHANNEL A7105_0F_PLL_I

enum A7105_MASK {
    A7105_MASK_FBCF = 1 << 4,
    A7105_MASK_VBCF = 1 << 3,
};

enum AFHDS2A
{
	PWM_IBUS = 0,
	PPM_IBUS = 1,
	PWM_SBUS = 2,
	PPM_SBUS = 3,
};

enum ePaketType{
	AFHDS2A_PACKET_STICKS,
	AFHDS2A_PACKET_SETTINGS,
	AFHDS2A_PACKET_FAILSAFE,
};

enum ePhase {
	AFHDS2A_BIND1 = 0,
	AFHDS2A_BIND2 = 1,
	AFHDS2A_BIND3 = 2,
	AFHDS2A_BIND4 = 3,
	AFHDS2A_DATA  = 4,
};

//typedef struct tel_AFHDS2A_t{}

#define CALLER    4
#define TIM_CALL  0
#define GPIO_CALL 1

#define SEND_RES  5
#define SEND      0
#define RES       1

#define PASS_INERRUPT  6
#define NOT_PASS       0
#define PASS           1

#define TEL_COUNTER_MAX  255
#define TEL_NUMBER       43
void A7105_Sleep(void);
void A7105_Init(void);
void A7105_AdjustLOBaseFreq(void);
void A7105_ReadData(uint8_t len);
uint8_t A7105_ReadReg(uint8_t address);
void A7105_WriteReg(uint8_t address, uint8_t data);
void A7105_SetPower();
void A7105_SetTxRxMode(uint8_t mode);
void A7105_Strobe(uint8_t address);
void A7105_WriteData(uint8_t len, uint8_t channel);
uint16_t convert_channel_ppm(uint8_t num);
uint16_t convert_failsafe_ppm(uint8_t num);
void A7105_AntSwitch(void);

#endif
