/*
 * drivers.cpp
 *
 *  Created on: 02 . 2019 .
 *      Author: KOSTYA
 */
/*
 * Author - Erez Raviv <erezraviv@gmail.com>
 *
 * Based on th9x -> http://code.google.com/p/th9x/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include "er9x.h"
#include "pulses.h"
#include "audio.h"
/*****************************************************************************/
Ti2c_buffer i2c_buffer;
/*****************************************************************************/
#define   PINC 0
/*---------------------------------------------------------------------------*/

/*****************************************************************************/
#define EE_TIME_WR 90
void ee_waite() {
	while (tmrEEPROM < EE_TIME_WR)   // make sure EEPROM is ready
	{
		if (Ee_lock & EE_TRIM_LOCK)  // Only if writing trim changes
		{
			mainSequence();          // Keep the controls running while waiting
		}
	}
}


static inline void __attribute__ ((always_inline))
eeprom_write_byte_cmp(uint8_t dat, uint16_t pointer_eeprom) {
	ee_waite();
	uint8_t buff[3];
	buff[0] = (uint8_t) ((pointer_eeprom) >> 8);
	buff[1] = (uint8_t) ((pointer_eeprom) & 0xff);
	buff[2] = (dat);
	i2c_buffer.length = 3;
	i2c_buffer.buf = buff;
	i2c_master(I2C_TX, 0x50);
	tmrEEPROM = 0;
}


void eeprom_read_block(void *i_pointer_ram, const void *i_pointer_eeprom,
		size_t size) {
	while (tmrEEPROM < EE_TIME_WR) {
	}   // make sure EEPROM is ready
	uint16_t pointer_eeprom = (unsigned) (uint16_t*) i_pointer_eeprom;
	uint8_t buff[2];
	buff[0] = (uint8_t) (pointer_eeprom >> 8);
	buff[1] = (uint8_t) (pointer_eeprom & 0xff);
	i2c_buffer.length = 2;
	i2c_buffer.buf = buff;
	i2c_master(I2C_TX, 0x50);
	i2c_buffer.length = size;
	i2c_buffer.buf = (uint8_t*) i_pointer_ram;
	i2c_master(I2C_RX, 0x50);

}
/*--------------------------------------------------------------------------*/
#define P_SIZE 64
#define EE_ADDR_SIZE 2
void eeprom_write_page_cmp(const char* pointer_ram, uint16_t pointer_eeprom,
		size_t size) {

	uint8_t page_head;
	uint8_t page_tail;
	int8_t full_page_count;
	uint8_t head_size;
	uint8_t tail_size = 0;
	uint8_t buff[P_SIZE + EE_ADDR_SIZE];

	page_head = pointer_eeprom / P_SIZE;
	page_tail = (pointer_eeprom + size) / P_SIZE;
	head_size = pointer_eeprom % P_SIZE;
	if ((pointer_eeprom + size) % P_SIZE)
		tail_size = P_SIZE - ((pointer_eeprom + size) % P_SIZE);
	full_page_count = (page_tail - page_head);

	if (page_head == page_tail) {
		ee_waite();
		eeprom_read_block(&buff[EE_ADDR_SIZE],
				(const void*) (page_head * P_SIZE), P_SIZE);
		memcpy(&buff[head_size + EE_ADDR_SIZE], pointer_ram, size);
		buff[0] = (uint8_t) ((page_head * P_SIZE) >> 8);
		buff[1] = (uint8_t) ((page_head * P_SIZE) & 0xff);
		i2c_buffer.length = sizeof(buff);
		i2c_buffer.buf = buff;
		i2c_master(I2C_TX, 0x50);
		tmrEEPROM = 0;
		return;
	}

	if (head_size) {
		//head
		ee_waite();
		eeprom_read_block(&buff[EE_ADDR_SIZE],
				(const void*) (page_head * P_SIZE), head_size);
		memcpy(&buff[head_size + EE_ADDR_SIZE], pointer_ram,
				(P_SIZE - head_size));
		buff[0] = (uint8_t) ((page_head * P_SIZE) >> 8);
		buff[1] = (uint8_t) ((page_head * P_SIZE) & 0xff);
		i2c_buffer.length = sizeof(buff);
		i2c_buffer.buf = buff;
		i2c_master(I2C_TX, 0x50);
		tmrEEPROM = 0;
		page_head++;
		full_page_count--;
	}

	for (uint8_t i = 0; i < full_page_count; i++) {
		ee_waite();
		memcpy(&buff[EE_ADDR_SIZE], (pointer_ram + (i * P_SIZE) + head_size),
				P_SIZE);
		buff[0] = (uint8_t) (((page_head + i) * P_SIZE) >> 8);
		buff[1] = (uint8_t) (((page_head + i) * P_SIZE) & 0xff);
		i2c_buffer.length = sizeof(buff);
		i2c_buffer.buf = buff;
		i2c_master(I2C_TX, 0x50);
		tmrEEPROM = 0;
	}

	if (tail_size) {
		//head
		ee_waite();
		eeprom_read_block(&buff[EE_ADDR_SIZE + (P_SIZE - tail_size)],
				(const void*) (page_tail * (P_SIZE - tail_size)), tail_size);
		memcpy(&buff[EE_ADDR_SIZE], pointer_ram + size - tail_size, tail_size);
		buff[0] = (uint8_t) (((page_tail) * P_SIZE) >> 8);
		buff[1] = (uint8_t) (((page_tail) * P_SIZE) & 0xff);
		i2c_buffer.length = sizeof(buff);
		i2c_buffer.buf = buff;
		i2c_master(I2C_TX, 0x50);
		tmrEEPROM = 0;
	}

}
/*--------------------------------------------------------------------------*/
void eeWriteBlockCmp(const void *i_pointer_ram, uint16_t i_pointer_eeprom,
		size_t size) {
	const char* pointer_ram = (const char*) i_pointer_ram;
	uint16_t pointer_eeprom = i_pointer_eeprom;
	if (size > 1) {
		eeprom_write_page_cmp(pointer_ram, pointer_eeprom, size);
	} else {
		eeprom_write_byte_cmp(*pointer_ram, pointer_eeprom);
	}
}



uint8_t s_evt;


class Key {
#define FILTERBITS      4
#define FFVAL          ((1<<FILTERBITS)-1)
#define KSTATE_OFF      0
#define KSTATE_RPTDELAY 95 // gruvin: longer dely before key repeating starts
	//#define KSTATE_SHORT   96
#define KSTATE_START   97
#define KSTATE_PAUSE   98
#define KSTATE_KILLED  99
	uint8_t m_vals :FILTERBITS;   // key debounce?  4 = 40ms
	uint8_t unused_m_dblcnt :2;
	uint8_t m_cnt;
	uint8_t m_state;
public:
	void input(bool val, EnumKeys enuk);
	bool state() {
		return m_vals == FFVAL;
	}
#ifdef FAILSAFE
	bool isKilled() {return m_state == KSTATE_KILLED;}
#endif
	void pauseEvents() {
		m_state = KSTATE_PAUSE;
		m_cnt = 0;
	}
	void killEvents() {
		m_state = KSTATE_KILLED; /*m_dblcnt=0;*/
	}
//  uint8_t getDbl()   { return m_dblcnt;                     }
};

Key keys[NUM_KEYS];
void Key::input(bool val, EnumKeys enuk) {
	//  uint8_t old=m_vals;
	uint8_t t_vals;
//  m_vals <<= 1;  if(val) m_vals |= 1; //portbit einschieben
	t_vals = m_vals;
	t_vals <<= 1;
	if (val)
		t_vals |= 1; //portbit einschieben
	m_vals = t_vals;
	m_cnt++;

	if (m_state && m_vals == 0) {  //gerade eben sprung auf 0
		if (m_state != KSTATE_KILLED) {
			putEvent(EVT_KEY_BREAK(enuk));
//      if(!( m_state == 16 && m_cnt<16)){
//        m_dblcnt=0;
//      }
			//      }
		}
		m_cnt = 0;
		m_state = KSTATE_OFF;
	}
	switch (m_state) {
	case KSTATE_OFF:
		if (m_vals == FFVAL) { //gerade eben sprung auf ff
			m_state = KSTATE_START;
//        if(m_cnt>16) m_dblcnt=0; //pause zu lang fuer double
			m_cnt = 0;
		}
		break;
		//fallthrough
	case KSTATE_START:
		putEvent(EVT_KEY_FIRST(enuk));
		Inactivity.inacCounter = 0;
//      m_dblcnt++;
#ifdef KSTATE_RPTDELAY
		m_state = KSTATE_RPTDELAY;
#else
		m_state = 16;
#endif
		m_cnt = 0;
		break;
#ifdef KSTATE_RPTDELAY
	case KSTATE_RPTDELAY: // gruvin: longer delay before first key repeat
		if (m_cnt == 32)
			putEvent(EVT_KEY_LONG(enuk)); // need to catch this inside RPTDELAY time
		if (m_cnt == 40) {
			m_state = 16;
			m_cnt = 0;
		}
		break;
#endif
	case 16:
#ifndef KSTATE_RPTDELAY
		if(m_cnt == 32) putEvent(EVT_KEY_LONG(enuk));
		//fallthrough
#endif
	case 8:
	case 4:
	case 2:
		if (m_cnt >= 48) { //3 6 12 24 48 pulses in every 480ms
			m_state >>= 1;
			m_cnt = 0;
		}
		//fallthrough
	case 1:
		if ((m_cnt & (m_state - 1)) == 0)
			putEvent(EVT_KEY_REPT(enuk));
		break;

	case KSTATE_PAUSE: //pause
		if (m_cnt >= 64) {
			m_state = 8;
			m_cnt = 0;
		}
		break;

	case KSTATE_KILLED: //killed
		break;
	}
}

#ifdef FAILSAFE
uint8_t menuPressed()
{
	if ( keys[KEY_MENU].isKilled() )
	{
		return 0;
	}
	return ( read_keys() & 2 ) == 0;
}
#endif

// Returns 0, 1
uint8_t switchPosition(uint8_t swtch) {
  return keyState((EnumKeys)swtch) ? 0 : 1;
}


bool keyState(EnumKeys enuk) {
	uint8_t xxx = 0;
	uint8_t ping = PING();
	uint8_t pine = PINE();
	if (enuk < (int) DIM(keys))
		return keys[enuk].state() ? 1 : 0;

	switch ((uint8_t) enuk) {
	case SW_ElevDR:
		xxx = pine & (1 << INP_E_ElevDR);
		break;

	case SW_AileDR:
		xxx = pine & (1 << INP_E_AileDR);
		break;

	case SW_RuddDR:
		xxx = ping & (1 << INP_G_RuddDR);
		break;
		//     INP_G_ID1 INP_E_ID2
		// id0    0        1
		// id1    1        1
		// id2    1        0
	case SW_ID0:
		xxx = ~ping & (1 << INP_G_ID1);
		break;
	case SW_ID1:
		xxx = (ping & (1 << INP_G_ID1));
		if (xxx)
			xxx = (PINE() & (1 << INP_E_ID2));
		break;
	case SW_ID2:
		xxx = ~pine & (1 << INP_E_ID2);
		break;
	case SW_Gear:
		xxx = pine & (1 << INP_E_Gear);
		break;
		//case SW_ThrCt  : return PINE() & (1<<INP_E_ThrCt);

#if defined(CPUM128) || defined(CPUM2561)
		case SW_ThrCt :
		if ( g_eeGeneral.FrskyPins )
		{
			xxx = PINC & (1<<INP_C_ThrCt);
		}
		else
		{
			xxx = pine & (1<<INP_E_ThrCt);
		}
#else
#if (!(defined(JETI) || defined(FRSKY) || defined(ARDUPILOT) || defined(NMEA)))
	case SW_ThrCt:
		xxx = pine & (1 << INP_E_ThrCt);
#else
		case SW_ThrCt : xxx = PINC & (1<<INP_C_ThrCt); //shad974: rerouted inputs to free up UART0
#endif
#endif
		break;

	case SW_Trainer:
		xxx = pine & (1 << INP_E_Trainer);
		break;
	default:
		;
	}
	if (xxx) {
		return 1;
	}
	return 0;
}

void pauseEvents(uint8_t event) {
	event = event & EVT_KEY_MASK;
	if (event < (int) DIM(keys))
		keys[event].pauseEvents();
}
void killEvents(uint8_t event) {
	event = event & EVT_KEY_MASK;
	if (event < (int) DIM(keys))
		keys[event].killEvents();
}

//uint8_t getEventDbl(uint8_t event)
//{
//  event=event & EVT_KEY_MASK;
//  if(event < (int)DIM(keys))  return keys[event].getDbl();
//  return 0;
//}

//uint16_t g_anaIns[8];
volatile uint32_t g_tmr10ms;
//volatile uint8_t g8_tmr10ms ;
volatile uint8_t g_blinkTmr10ms;
extern uint8_t StickScrollTimer;

void per10ms() {
	uint16_t tmr;
//  g_tmr10ms++;				// 16 bit sized
//	g8_tmr10ms += 1 ;		// byte sized
//  g_blinkTmr10ms++;
	tmr = g_tmr10ms + 1;
	g_tmr10ms = tmr;
	g_blinkTmr10ms = tmr;
	uint8_t enuk = KEY_MENU;
	uint8_t in = ~PINB();

	static uint8_t current;
	uint8_t dir_keys;
	uint8_t lcurrent;

	dir_keys = in & 0x78;		// Mask to direction keys
	if ((lcurrent = current)) { // Something already pressed
		if ((lcurrent & dir_keys) == 0) {
			lcurrent = 0;	// No longer pressed
		} else {
			in &= lcurrent | 0x06;	// current or MENU or EXIT allowed
		}
	}
	if (lcurrent == 0) { // look for a key
		if (dir_keys & 0x20)	// right
				{
			lcurrent = 0x60;		// Allow L and R for 9X
		} else if (dir_keys & 0x40)	// left
				{
			lcurrent = 0x60;		// Allow L and R for 9X
		} else if (dir_keys & 0x08)	// down
				{
			lcurrent = 0x08;
		} else if (dir_keys & 0x10)	// up
				{
			lcurrent = 0x10;
		}
		in &= lcurrent | 0x06;	// current or MENU or EXIT allowed
	}
	current = lcurrent;

	for (uint8_t i = 1; i < 7; i++) {
		//INP_B_KEY_MEN 1  .. INP_B_KEY_LFT 6
		keys[enuk].input(in & 2, (EnumKeys) enuk);
		++enuk;
		in >>= 1;
	}

	const static unsigned char crossTrim[] = { 1 << INP_D_TRM_LH_DWN, 1
			<< INP_D_TRM_LH_UP, 1 << INP_D_TRM_LV_DWN, 1 << INP_D_TRM_LV_UP, 1
			<< INP_D_TRM_RV_DWN, 1 << INP_D_TRM_RV_UP, 1 << INP_D_TRM_RH_DWN, 1
			<< INP_D_TRM_RH_UP };


        in = ~PIND();

	for (int i = 0; i < 8; i++) {
		// INP_D_TRM_RH_UP   0 .. INP_D_TRM_LH_UP   7
		keys[enuk].input(in & pgm_read_byte(crossTrim + i), (EnumKeys) enuk);
		++enuk;
	}

	uint8_t value = Rotary.RotEncoder & 0x20;
	keys[enuk].input(value, (EnumKeys) enuk); // Rotary Enc. Switch

	in = ~PINB() & 0x7E;
	value |= in;
	if (value) {
		StickScrollTimer = STICK_SCROLL_TIMEOUT;
	}


}

#ifndef SIMU

//void serialVoiceInit() {
//	/*
//	 #undef BAUD
//	 #define BAUD 38400
//	 //#include <util/setbaud.h>

//	 //	DDRD |= 0x08 ;
//	 PORTD |= 0x04 ;		// Pullup on RXD1

//	 UBRR1H = UBRRH_VALUE;
//	 UBRR1L = UBRRL_VALUE;
//	 UCSR1A &= ~(1 << U2X1); // disable double speed operation.

//	 // set 8 N1
//	 UCSR1C = 0 | (1 << UCSZ11) | (1 << UCSZ10);

//	 while (UCSR1A & (1 << RXC1)) UDR1; // flush receive buffer

//	 UCSR1B = (1 << RXCIE1) | (0 << TXCIE1) | (0 << UDRIE1) | (1 << RXEN1) | (1 << TXEN1) | (0 << UCSZ12) ;
//	 //  UCSR1B |= (1 << TXEN1) | (1 << RXEN1) ; // enable TX & Rx
//	 //	UCSR1B |= (1 << RXCIE1); // enable Interrupt
//	 */
//}

//void startSerialVoice() {
//      //PausePulses = 1;
//   //   Backup_RestoreRunning = 1;
//      serialVoiceInit();
//}

//uint16_t SerialVoiceDebug ;

//void ISR_USART1_RX_vect(void) {
//	/*	UCSR1B &= ~(1 << RXCIE1); // disable Interrupt
//	 sei() ;
//	 //SerialVoiceDebug += 1 ;
//	 struct t_fifo16 *pfifo = &SvFifo ;
//	 uint8_t next = (pfifo->in + 1) & 0x0f ;
//	 uint8_t data = UDR1 ;
//	 if ( next != pfifo->out )
//	 {
//	 pfifo->fifo[pfifo->in] = data ;
//	 pfifo->in = next ;
//	 }
//	 cli() ;
//	 UCSR1B |= (1 << RXCIE1); // enable Interrupt*/
//}

#endif
