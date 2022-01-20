/*
 * er9x.cpp
 *
 *  Created on: 10 авг. 2019 г.
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
#include <stdlib.h>
#include "language.h"
#include "pulses.h"
#include "lcd.h"
#include "menus.h"
#include "voice.h"
#include "iface_a7105.h"
#include "Vario.h"
#include "crossfire/crossfire.h"
// Next two lines swapped as new complier/linker reverses them in memory!
const
//#ifdef REMOVE_FROM_64FRSKY
//#include "s9xsplashs.lbm"
//const prog_uchar APM s9xsplashMarker[] = {
//"Spls"
//};
//#else
//#include "s9xsplash.lbm"
//#include "splashmarker.h"
//#endif

#if defined(CPUM128) || defined(CPUM2561)
const uint8_t
#include "..\..\common\hand.lbm"
#endif

/*
mode1 rud ele thr ail
mode2 rud thr ele ail
mode3 ail ele thr rud
mode4 ail thr ele rud
*/

// Various debug defines
//#define	SERIALVOICE	1

// #define BLIGHT_DEBUG 1

#define ROTARY	1

//uint8_t RebootReason ;

extern int16_t AltOffset ;

#ifdef V2
uint8_t Last_switch[NUM_CSW+EXTRA_CSW] ;
#else
#if defined(CPUM128) || defined(CPUM2561)
uint8_t Last_switch[NUM_CSW+EXTRA_CSW] ;
#else
uint8_t Last_switch[NUM_CSW] ;
#endif
#endif // V2

static void checkMem( void );
void checkTHR( void );
///   Pr�ft beim Einschalten ob alle Switches 'off' sind.
void checkSwitches( void );

#ifdef V2
#ifdef USE_ADJUSTERS
static void	processAdjusters( void ) ;
#endif
#endif
int8_t getGvarSourceValue( uint8_t src ) ;

#ifndef SIMU
static void checkQuickSelect( void ); // Quick model select on startup
void getADC_osmp( void ) ;
#endif

#ifdef V2
V2EEGeneral g_eeGeneral;
V2ModelData g_model ;
#else
EEGeneral  g_eeGeneral;
ModelData g_model ;
#endif

extern uint8_t scroll_disabled ;
const char *AlertMessage ;
uint8_t Main_running ;
uint8_t SlaveMode ;
#ifdef V2
uint8_t Vs_state[EXTRA_VOICE_SW] ;
#else
#ifdef NOVOICE_SW
//uint8_t Vs_state[NUM_CHNOUT] ;
#else
uint8_t Vs_state[NUM_CHNOUT+EXTRA_VOICE_SW] ;
#endif
#endif
uint8_t Nvs_state[NUM_VOICE_ALARMS] ;
int16_t Nvs_timer[NUM_VOICE_ALARMS] ;
uint8_t CurrentVolume ;

uint8_t ppmInAvailable = 0 ;

struct t_rotary Rotary ;

uint8_t Tevent ;


#ifndef V2
TimerMode TimerConfig[2] ;
#endif // nV2

const uint8_t bchout_ar[] = {0x1B,
    0x1E,
    0x27,
    0x2D,
    0x36,
    0x39,
    0x4B,
    0x4E,
    0x63,
    0x6C,
    0x72,
    0x78,
    0x87,
    0x8D,
    0x93,
    0x9C,
    0xB1,
    0xB4,
    0xC6,
    0xC9,
    0xD2,
    0xD8,
    0xE1,
    0xE4};

//new audio object
audioQueue  audio;

uint8_t sysFlags = 0;
uint8_t SystemOptions ;


struct t_alarmControl AlarmControl = { 100, 0, 10, 2 } ;

#ifdef V2
int16_t  CsTimer[NUM_CSW+EXTRA_CSW] ;
#else
#if defined(CPUM128) || defined(CPUM2561)
int16_t  CsTimer[NUM_CSW+EXTRA_CSW] ;
#else
int16_t  CsTimer[NUM_CSW] ;
#endif
#endif // V2

const char  Str_Alert[] = STR_ALERT ;
const char  Str_Switches[] = SWITCHES_STR ;
const char Str_OFF[] =  STR_OFF ;
const char Str_ON[] = STR_ON ;

#if defined(CPUM128) || defined(CPUM2561)
const char  modi12x3[]= "\005" STR_STICK_NAMES ;
#else
const char modi12x3[]= {"\004" STR_STICK_NAMES} ;
#endif
const uint8_t stickScramble[]= {
    0, 1, 2, 3,
    0, 2, 1, 3,
    3, 1, 2, 0,
    3, 2, 1, 0 };

#ifdef CPUM2561
uint8_t Arduino = 0 ;
#endif

const char Str_Hyphens[] = "----" ;

uint8_t modeFixValue( uint8_t value )
{
	return *(stickScramble+g_eeGeneral.stickMode*4+value)+1 ;
}

const uint8_t csTypeTable[] =
#ifdef V2
{ CS_VOFS, CS_VOFS, CS_VOFS, CS_VOFS, CS_VBOOL, CS_VBOOL, CS_VBOOL,
 CS_VCOMP, CS_VCOMP, CS_VCOMP, CS_VCOMP, CS_VBOOL, CS_VBOOL, CS_TIMER, CS_VOFS, CS_TMONO, CS_TMONO
} ;
#else
#ifdef VERSION3
#if defined(CPUM128) || defined(CPUM2561)
{ CS_VOFS, CS_VOFS, CS_VOFS, CS_VOFS, CS_VBOOL, CS_VBOOL, CS_VBOOL,
 CS_VCOMP, CS_VCOMP, CS_VCOMP, CS_VCOMP, CS_VBOOL, CS_VBOOL, CS_TIMER, CS_VOFS, CS_TMONO, CS_TMONO
} ;
#else
{ CS_VOFS, CS_VOFS, CS_VOFS, CS_VOFS, CS_VBOOL, CS_VBOOL, CS_VBOOL,
 CS_VCOMP, CS_VCOMP, CS_VCOMP, CS_VCOMP, CS_VBOOL, CS_VBOOL, CS_TIMER, CS_VOFS
} ;
#endif
#else
{ CS_VOFS, CS_VOFS, CS_VOFS, CS_VOFS, CS_VBOOL, CS_VBOOL, CS_VBOOL,
 CS_VCOMP, CS_VCOMP, CS_VCOMP, CS_VCOMP, CS_VCOMP, CS_VCOMP, CS_TIMER, CS_VOFS
} ;
#endif
#endif // V2

uint32_t get_tmr10ms()
{

    return g_tmr10ms ;
}

uint8_t CS_STATE( uint8_t x)
{
	return *(csTypeTable+x-1) ;

}

MixData *mixaddress( uint8_t idx )
{
    return &g_model.mixData[idx] ;
}

//LimitData *limitaddress( uint8_t idx )
//{
//    return &g_model.limitData[idx];
//}

uint8_t throttleReversed()
{
	return g_model.throttleReversed ^	g_eeGeneral.throttleReversed ;
}

void putsChnRaw(uint8_t x,uint8_t y,uint8_t idx,uint8_t att)
{
	uint8_t chanLimit = NUM_XCHNRAW ;
	uint8_t mix = att & MIX_SOURCE ;
	if ( mix )
	{
		chanLimit += MAX_GVARS + 1 + 1 ;
		att &= ~MIX_SOURCE ;
	}
    if(idx==0)
        lcd_putsnAtt(x,y,Str_Hyphens,4,att);
    else if(idx<=4)
		{
			if ( g_model.useCustomStickNames )
			{
#ifdef V2
				lcd_putsnAtt( x, y, ( char *)(( g_model.useCustomStickNames == 2 ) ? g_model.customStickNames : g_eeGeneral.customStickNames)+4*(idx-1), 4, att|BSS ) ;
#else
				lcd_putsnAtt( x, y, ( char *)g_eeGeneral.customStickNames+4*(idx-1), 4, att|BSS ) ;
#endif
			}
			else
			{
        lcd_putsAttIdx(x,y,modi12x3,(idx-1),att) ;
			}
		}
    else if(idx<=chanLimit)
        lcd_putsAttIdx(x,y,Str_Chans_Gv,(idx-5),att);

#ifdef TELEMETRY  // kotello remove rem 
    else
		{
			if ( mix )
			{
				idx += TEL_ITEM_SC1-(chanLimit-NUM_XCHNRAW) ;
			}
  	  lcd_putsAttIdx(x,y,Str_telemItems,(idx-NUM_XCHNRAW),att);
		}
#endif
}

void putsChn(uint8_t x,uint8_t y,uint8_t idx1,uint8_t att)
{
#if defined(CPUM128) || defined(CPUM2561)
	if ( idx1 == 0 )
	{
    lcd_putsnAtt(x,y,PSTR("--- "),4,att);
	}
	else
	{
		uint8_t x1 ;
		x1 = x + 4*FW-2 ;
		if ( idx1 < 10 )
		{
			x1 -= FWNUM ;
		}
		lcd_2_digits( x1, y, idx1, att ) ;
    lcd_putsnAtt(x,y,PSTR(STR_CH),2,att);
	}
#else
	putsChnRaw( x, y, idx1 ? idx1+20 : idx1, att ) ;
#endif
}

#ifdef SWITCH_MAPPING
uint8_t switchMapTable[41] ;
uint8_t switchUnMapTable[HSW_MAX+1] ;
uint8_t MaxSwitchIndex ;		// For ON and OFF

//uint8_t numSwitchpositions( uint8_t swtch )
//{
//	uint8_t positions = ( swtch == SW_ID0 ) ? 3 : 2 ;
//	uint8_t map = g_eeGeneral.switchMapping ;
//	if ( swtch == SW_ElevDR )
//	{
//		if ( map & USE_ELE_3POS )
//		{
//			positions = 3 ;
//		}
//	}
//	return positions ;
//}


void createSwitchMapping()
{
	uint8_t *p = switchMapTable ;
	FORCE_INDIRECT(p) ;
	uint8_t map = g_eeGeneral.switchMapping ;
	*p++ = 0 ;

#ifdef XSW_MOD
	if ( map & USE_THR_3POS )
	{
		*p++ = HSW_Thr3pos0 ;
		*p++ = HSW_Thr3pos1 ;
		*p++ = HSW_Thr3pos2 ;
	}
	else
	{
		*p++ = HSW_ThrCt ;
	}
#else
	*p++ = HSW_ThrCt ;
#endif
	if ( map & USE_RUD_3POS )
	{
		*p++ = HSW_Rud3pos0 ;
		*p++ = HSW_Rud3pos1 ;
		*p++ = HSW_Rud3pos2 ;
	}
	else
	{
		*p++ = HSW_RuddDR ;
	}
	if ( map & USE_ELE_3POS )
	{
		*p++ = HSW_Ele3pos0 ;
		*p++ = HSW_Ele3pos1 ;
		*p++ = HSW_Ele3pos2 ;
	}
	else
	{
		*p++ = HSW_ElevDR ;
	}
	*p++ = HSW_ID0 ;
	*p++ = HSW_ID1 ;
	*p++ = HSW_ID2 ;
	if ( map & USE_AIL_3POS )
	{
		*p++ = HSW_Ail3pos0 ;
		*p++ = HSW_Ail3pos1 ;
		*p++ = HSW_Ail3pos2 ;
	}
	else
	{
		*p++ = HSW_AileDR ;
	}
	if ( map & USE_GEA_3POS )
	{
		*p++ = HSW_Gear3pos0 ;
		*p++ = HSW_Gear3pos1 ;
		*p++ = HSW_Gear3pos2 ;
	}
	else
	{
		*p++ = HSW_Gear ;
	}
	*p++ = HSW_Trainer ;
	if ( map & USE_PB1 )
	{
		*p++ = HSW_Pb1 ;
	}
	if ( map & USE_PB2 )
	{
		*p++ = HSW_Pb2 ;
	}
#if defined(CPUM128) || defined(CPUM2561)
	for ( uint8_t i = 10 ; i <=27 ; i += 1  )
#else
	for ( uint8_t i = 10 ; i <=21 ; i += 1  )
#endif
	{
		*p++ = i ;	// Custom switches
	}
	*p = MAX_DRSWITCH ;
	MaxSwitchIndex = p - switchMapTable ;
	*++p = MAX_DRSWITCH+1 ;
	for ( uint8_t i = 0 ; i <= (uint8_t)MaxSwitchIndex+1 ; i += 1  )
	{
		switchUnMapTable[switchMapTable[i]] = i ;
	}
//	uint8_t index = 1 ;
//	Sw3PosList[0] = HSW_ID0 ;
//	Sw3PosCount[0] = 3 ;
//	Sw3PosCount[index] = 2 ;
//	Sw3PosList[index] = HSW_ThrCt ;
//	Sw3PosCount[++index] = 2 ;
//	Sw3PosList[index] = HSW_RuddDR ;
//	Sw3PosCount[++index] = 2 ;
//	Sw3PosList[index] = HSW_ElevDR ;
//	if ( map & USE_ELE_3POS )
//	{
//		Sw3PosCount[index] = 3 ;
//		Sw3PosList[index] = HSW_Ele3pos0 ;
//	}
//	Sw3PosCount[++index] = 2 ;
//	Sw3PosList[index] = HSW_AileDR ;
//	Sw3PosCount[++index] = 2 ;
//	Sw3PosList[index] = HSW_Gear ;
//	Sw3PosCount[++index] = 2 ;
//	Sw3PosList[index] = HSW_Trainer ;

	if ( g_eeGeneral.pg2Input )
	{
/*
    DDRG &= ~0x04 ;
		PORTG |= 0x04 ; // pullups
*/
	}
	if ( g_eeGeneral.pb7Input )
	{
/*
    DDRB &= ~(1<<OUT_B_LIGHT) ;
		PORTB |= (1<<OUT_B_LIGHT) ; // pullups
*/
	}
}

int8_t switchUnMap( int8_t x )
{
	uint8_t sign = 0 ;
	if ( x < 0 )
	{
		sign = 1 ;
		x = -x ;
	}
	x = switchUnMapTable[x] ;
	if ( sign )
	{
		x = -x ;
	}
	return x ;
}

int8_t switchMap( int8_t x )
{
	uint8_t sign = 0 ;
	if ( x < 0 )
	{
		sign = 1 ;
		x = -x ;
	}
	x = switchMapTable[x] ;
	if ( sign )
	{
		x = -x ;
	}
	return x ;
}


#endif

void putsMomentDrSwitches(uint8_t x,uint8_t y,int8_t idx1,uint8_t att)
{
	if ( idx1 > TOGGLE_INDEX )
	{
		lcd_putcAtt(x+3*FW,  y,'m',att);
		idx1 -= TOGGLE_INDEX  ;
	}
  putsDrSwitches( x-1*FW, y, idx1, att ) ;
}

void putsDrSwitches(uint8_t x,uint8_t y,int8_t idx1,uint8_t att)//, bool nc)
{

		switch(idx1){
    case  0:            lcd_putsAtt(x+FW,y,&Str_Hyphens[1],att);return;
    case  MAX_DRSWITCH: lcd_putsAtt(x+FW,y,Str_ON,att);return;
    case -MAX_DRSWITCH: lcd_putsAtt(x+FW,y,Str_OFF,att);return;
    }
//		if ( idx1 < 0 )
//		{
//		}
		int8_t z ;
		z = idx1 ;
		if ( z < 0 )
		{
  		lcd_putcAtt(x,y, '!',att);
			z = -idx1 ;
		}
		z -= 1 ;
#ifdef XSW_MOD
  if (z >= (SW_3POS_BASE-1)) {
    // to get 3pos switch names from Str_Switches
    z -= (SW_3POS_BASE-1-(MAX_PSWITCH+MAX_CSWITCH));
  }
#else
//		z *= 3 ;
	if ( z > MAX_DRSWITCH )
	{
		z -= HSW_OFFSET ;
	}
#endif
  lcd_putsAttIdx(x+FW,y,Str_Switches,z,att) ;
}

void putsTmrMode(uint8_t x, uint8_t y, uint8_t attr, uint8_t type )
{ // Valid values of type are 0, 1 or 2 only

#ifndef V2
	TimerMode *ptConfig ;
#else
	V2TimerMode *ptConfig ;
#endif
  int8_t tm ;
  int8_t tmb ;
#ifndef V2
	if ( type & 0x80 )
	{
		ptConfig = &TimerConfig[1] ;
	}
	else
	{
		ptConfig = &TimerConfig[0] ;
	}
#else
	if ( type & 0x80 )
	{
		ptConfig = &g_model.timer[1] ;
	}
	else
	{
		ptConfig = &g_model.timer[0] ;
	}
#endif
	tm = ptConfig->tmrModeA ;
  tmb = ptConfig->tmrModeB ;

	type &= 3 ;
	if ( type < 2 )		// 0 or 1
	{
	  if(tm<TMR_VAROFS)
		{
			lcd_putsnAtt(  x, y,(STR_TRIGA_OPTS)+3*tm,3,attr);
  	}
		else
		{
  		tm -= TMR_VAROFS - 7 ;
      lcd_putsAttIdx(  x, y, Curve_Str, tm, attr ) ;
			if ( tm < 9 + 7 )	// Allow for 7 offset above
			{
				x -= FW ;
			}
  		lcd_putcAtt(x+3*FW,  y,'%',attr);
		}
	}
	if ( ( type == 2 ) || ( ( type == 0 ) && ( tm == 1 ) ) )
	{
		putsMomentDrSwitches( x, y, tmb, attr );
	}
	asm("") ;
}

#ifdef TELEMETRY

uint8_t Unit ;

//uint16_t scale_telem_value( uint16_t val, uint8_t channel, uint8_t *p_att )
//{
//  uint32_t value ;
//	uint16_t ratio ;
//#ifdef V2
//	V2FrSkyChannelData *fd ;
//#else
//	FrSkyChannelData *fd ;
//#endif
//  uint8_t unit = 'v' ;
//	uint8_t places = 1 ;

//	//TODO fd = &g_model.frsky.channels[channel] ;
//  value = val ;

//#ifdef V2
//	ratio = fd->ratio ;
//	if ( ratio < 100 )
//	{
//		places = 2 ;
//		value *= 10 ;
//	}
//	value *= ratio ;
//	value /= 256 ;
//	unit = fd->unit ;
//#else
//	ratio = fd->opt.alarm.ratio ;
//  if (fd->opt.alarm.type == 2/*V*/)
//  {
//      ratio <<= 1 ;
//  }
//  value *= ratio ;
//	if ( fd->opt.alarm.type == 3/*A*/)
//  {
//		unit = 'A' ;
//    value /= 100 ;
//  }
//  else if ( ( ratio < 100 ) && ( fd->opt.alarm.type != 1 ) )	// Not raw
//  {
//      value *= 2 ;
//      value /= 51 ;  // Same as *10 /255 but without overflow
//      places = 2 ;
//  }
//  else
//  {
//      value /= 255 ;
//  }
//	if ( fd->opt.alarm.type == 1 )
//	{
//		unit = ' ' ;
//  	places = 0 ;
//	}
//#endif
//	if ( p_att )
//	{
//  	*p_att = places ;
//	}
//	Unit = unit ;
//	return value ;
//}

uint8_t putsTelemValue(uint8_t x, uint8_t y, int32_t val, uint8_t channel, uint8_t att, uint8_t dig) {

if (AFHDS2A_tel_status & ((uint64_t)1<< channel)){ 

  switch (AFHDS2A_tel[channel].prec) {
  case 1:
    att |= PREC1;
    break;
  case 2:
    att |= PREC2;
    break;
  }
  lcd_outdezNAtt(x, y, val, att, dig);
} else{ 
if( !(att & LEFT)){  
if(att & DBLSIZE)
x -= 6 * FW;
else
x -= 3 * FW;
} else x+=FW;
lcd_putsAtt(x, y, "---", att);
}
  return 0;
}

#endif

#ifdef XSW_MOD
inline uint8_t switchPosition(uint8_t swtch)
{
		uint8_t pi = (swtch - 1) ;
    if (swtch >= SW_3POS_BASE) {
			//assert(swtch <= SW_3POS_END);
			swtch -= SW_3POS_BASE;
      pi = swtch / 3;
	  }
    return switchState(PSW_BASE + pi);
}
#endif

int16_t getValue(uint8_t i)
{
    if(i<7) return calibratedStick[i];//-512..512
    if(i<PPM_BASE) return 0 ;
		else if(i<CHOUT_BASE)
		{
			int16_t x ;
			x = g_ppmIns[i-PPM_BASE] ;
			if(i<PPM_BASE+4)
			{
				x -= g_eeGeneral.trainer.calib[i-PPM_BASE] ;
			}
			return x*2;
		}
		else if(i<CHOUT_BASE+NUM_CHNOUT) return Ex_chans[i-CHOUT_BASE];
    else if(i<CHOUT_BASE+NUM_CHNOUT+NUM_TELEM_ITEMS)
		{
			return get_telemetry_value( i-CHOUT_BASE-NUM_CHNOUT ) ;
		}
    return 0;
}

bool getSwitch00( int8_t swtch )
{
	return getSwitch( swtch, 0, 0 ) ;
}

bool getSwitch(int8_t swtch, bool nc, uint8_t level)
{
    bool ret_value ;
    uint8_t cs_index ;

    switch(swtch){
    case  0:            return  nc;
    case  MAX_DRSWITCH: return  true;
    case -MAX_DRSWITCH: return  false;
    }

#ifdef XSW_MOD
    bool dir = (swtch > 0);
    uint8_t aswtch = swtch ;
    if ( swtch < 0 )
      aswtch = -swtch ;

    if (aswtch <= MAX_PSWITCH) {
      aswtch = switchState(PSW_BASE + aswtch - 1) ;
      return (dir ? (aswtch != ST_UP) : (aswtch == ST_UP)) ;
    }

    if (aswtch >= SW_3POS_BASE) {
      if (aswtch > SW_3POS_END)
        return false;
      aswtch -= SW_3POS_BASE;
      uint8_t pi = aswtch / 3;
      aswtch -= pi * 3;
      ret_value = (switchState(PSW_BASE + pi) == aswtch);
      return (dir ? ret_value : !ret_value);
    }

    //custom switch, Issue 78
    //use putsChnRaw
    //input -> 1..4 -> sticks,  5..8 pots
    //MAX,FULL - disregard
    //ppm
    cs_index = aswtch - MAX_PSWITCH - 1;

#else // !XSW_MOD

#ifndef SWITCH_MAPPING
		if ( swtch > MAX_DRSWITCH )
		{
			return false ;
		}
#endif

#ifdef SWITCH_MAPPING
	if ( abs(swtch) > MAX_DRSWITCH )
	{
		uint8_t value = hwKeyState( abs(swtch) ) ;
		if ( swtch > 0 )
		{
			return value ;
		}
		else
		{
			return ! value ;
		}
	}
#endif

    uint8_t dir = swtch>0;
    uint8_t aswtch = swtch ;
		if ( swtch < 0 )
		{
			aswtch = -swtch ;
		}

#ifdef V2
    if(aswtch<(MAX_DRSWITCH-NUM_CSW-EXTRA_CSW))
#else
 #if defined(CPUM128) || defined(CPUM2561)
    if(aswtch<(MAX_DRSWITCH-NUM_CSW-EXTRA_CSW))
 #else
    if(aswtch<(MAX_DRSWITCH-NUM_CSW))
 #endif
#endif
		{
			aswtch = keyState((EnumKeys)(SW_BASE+aswtch-1)) ;
			return !dir ? (!aswtch) : aswtch ;
    }

    //custom switch, Issue 78
    //use putsChnRaw
    //input -> 1..4 -> sticks,  5..8 pots
    //MAX,FULL - disregard
    //ppm

#endif  // XSW_MOD

#ifdef V2
#ifndef XSW_MOD
    cs_index = aswtch-(MAX_DRSWITCH-NUM_CSW-EXTRA_CSW) ;
#endif
		CxSwData *cs = &g_model.customSw[cs_index];

		if(!cs->func) return false;

    if ( level>4 )
    {
    		ret_value = Last_switch[cs_index] & 1 ;
        return dir ? ret_value : !ret_value ;
    }

    int8_t a = cs->v1;
    int8_t b = cs->v2;
    int16_t x = 0;
    int16_t y = 0;
		uint8_t valid = 1 ;

    // init values only if needed
    uint8_t s = CS_STATE(cs->func);

    if(s == CS_VOFS)
    {
        x = getValue(cs->v1-1);
#ifdef FRSKY
        if (cs->v1 > CHOUT_BASE+NUM_CHNOUT)
				{
					uint8_t idx = cs->v1-CHOUT_BASE-NUM_CHNOUT-1 ;
          y = convertTelemConstant( idx, cs->v2 ) ;
					valid = telemItemValid( idx ) ;
				}
        else
#endif
            y = calc100toRESX(cs->v2);
    }
    else if(s == CS_VCOMP)
    {
        x = getValue(cs->v1-1);
        y = getValue(cs->v2-1);
    }

    switch ((uint8_t)cs->func) {
    case (CS_VPOS):
        ret_value = (x>y);
        break;
    case (CS_VNEG):
        ret_value = (x<y) ;
        break;
    case (CS_APOS):
    {
        ret_value = (abs(x)>y) ;
    }
    break;
    case (CS_ANEG):
    {
        ret_value = (abs(x)<y) ;
    }
    break;
		case CS_EXEQUAL:
			if ( isAgvar( cs->v1 ) )
			{
				x *= 10 ;
				y *= 10 ;
			}
  		ret_value = abs(x-y) < 32 ;
  	break;

    case (CS_AND):
    case (CS_OR):
    case (CS_XOR):
    {
        bool res1 = getSwitch(a,0,level+1) ;
        bool res2 = getSwitch(b,0,level+1) ;
        if ( cs->func == CS_AND )
        {
            ret_value = res1 && res2 ;
        }
        else if ( cs->func == CS_OR )
        {
            ret_value = res1 || res2 ;
        }
        else  // CS_XOR
        {
            ret_value = res1 ^ res2 ;
        }
    }
    break;

    case (CS_EQUAL):
        ret_value = (x==y);
        break;
    case (CS_NEQUAL):
        ret_value = (x!=y);
        break;
    case (CS_GREATER):
        ret_value = (x>y);
        break;
    case (CS_LESS):
        ret_value = (x<y);
        break;
#ifndef VERSION3
    case (CS_EGREATER):
        ret_value = (x>=y);
        break;
    case (CS_ELESS):
        ret_value = (x<=y);
        break;
#endif
    case (CS_TIME):
        ret_value = CsTimer[cs_index] >= 0 ;
        break;
#ifdef VERSION3
		case (CS_LATCH) :
  	case (CS_FLIP) :
    	ret_value = Last_switch[cs_index] & 1 ;
	  break ;
#endif
    default:
        ret_value = false;
        break;
    }
		if ( valid == 0 )			// Catch telemetry values not present
		{
      ret_value = false;
		}
		if ( ret_value )
		{
			int8_t x ;
			x = cs->andsw ;
			if ( x )
			{
      	ret_value = getSwitch( x, 0, level+1) ;
			}
		}
#ifdef VERSION3
		if ( cs->func < CS_LATCH )
		{
#endif
			Last_switch[cs_index] = ret_value ;
#ifdef VERSION3
		}
#endif
//#ifdef XSW_MOD
//    return dir ? ret_value : !ret_value ;
//#else
    return swtch>0 ? ret_value : !ret_value ;
//#endif




#else // !V2

#ifndef XSW_MOD
 #if defined(CPUM128) || defined(CPUM2561)
    cs_index = aswtch-(MAX_DRSWITCH-NUM_CSW-EXTRA_CSW);
 #else
    cs_index = aswtch-(MAX_DRSWITCH-NUM_CSW);
 #endif
#endif

#if defined(CPUM128) || defined(CPUM2561)
		if ( cs_index >= NUM_CSW )
		{
			CxSwData *cs = &g_model.xcustomSw[cs_index-NUM_CSW];
			if(!cs->func) return false;

    	if ( level>4 )
    	{
			    ret_value = Last_switch[cs_index] & 1 ;
//#ifdef XSW_MOD
//			    return dir ? ret_value : !ret_value ;
//#else
			    return swtch>0 ? ret_value : !ret_value ;
//#endif
    	}

    	int8_t a = cs->v1;
    	int8_t b = cs->v2;
    	int16_t x = 0;
    	int16_t y = 0;
			uint8_t valid = 1 ;

    	// init values only if needed
    	uint8_t s = CS_STATE(cs->func);

    	if(s == CS_VOFS)
    	{
    	    x = getValue(cs->v1-1);
	#ifdef FRSKY
    	    if (cs->v1 > CHOUT_BASE+NUM_CHNOUT)
					{
						uint8_t idx = cs->v1-CHOUT_BASE-NUM_CHNOUT-1 ;
    	      y = convertTelemConstant( idx, cs->v2 ) ;
						valid = telemItemValid( idx ) ;
					}
    	    else
	#endif
    	        y = calc100toRESX(cs->v2);
    	}
    	else if(s == CS_VCOMP)
    	{
    	    x = getValue(cs->v1-1);
    	    y = getValue(cs->v2-1);
    	}

    	switch ((uint8_t)cs->func) {
    	case (CS_VPOS):
    	    ret_value = (x>y);
    	    break;
    	case (CS_VNEG):
    	    ret_value = (x<y) ;
    	    break;
    	case (CS_APOS):
    	{
    	    ret_value = (abs(x)>y) ;
    	}
    	break;
    	case (CS_ANEG):
    	{
    	    ret_value = (abs(x)<y) ;
    	}
    	break;
			case CS_EXEQUAL:
				if ( isAgvar( cs->v1 ) )
				{
					x *= 10 ;
					y *= 10 ;
				}
  		  ret_value = abs(x-y) < 32 ;
  		break;

    	case (CS_AND):
    	case (CS_OR):
    	case (CS_XOR):
    	{
    	    bool res1 = getSwitch(a,0,level+1) ;
    	    bool res2 = getSwitch(b,0,level+1) ;
    	    if ( cs->func == CS_AND )
    	    {
    	        ret_value = res1 && res2 ;
    	    }
    	    else if ( cs->func == CS_OR )
    	    {
    	        ret_value = res1 || res2 ;
    	    }
    	    else  // CS_XOR
    	    {
    	        ret_value = res1 ^ res2 ;
    	    }
    	}
    	break;

    	case (CS_EQUAL):
    	    ret_value = (x==y);
    	    break;
    	case (CS_NEQUAL):
    	    ret_value = (x!=y);
    	    break;
    	case (CS_GREATER):
    	    ret_value = (x>y);
    	    break;
    	case (CS_LESS):
    	    ret_value = (x<y);
    	    break;
#ifndef VERSION3
    	case (CS_EGREATER):
    	    ret_value = (x>=y);
    	    break;
    	case (CS_ELESS):
    	    ret_value = (x<=y);
    	    break;
#endif
    	case (CS_TIME):
    	    ret_value = CsTimer[cs_index] >= 0 ;
    	    break;
#ifdef VERSION3
			case (CS_MONO):
		  case (CS_RMONO):
		    ret_value = CsTimer[cs_index] > 0 ;
		  break ;
		  case (CS_LATCH) :
  		case (CS_FLIP) :
  		  ret_value = Last_switch[cs_index] & 1 ;
  		break ;
#endif
    	default:
    	    ret_value = false;
    	    break;
    	}
			if ( valid == 0 )			// Catch telemetry values not present
			{
    	  ret_value = false;
			}
			if ( ret_value )
			{
				int8_t x ;
				x = cs->andsw ;
				if ( x )
				{
#ifdef XSW_MOD
					if ( ( x >= DSW_TRN ) && ( x <= DSW_TRN+NUM_CSW+EXTRA_CSW ) )
					{
						x += 1 ;
					}
					if ( ( x <= -DSW_TRN ) && ( x >= -(DSW_TRN+NUM_CSW+EXTRA_CSW) ) )
					{
						x -= 1 ;
					}
					if ( x == DSW_TRN+NUM_CSW+EXTRA_CSW+1 )
					{
						x = DSW_TRN ;			// Tag TRN on the end, keep EEPROM values
					}
					if ( x == -(DSW_TRN+NUM_CSW+EXTRA_CSW+1) )
					{
						x = -DSW_TRN ;			// Tag TRN on the end, keep EEPROM values
					}
#else // !XSW_MOD
					if ( ( x > 8 ) && ( x <= 9+NUM_CSW+EXTRA_CSW ) )
					{
						x += 1 ;
					}
					if ( ( x < -8 ) && ( x >= -(9+NUM_CSW+EXTRA_CSW) ) )
					{
						x -= 1 ;
					}
					if ( x == 9+NUM_CSW+EXTRA_CSW+1 )
					{
						x = 9 ;			// Tag TRN on the end, keep EEPROM values
					}
					if ( x == -(9+NUM_CSW+EXTRA_CSW+1) )
					{
						x = -9 ;			// Tag TRN on the end, keep EEPROM values
					}
#endif  // XSW_MOD
    	  	ret_value = getSwitch( x, 0, level+1) ;
				}
			}
#ifdef VERSION3
			if ( cs->func < CS_LATCH )
			{
#endif
				Last_switch[cs_index] = ret_value ;
#ifdef VERSION3
			}
#endif
//#ifdef XSW_MOD
//	    return dir ? ret_value : !ret_value ;
//#else
	    return swtch>0 ? ret_value : !ret_value ;
//#endif
		}
#endif
		CSwData *cs = &g_model.customSw[cs_index];

		if(!cs->func) return false;

    if ( level>4 )
    {
    		ret_value = Last_switch[cs_index] & 1 ;
//#ifdef XSW_MOD
//		    return dir ? ret_value : !ret_value ;
//#else
    		return swtch>0 ? ret_value : !ret_value ;
//#endif
    }

    int8_t a = cs->v1;
    int8_t b = cs->v2;
    int16_t x = 0;
    int16_t y = 0;
		uint8_t valid = 1 ;

    // init values only if needed
    uint8_t s = CS_STATE(cs->func);

    if(s == CS_VOFS)
    {
        x = getValue(cs->v1-1);
#ifdef FRSKY
        if (cs->v1 > CHOUT_BASE+NUM_CHNOUT)
				{
					uint8_t idx = cs->v1-CHOUT_BASE-NUM_CHNOUT-1 ;
          y = convertTelemConstant( idx, cs->v2 ) ;
					valid = telemItemValid( idx ) ;
				}
        else
#endif
            y = calc100toRESX(cs->v2);
    }
    else if(s == CS_VCOMP)
    {
        x = getValue(cs->v1-1);
        y = getValue(cs->v2-1);
    }

    switch ((uint8_t)cs->func) {
    case (CS_VPOS):
        ret_value = (x>y);
        break;
    case (CS_VNEG):
        ret_value = (x<y) ;
        break;
    case (CS_APOS):
    {
        ret_value = (abs(x)>y) ;
    }
    break;
    case (CS_ANEG):
    {
        ret_value = (abs(x)<y) ;
    }
    break;

    case (CS_AND):
    case (CS_OR):
    case (CS_XOR):
    {
        bool res1 = getSwitch(a,0,level+1) ;
        bool res2 = getSwitch(b,0,level+1) ;
        if ( cs->func == CS_AND )
        {
            ret_value = res1 && res2 ;
        }
        else if ( cs->func == CS_OR )
        {
            ret_value = res1 || res2 ;
        }
        else  // CS_XOR
        {
            ret_value = res1 ^ res2 ;
        }
    }
    break;

    case (CS_EQUAL):
        ret_value = (x==y);
        break;
    case (CS_NEQUAL):
        ret_value = (x!=y);
        break;
    case (CS_GREATER):
        ret_value = (x>y);
        break;
    case (CS_LESS):
        ret_value = (x<y);
        break;
#ifndef VERSION3
    case (CS_EGREATER):
        ret_value = (x>=y);
        break;
    case (CS_ELESS):
        ret_value = (x<=y);
        break;
#endif
    case (CS_TIME):
        ret_value = CsTimer[cs_index] >= 0 ;
        break;
#ifdef VERSION3
		case (CS_LATCH) :
  	case (CS_FLIP) :
    	ret_value = Last_switch[cs_index] & 1 ;
	  break ;
#endif
    default:
        ret_value = false;
        break;
    }
		if ( valid == 0 )			// Catch telemetry values not present
		{
      ret_value = false;
		}
		if ( ret_value )
		{
			int8_t x ;
			x = cs->andsw ;
			if ( x )
			{
#ifdef XSW_MOD
				if ( x >= DSW_TRN )
#else
				if ( x > 8 )
#endif
				{
					x += 1 ;
				}
      	ret_value = getSwitch( x, 0, level+1) ;
			}
		}
#ifdef VERSION3
		if ( cs->func < CS_LATCH )
		{
#endif
			Last_switch[cs_index] = ret_value ;
#ifdef VERSION3
		}
#endif
//#ifdef XSW_MOD
//    return dir ? ret_value : !ret_value ;
//#else
    return swtch>0 ? ret_value : !ret_value ;
//#endif
#endif // V2

}


//#define CS_EQUAL     8
//#define CS_NEQUAL    9
//#define CS_GREATER   10
//#define CS_LESS      11
//#define CS_EGREATER  12
//#define CS_ELESS     13

inline uint8_t keyDown()
{
    return (~PINB()) & 0x7E;
}

static void clearKeyEvents()
{
#ifdef SIMU
    while (keyDown() && main_thread_running) sleep(1/*ms*/);
#else
    while (keyDown())
		{
			wdt_reset() ; // loop until all keys are up
		}
#endif
    putEvent(0);
}

void check_backlight_voice()
{
	static uint8_t tmr10ms ;
    if(getSwitch00(g_eeGeneral.lightSw) || g_LightOffCounter)
        BACKLIGHT_ON ;
    else
        BACKLIGHT_OFF ;

	uint8_t x ;
	x = g_blinkTmr10ms ;
	if ( tmr10ms != x )
	{
		tmr10ms = x ;
		Voice.voice_process() ;
	}
}

uint16_t stickMoveValue()
{
#define INAC_DEVISOR 256   // Issue 206 - bypass splash screen with stick movement
    uint16_t sum = 0;
    for(uint8_t i=0; i<4; i++)
        sum += anaIn(i) ;
	sum += 128 ;
	return sum / INAC_DEVISOR ;
}


const uint8_t DoubleBits[] = {
	0x00, 0x03, 0x0C, 0x0F,
	0x30, 0x33, 0x3C, 0x3F,
	0xC0, 0xC3, 0xCC, 0xCF,
	0xF0, 0xF3, 0xFC, 0xFF } ;

static void doSplash()
{
    {

#ifdef SIMU
	    if (!main_thread_running) return;
  	  sleep(1/*ms*/);
#endif

        check_backlight_voice() ;

        lcd_clear();

#ifndef REMOVE_FROM_64FRSKY
//        lcd_img(32, 16, s9xsplash, 0 ) ;
				{
				  const prog_uchar  *q ;

  				for(uint8_t yb = 0 ; yb < 8 ; yb += 1 )
					{
extern uint8_t DisplayBuf[DISPLAY_W*DISPLAY_H/8] ;

						uint8_t   *p = &DisplayBuf[ (yb) * DISPLAY_W ] ;
						q = &s9xsplash[(yb/2) * (DISPLAY_W/2)] ;
  				  for(uint8_t x=0 ; x < DISPLAY_W/2 ; x++ )
						{
  				    uint8_t b = pgm_read_byte(q++);
							if ( yb & 1 )
							{
								b >>= 4 ;
							}
							b = pgm_read_byte(&DoubleBits[b & 0x0F] ) ;
  				    *p++ = b ;
  				    *p++ = b ;
  				  }
  				}
				}
#else
        //text splash screen
        /*---------------------------Kotello--------------------------------*/
        //lcd_img(0, 0, s9xsplash,0);
        lcd_putsnAtt(4, 20, "--ERFly6--", 10, DBLSIZE);
        lcd_putsnAtt(3 * FW, 6 * FH, "PILOT-", 6, BSS);
        /*------------------------------------------------------------------*/

#endif
                                if (!g_eeGeneral.hideNameOnSplash)
                                  lcd_putsnAtt(9 * FW, 6 * FH, g_eeGeneral.ownerName,
                                      sizeof(g_eeGeneral.ownerName), BSS);

                                refreshDiplay();

                                clearKeyEvents();

#ifndef SIMU
        getADC_osmp();
#endif
        uint16_t inacSum = stickMoveValue();

        uint16_t tgtime = get_tmr10ms() + SPLASH_TIMEOUT;
        do
				{
        	refreshDiplay();
          check_backlight_voice() ;
#ifdef SIMU
            if (!main_thread_running) return;
            sleep(1/*ms*/);
#else
            getADC_osmp();
#endif
            uint16_t tsum = stickMoveValue();

            if(keyDown() || (tsum!=inacSum))   return;  //wait for key release

        } while(tgtime != get_tmr10ms()) ;
    }
}

static void checkMem()
{
    if(g_eeGeneral.disableMemoryWarning) return;
    if(EeFsGetFree() < 200)
    {
        alert((STR_EE_LOW_MEM));
    }

}

#define	ALERT_TYPE	0
#define MESS_TYPE	1
#define	ALERT_SKIP	2
#define	ALERT_VOICE	4

void almess( const  char * s, uint8_t type )
{
	const char *h ;
  lcd_clear();
  lcd_puts_Pleft(4*FW,s);

	if ( type & ALERT_VOICE)
	{
		audioVoiceDefevent(AU_ERROR, V_ALERT);
	}

	if ( ( type & 1 ) == ALERT_TYPE)
	{
		if ( type & 2 )
		{
    	lcd_puts_Pleft(6*FH,(STR_PRESS_KEY_SKIP) ) ;
		}
		else
		{
    	lcd_puts_P(64-6*FW,7*FH,(STR_PRESS_ANY_KEY));
		}
		h = Str_Alert ;
	}
	else
	{
		h = (STR_MESSAGE) ;
	}
  lcd_putsAtt(64-7*FW,0*FH, h,DBLSIZE);

//	if ( ( type & 2 ) == 0 )
//	{
//		refreshDiplay() ;
//	}
//	asm("") ;
}


void message(const char * s)
{
	almess( s, MESS_TYPE ) ;
	refreshDiplay() ;
}

void alert(const char * s)
{
	alertx( s, false ) ;
}

void alertx(const char * s, bool defaults)
{
	if ( Main_running )
	{
		AlertMessage = s ;
		return ;
	}
	almess( s, ALERT_TYPE | ALERT_VOICE ) ;
	refreshDiplay() ;

	lcdSetRefVolt(defaults ? LCD_NOMCONTRAST : g_eeGeneral.contrast);
//  audioVoiceDefevent(AU_ERROR, V_ALERT);

    clearKeyEvents();
    while(1)
    {
#ifdef SIMU
    if (!main_thread_running) return;
    sleep(1/*ms*/);
#endif
        if(keyDown())
        {
				    clearKeyEvents() ;
            return;  //wait for key release
        }
        if(heartbeat == 0x3)
        {
            wdt_reset();
            heartbeat = 0;
        }

        if(defaults)
        	BACKLIGHT_ON ;
		    else
    	    BACKLIGHT_OFF ;
        check_backlight_voice() ;
    }

}


//static int16_t tanaIn( uint8_t chan )
//{
// 	int16_t v = anaIn(chan) ;
//	return v ;
////	return  (g_eeGeneral.throttleReversed) ? -v : v ;
//}


uint8_t checkThrottlePosition()
{
  uint8_t thrchn=(2-(g_eeGeneral.stickMode&1));//stickMode=0123 -> thr=2121
	int16_t v = scaleAnalog( anaIn(thrchn), thrchn ) ;

//	if ( g_model.throttleIdle == 2 )
//	{
//  	if(v >= RESX - THRCHK_DEADBAND  )
//  	{
//  		return 1 ;
//  	}
//	}
//	else
	if ( g_model.throttleIdle )
	{
		if ( abs( v ) < THRCHK_DEADBAND )
		{
			return 1 ;
		}
	}
	else
	{
  	if(v <= -RESX + THRCHK_DEADBAND )
  	{
  		return 1 ;
  	}
	}
	return 0 ;
}

void checkTHR()
{
    if(g_eeGeneral.disableThrottleWarning) return;

//    uint8_t thrchn=(2-(g_eeGeneral.stickMode&1));//stickMode=0123 -> thr=2121

#ifndef SIMU
		getADC_osmp();   // if thr is down - do not display warning at all
#endif

//		int16_t lowLim = g_eeGeneral.calibMid[thrchn] ;

//		lowLim = (g_eeGeneral.throttleReversed ? (- lowLim) - g_eeGeneral.calibSpanPos[thrchn] : lowLim - g_eeGeneral.calibSpanNeg[thrchn]);
//		lowLim += THRCHK_DEADBAND ;

    if ( checkThrottlePosition() )
		{
			return ;
		}

    // first - display warning

#if defined(CPUM128) || defined(CPUM2561)
	lcd_clear();
  lcd_img( 1, 0, HandImage,0 ) ;
  lcd_putsAtt(32,0*FH,PSTR("THROTTLE"),DBLSIZE);
  lcd_putsAtt(32,2*FH,PSTR("WARNING"),DBLSIZE);
	lcd_puts_P(0,5*FH,  PSTR(STR_THR_NOT_IDLE) ) ;
	lcd_puts_P(0,6*FH,  PSTR(STR_RST_THROTTLE) ) ;
	lcd_puts_P(0,7*FH,  PSTR(STR_PRESS_KEY_SKIP) ) ;
	audioVoiceDefevent(AU_ERROR, V_ALERT);
	putVoiceQueue( V_THR_WARN ) ;
#else
		almess((STR_THR_NOT_IDLE "\037" STR_RST_THROTTLE), ALERT_SKIP | ALERT_VOICE ) ;
#endif
    refreshDiplay() ;
    clearKeyEvents();

    //loop until all switches are reset
    while (1)
    {
#ifdef SIMU
      if (!main_thread_running) return;
      sleep(1/*ms*/);
#else
        getADC_osmp();
#endif
        check_backlight_voice() ;

        wdt_reset() ;

		    if ( checkThrottlePosition() )
				{
					return ;
				}
				if( keyDown() )
        {
			    clearKeyEvents() ;
          return ;
        }
    }
}

#ifndef MINIMISE_CODE
static void checkAlarm() // added by Gohst
{
    if(g_eeGeneral.disableAlarmWarning) return;
    if(!g_eeGeneral.beeperVal) alert((STR_ALARMS_DISABLE));
}
#endif

static void checkWarnings()
{
    if(sysFlags)
    {
        alert((STR_OLD_VER_EEPROM)); //will update on next save
        sysFlags = 0 ; //clear flag
    }
}

void putWarnSwitch( uint8_t x, uint8_t idx )
{
#if defined(CPUM128) || defined(CPUM2561)
  lcd_putsAttIdx( x, 5*FH, Str_Switches, idx, 0) ;
#else
  lcd_putsAttIdx( x, 2*FH, Str_Switches, idx, 0) ;
#endif
}

#ifdef XSW_MOD
uint16_t getCurrentSwitchStates()
{
  uint16_t i = 0, j = PSW_END;
  while (j > PSW_3POS_END) {
    i <<= 1;
    if (switchState( j-- ) == ST_DN)   // ST_UP or ST_DN or ST_NC for PB1/PB2
      i |= 1;
  }
  while (j >= PSW_BASE) {
    uint8_t t = switchState( j-- ) ;
    i <<= 2 ;
    i |= t ;
  }
  return i;   // (MSB=0,Trainer:1,PB2:1,PB1:1,Gear:2,AileDR:2,ElevDR:2,RuddDR:2,ThrCt:2,IDL:2)
}

void checkSwitches()
{
  if(g_eeGeneral.disableSwitchWarning)
    return; // if warning is off

  uint16_t warningStates = g_model.switchWarningStates ;

  uint8_t first = 1 ;
  uint8_t voice = 0 ;

  //loop until all switches are reset
  while (true) {
    uint16_t i = getCurrentSwitchStates() ;
    //show the difference between i and switch?
    //show just the offending switches.
    //first row - IDL, THR, RUD, ELE, AIL, GEA, PB1, PB2, TRN
    uint16_t x = i ^ warningStates ;

#if defined(CPUM128) || defined(CPUM2561)
				lcd_clear();
    		lcd_img( 1, 0, HandImage,0 ) ;
	  		lcd_putsAtt(32,0*FH,PSTR("Switch"),DBLSIZE);
	  		lcd_putsAtt(32,2*FH,PSTR("Warning"),DBLSIZE);
				lcd_puts_P(0,7*FH,  PSTR(STR_PRESS_KEY_SKIP) ) ;
				if ( voice )
				{
					audioVoiceDefevent(AU_ERROR, V_ALERT);
					putVoiceQueue( V_SW_WARN ) ;
				}
#else
    almess( (STR_SWITCH_WARN+"\037"+STR_RESET_SWITCHES), ALERT_SKIP | voice ) ;
#endif
    voice = 0 ;

    uint16_t m = 3;
    uint8_t k = 0, w = 2, dw = 3*FW + FW/2;
    uint8_t j = PSW_BASE;
    while (j++ <= PSW_3POS_END) {
      if (x & m) {
        putWarnSwitch(w, k);
      }
      m <<= 2;
      w += dw;
      k++;
    }
    m = 1 << (k << 1);
    while (j++ <= PSW_END) {
      if (x & m) {
        putWarnSwitch(w, k);
      }
      m <<= 1;
      w += dw;
      k++;
    }

    refreshDiplay();

    if ( first )
    {
      voice = ALERT_VOICE ;
      clearKeyEvents();
      first = 0 ;
    }

    if( (i==warningStates) || (keyDown())) // check state against settings
    {
      return;  //wait for key release
    }

    check_backlight_voice() ;
    wdt_reset() ;

  }
}

//uint16_t CurrentSwitches ;
#if defined(CPUM128) || defined(CPUM2561)
static uint16_t switches_state = 0;

int8_t getMovedSwitch()
{
  static uint16_t s_last_time = 0 ;

  uint16_t xstate = switches_state;
  switches_state = getCurrentSwitchStates();
//	CurrentSwitches = switches_state ;

  uint16_t time = get_tmr10ms() ;
  bool skipping = ( (uint16_t)(time - s_last_time) > 10) ;
  s_last_time = time ;
  if (skipping)
    return 0;

  //         DSW_TRN,  DSW_PB2, _PB1, _GEA, _AIL,  DSW_ELE, DSW_RUD, DSW_THR,DSW_IDL
  // sw_index   (x,9,        8,    7,    6,     5,       4,       3,       2,      1    )
  // sw_state = (0,Trainer:1,PB2:1,PB1:1,Gear:2,AileDR:2,ElevDR:2,RuddDR:2,ThrCt:2,IDL:2)
  int8_t swi = 0, swi3 = SW_3POS_BASE, result = 0 ;
  uint16_t cstate = switches_state;
  xstate ^= cstate;
  while (swi < MAX_PSW3POS)
	{
		swi += 1 ;
    if (xstate & 3)           // 3pos switch moved
		{
      if (is3PosSwitch(swi))  // 3pos pin connected/mapped?
			{
        uint8_t s3 = (cstate & 3) ;
        result = swi3 + s3 ;
      }
			else
			{
        result = (cstate & 2) ? swi : (-swi) ;
      }
      break;
    }
    xstate >>= 2;
    cstate >>= 2;
    swi3 += 3;
  }
  if (result == 0)
	{
    while (swi++ < MAX_PSWITCH)
		{
      if (xstate & 1)       // 2pos switch moved
			{
        result = ((cstate & 1) ? swi : (-swi)) ;
        break;
      }
      xstate >>= 1 ;
      cstate >>= 1 ;
    }
  }
  return result ;
}
#endif

#else	// !XSW_MOD

uint8_t getCurrentSwitchStates()
{
  uint8_t i = 0 ;
  for( uint8_t j=0; j<8; j++ )
  {
    bool t=keyState( (EnumKeys)(SW_BASE_DIAG+7-j) ) ;
		i <<= 1 ;
    i |= t ;
  }
#ifdef SWITCH_MAPPING
		if ( g_eeGeneral.switchMapping & USE_ELE_3POS )
		{
			i &= 0xC3 ;
			uint8_t k = HSW_Ele3pos0 ;
			i |= switchPosition(k) << 2 ;
			k = HSW_ID0 ;
			i |= switchPosition(k) << 4 ;
		}
//		if ( g_eeGeneral.switchMapping & USE_AIL_3POS )
//		{
//			i &= 0xC3 ;
//			uint8_t k = HSW_Ail3pos0 ;
//			i |= switchPosition(k) << 2 ;
//			k = HSW_ID0 ;
//			i |= switchPosition(k) << 4 ;
//		}
#endif
	return i ;
}

void checkSwitches()
{
	uint8_t warningStates ;
#ifdef SWITCH_MAPPING
	uint8_t exWarningStates	;
#endif

	if(g_eeGeneral.disableSwitchWarning) return; // if warning is on

	warningStates = g_model.switchWarningStates ;
#ifdef SWITCH_MAPPING
	exWarningStates = g_model.exSwitchWarningStates ;

	if ( ( g_eeGeneral.switchMapping & USE_ELE_3POS ) == 0 )
	{
#endif
    uint8_t x = warningStates & SWP_IL5;
    if(!(x==SWP_LEG1 || x==SWP_LEG2 || x==SWP_LEG3)) //legal states for ID0/1/2
    {
        warningStates &= ~SWP_IL5; // turn all off, make sure only one is on
        warningStates |=  SWP_ID0B;
				g_model.switchWarningStates = warningStates ;
    }
#ifdef SWITCH_MAPPING
	}
#endif

	uint8_t first = 1 ;
	uint8_t voice = 0 ;

    //loop until all switches are reset
    while (1)
    {
	getADC_osmp();   
        uint8_t i = getCurrentSwitchStates() ;
#ifdef SWITCH_MAPPING
        uint8_t j = switchPosition(HSW_Ail3pos0) ;
#endif

        //show the difference between i and switch?
        //show just the offending switches.
        //first row - THR, GEA, AIL, ELE, ID0/1/2
        uint8_t x = i ^ warningStates ;
#ifdef SWITCH_MAPPING
        uint8_t y = j ^ exWarningStates ;
#endif

#if defined(CPUM128) || defined(CPUM2561)
				lcd_clear();
    		lcd_img( 1, 0, HandImage,0 ) ;
	  		lcd_putsAtt(32,0*FH,PSTR("Switch"),DBLSIZE);
	  		lcd_putsAtt(32,2*FH,PSTR("Warning"),DBLSIZE);
				lcd_puts_P(0,7*FH,  PSTR(STR_PRESS_KEY_SKIP) ) ;
				if ( voice )
				{
					audioVoiceDefevent(AU_ERROR, V_ALERT);
					putVoiceQueue( V_SW_WARN ) ;
				}
#else
		    almess((STR_SWITCH_WARN "\037" STR_RESET_SWITCHES), ALERT_SKIP | voice ) ;
#endif
				voice = 0 ;

        if(x & SWP_THRB)
            putWarnSwitch(2 + 0*FW, 0 );
        if(x & SWP_RUDB)
            putWarnSwitch(2 + 3*FW + FW/2, 1 );

#ifdef SWITCH_MAPPING
			if ( g_eeGeneral.switchMapping & USE_ELE_3POS )
			{
				if ( x & 0x0C )
				{
          putWarnSwitch(2 + 7*FW, ( (i>>2) & 3 ) + ( HSW_Ele3pos0 - HSW_OFFSET - 1 ) ) ;
				}
        if( x & 0x30 )
				{
          putWarnSwitch(2 + 10*FW + FW/2, ( (i>>4) & 3 ) + 3 );
				}
			}
			else
			{
				if(x & SWP_ELEB)
            putWarnSwitch(2 + 7*FW, 2 );

        if(x & SWP_IL5)
        {
            if(i & SWP_ID0B)
                putWarnSwitch(2 + 10*FW + FW/2, 3 );
            else if(i & SWP_ID1B)
                putWarnSwitch(2 + 10*FW + FW/2, 4 );
            else if(i & SWP_ID2B)
                putWarnSwitch(2 + 10*FW + FW/2, 5 );
        }
			}
#else
				if(x & SWP_ELEB)
            putWarnSwitch(2 + 7*FW, 2 );

        if(x & SWP_IL5)
        {
            if(i & SWP_ID0B)
                putWarnSwitch(2 + 10*FW + FW/2, 3 );
            else if(i & SWP_ID1B)
                putWarnSwitch(2 + 10*FW + FW/2, 4 );
            else if(i & SWP_ID2B)
                putWarnSwitch(2 + 10*FW + FW/2, 5 );
        }
#endif

#ifdef SWITCH_MAPPING
			if ( g_eeGeneral.switchMapping & USE_AIL_3POS )
			{
				if ( y & 0x03 )
				{
          putWarnSwitch(2 + 14*FW, ( j & 3 ) + ( HSW_Ail3pos0 - HSW_OFFSET - 1 ) ) ;
				}
			}
			else
			{
				j = exWarningStates ;
        if(x & SWP_AILB)
            putWarnSwitch(2 + 14*FW, 6 );
			}
#else
        if(x & SWP_AILB)
            putWarnSwitch(2 + 14*FW, 6 );
#endif
        if(x & SWP_GEAB)
            putWarnSwitch(2 + 17*FW + FW/2, 7 );


//#if SERIALVOICE
//				k += 1 ;
//				if ( k > 2500 )
//				{
//					k = 0 ;
//					if ( p == 0 )
//					{
//						p = 1 ;
//			  	  UDR1 = 'A' ;
//					}
//					else
//					{
//						p = 0 ;
//			  	  UDR1 = 'B' ;
//					}
//					UCSR1A = ( 1 << TXC1 ) ;		// CLEAR flag
//					while ( ( UCSR1A & ( 1 << TXC1 ) ) == 0 )
//					{
//						// wait
//					}
//#undef BAUD
//#define BAUD 19200
//				  UBRR1L = UBRRL_VALUE;
//				}
//				if (UCSR1A & (1 << RXC1))
//				{
//					q = UDR1 ;
//#undef BAUD
//#define BAUD 38400
//				  UBRR1L = UBRRL_VALUE;
//				}
//				lcd_putc( 0, 8, q ) ;

//#endif
        refreshDiplay();

				if ( first )
				{
					voice = ALERT_VOICE ;
    			clearKeyEvents();
					first = 0 ;
				}

#ifdef SWITCH_MAPPING
        if( ( (i==warningStates) && (j == exWarningStates) ) || (keyDown())) // check state against settings
#else
        if( (i==warningStates) || (keyDown())) // check state against settings
#endif
        {
//#if SERIALVOICE
//  UCSR1B &= ~(1 << TXEN1) ; // disable TX pin
//  UCSR1B &= ~(1 << RXEN1) ; // disable RX
//#endif
            return;  //wait for key release
        }

        check_backlight_voice() ;
        wdt_reset() ;

    }


}

#if defined(CPUM128) || defined(CPUM2561)

#ifdef SWITCH_MAPPING
static uint16_t switches_states = 0 ;
#else
static uint8_t switches_states = 0 ;
#endif
static uint8_t trainer_state = 0 ;

// Can we save flash by using :
// uint8_t getCurrentSwitchStates()
int8_t getMovedSwitch()
{
	uint8_t skipping = 0 ;
  int8_t result = 0 ;

  static uint16_t s_last_time = 0 ;

	uint16_t time = get_tmr10ms() ;
  if ( (uint16_t)(time - s_last_time) > 10)
	{
		skipping = 1 ;
		switches_states = 0 ;
	}
  s_last_time = time ;

#ifdef SWITCH_MAPPING
  uint16_t mask = 0xC000 ;
	uint8_t map = g_eeGeneral.switchMapping ;
  for (uint8_t i=MAX_PSWITCH-1; i>0; i--)
	{
    uint8_t prev = (switches_states & mask) >> (i*2-2) ;
  	uint8_t next = getSwitch00(i) ;
		uint8_t swtchIndex = i ;

		switch ( i )
		{
			case 7 :// Ail
				if ( map & USE_AIL_3POS )
				{
					next = switchPosition( HSW_Ail3pos0 ) ;
					swtchIndex = HSW_Ail3pos0 + next ;
				}
			break ;

			case 3 :// ELE
				if ( map & USE_ELE_3POS )
				{
					next = switchPosition( HSW_Ele3pos0 ) ;
					swtchIndex = HSW_Ele3pos0 + next ;
				}
			break ;
		}
    if (prev != next)
		{
     	switches_states = (switches_states & (~mask)) | (next << (i*2-2));
			if ( swtchIndex > 8 )
			{
				result = swtchIndex ;
			}
			else
			{
				if ( ( i>=4 ) && (i<=6) )
				{
					if ( next )
					{
						result = i ;
					}
				}
				else
				{
        	result = next ? i : -i ;
				}
			}
		}
		mask >>= 2 ;
	}
#else
  uint8_t mask = 0x80 ;
  for (uint8_t i=MAX_PSWITCH-1; i>0; i--)
	{
  	bool next = getSwitch00(i) ;

		if ( skipping )
		{
			if ( next )
			{
				switches_states |= mask ;
			}
		}
		else
		{
			uint8_t value = next ? mask : 0 ;
			if ( ( switches_states ^ value ) & mask )
			{ // State changed
				switches_states ^= mask ;
        result = next ? i : -i ;
				if ( ( result <= -4 ) && ( result >= -6 ) )
				{
					result = 0 ;
				}
				break ;
			}
		}
		mask >>= 1 ;
  }
#endif

#ifdef SWITCH_MAPPING
	if ( result == 0 )
	{
		mask = getSwitch00( 9 ) ;
		if ( ( mask ^ trainer_state ) & 1 )
		{
			if ( mask )
			{
				result = 9 ;
			}
			trainer_state ^= 1 ;
		}
	}
	if ( map & USE_PB1 )
	{
		if ( result == 0 )
		{
			mask = getSwitch00( HSW_Pb1 ) << 1 ;
			if ( ( mask ^ trainer_state ) & 2 )
			{
				if ( mask )
				{
					result = HSW_Pb1 ;
				}
				trainer_state ^= 2 ;
			}
		}
	}
	if ( map & USE_PB2 )
	{
		if ( result == 0 )
		{
			mask = getSwitch00( HSW_Pb2 ) << 2 ;
			if ( ( mask ^ trainer_state ) & 4 )
			{
				if ( mask )
				{
					result = HSW_Pb2 ;
				}
				trainer_state ^= 4 ;
			}
		}
	}
#else
	if ( result == 0 )
	{
		mask = getSwitch00( 9 ) ;
		if ( mask && ( trainer_state == 0 ) )
		{
			result = 9 ;
		}
		trainer_state = mask ;
	}
#endif

  if ( skipping )
    result = 0 ;

	return result;
}
#endif
#endif  // XSW_MOD

#ifndef SIMU
static void checkQuickSelect()
{
    uint8_t i = keyDown(); //check for keystate
#ifdef QUICK_SELECT
    uint8_t j;
#endif

		if ( ( i & 6 ) == 6 )
		{
			SystemOptions |= SYS_OPT_MUTE ;
			return ;
		}

#ifdef QUICK_SELECT

    for(j=0; j<6; j++)
		{
			if ( i & 0x02 ) break ;
			i >>= 1 ;
		}


    if(j<6) {
        if(!eeModelExists(j)) return;

        eeLoadModel(g_eeGeneral.currModel = j);
        STORE_GENERALVARS;
        //        eeDirty(EE_GENERAL);

        lcd_clear();
        lcd_putsAtt(64-7*FW,0*FH,(STR_LOADING),DBLSIZE);

				putsDblSizeName( 3*FH ) ;

        refreshDiplay();
        clearKeyEvents(); // wait for user to release key
    }
#endif // QUICK_SELECT
}
#endif

uint8_t StickScrollAllowed ;
uint8_t StickScrollTimer ;

MenuFuncP g_menuStack[5];

uint8_t  g_menuStackPtr = 0;
uint8_t  EnterMenu = 0 ;


#ifdef FMODE_TRIM
int8_t *TrimPtr[4] =
{
    &g_model.trim[0],
    &g_model.trim[1],
    &g_model.trim[2],
    &g_model.trim[3]
} ;
#endif

uint8_t getFlightPhase()
{
	uint8_t i ;
#ifdef V2
	V2PhaseData *phase = &g_model.phaseData[0];
#else
	PhaseData *phase = &g_model.phaseData[0];
#endif

  for ( i = 0 ; i < MAX_MODES ; i += 1 )
	{
    if ( phase->swtch && getSwitch00( phase->swtch ) )
		{
      return i + 1 ;
    }
		phase += 1 ;
  }
  return 0 ;
}

// Using this will save 14 bytes flash
//NOINLINE int16_t *phaseTrimAddress( uint8_t phase, uint8_t index )
//{
//	return &g_model.phaseData[phase-1].trim[index] ;
//}


int16_t getRawTrimValue( uint8_t phase, uint8_t idx )
{
	if ( phase )
	{
		return g_model.phaseData[phase-1].trim[idx] + TRIM_EXTENDED_MAX + 1 ;
	}
	else
	{
#ifdef FMODE_TRIM
		return *TrimPtr[idx] ;
#else
		return g_model.trim[idx] ;
#endif
	}
}

uint8_t getTrimFlightPhase( uint8_t phase, uint8_t idx )
{
  for ( uint8_t i=0 ; i<MAX_MODES ; i += 1 )
	{
    if (phase == 0) return 0;
    int16_t trim = getRawTrimValue( phase, idx ) ;
    if ( trim <= TRIM_EXTENDED_MAX )
		{
			return phase ;
		}
    uint8_t result = trim-TRIM_EXTENDED_MAX-1 ;
    if (result >= phase)
		{
			result += 1 ;
		}
    phase = result;
  }
  return 0;
}


int16_t getTrimValue( uint8_t phase, uint8_t idx )
{
  return getRawTrimValue( getTrimFlightPhase( phase, idx ), idx ) ;
}

int16_t validatePlusMinus125( int16_t trim )
{
  if(trim > 125)
	{
		return 125 ;
	}
  if(trim < -125 )
	{
		return -125 ;
	}
	return trim ;
}

void setTrimValue(uint8_t phase, uint8_t idx, int16_t trim)
{
	if ( phase )
	{
		phase = getTrimFlightPhase( phase, idx ) ;
	}
	trim = validatePlusMinus125( trim ) ;
	if ( phase )
	{
//		trim = validatePlusMinus125( trim ) ;
//    if(trim < -125 || trim > 125)
//    if(trim < -500 || trim > 500)
//		{
//			trim = ( trim > 0 ) ? 125 : -125 ;
//			trim = ( trim > 0 ) ? 500 : -500 ; For later addition
//		}
  	g_model.phaseData[phase-1].trim[idx] = trim - ( TRIM_EXTENDED_MAX + 1 ) ;
	}
	else
	{
//		trim = validatePlusMinus125( trim ) ;
//    if(trim < -125 || trim > 125)
//		{
//			trim = ( trim > 0 ) ? 125 : -125 ;
//		}
#ifdef FMODE_TRIM
   	*TrimPtr[idx] = trim ;
#else
		g_model.trim[idx] = trim ;
#endif
	}
  STORE_MODELVARS_TRIM ;
}


//uint8_t GvarSource[4] ;

static uint8_t checkTrim(uint8_t event)
{
    int8_t  k = (event & EVT_KEY_MASK) - TRM_BASE;
    int8_t  s = g_model.trimInc;
//    if (s>1) s = 1 << (s-1);  // 1=>1  2=>2  3=>4  4=>8
    if ( s == 4 )
    {
            s = 8 ;			  // 1=>1  2=>2  3=>4  4=>8
    }
    else
    {
            if ( s == 3 )
            {
                    s = 4 ;			  // 1=>1  2=>2  3=>4  4=>8
            }
    }

    if( (k>=0) && (k<8) && !IS_KEY_BREAK(event)) // && (event & _MSK_KEY_REPT))
    {
        //LH_DWN LH_UP LV_DWN LV_UP RV_DWN RV_UP RH_DWN RH_UP
        uint8_t idx = (uint8_t)k/2;

// SORT idx for stickmode
        idx = modeFixValue( idx ) - 1 ;
        if ( g_eeGeneral.crosstrim )
        {
                idx = 3 - idx ;
        }
        uint8_t phaseNo = getTrimFlightPhase( CurrentPhase, idx ) ;
        int16_t tm = getTrimValue( phaseNo, idx ) ;
        int8_t  v = (s==0) ? (abs(tm)/4)+1 : s;
        bool thrChan = (2 == idx) ;

        bool thro = false ;
        if ( thrChan )
        {
                if ( g_model.thrTrim )
                {
                        thro = true ;
                        v = 2 ; // if throttle trim and trim trottle then step=2
                }
                if(throttleReversed())
                {
                        v = -v;  // throttle reversed = trim reversed
                }
        }

//				bool thro = (thrChan && (g_model.thrTrim));
//        if(thro) v = 2 ; // if throttle trim and trim trottle then step=2

//				if ( GvarSource[idx] )
//				{
//					v = 1 ;
//				}

//        if(thrChan && throttleReversed()) v = -v;  // throttle reversed = trim reversed
        int16_t x = (k&1) ? tm + v : tm - v;   // positive = k&1

        if(((x==0)  ||  ((x>=0) != (tm>=0))) && (!thro) && (tm!=0)){
						setTrimValue( phaseNo, idx, 0 ) ;
            killEvents(event);
            audioDefevent(AU_TRIM_MIDDLE);

        } else if(x>-125 && x<125){
              setTrimValue( phaseNo, idx, x ) ;
//if(event & _MSK_KEY_REPT) warble = true;
//            if(x <= 125 && x >= -125){

              int8_t t = x ;
              if ( t < 0 )
              {
                      t = -t ;
              }
              t /= 4 ;
              audioEvent(AU_TRIM_MOVE,t+60) ;
//            }
        }
        else
        {
            setTrimValue( phaseNo, idx, (x>0) ? 125 : -125 ) ;
            if(x <= 125 && x >= -125)
            {
              int8_t t = x ;
              if ( t > 0 )
              {
                      t = -t ;
              }
              t /= 4 ;
              audioEvent(AU_TRIM_MOVE,(t+60));
            }
        }

        return 0;
    }
    return event;
}

//int8_t MovedSwitch ;

//global helper vars
//bool    checkIncDec_Ret;
#ifndef NOPOTSCROLL
struct t_p1 P1values ;
#endif
static uint8_t LongMenuTimer ;
uint8_t StepSize ;

int16_t checkIncDec16( int16_t val, int16_t i_min, int16_t i_max, uint8_t i_flags)
{
    int16_t newval = val;
    uint8_t kpl=KEY_RIGHT, kmi=KEY_LEFT, kother = -1;

    uint8_t event = Tevent ;

    if(event==EVT_KEY_FIRST(kpl) || event== EVT_KEY_REPT(kpl) || (s_editMode && (event==EVT_KEY_FIRST(KEY_UP) || event== EVT_KEY_REPT(KEY_UP))) )
    {
        if ( ( read_keys() & 2 ) == 0 )
        {
            newval += StepSize ;
        }
        else
        {
            newval += 1 ;
        }

        audioDefevent(AU_KEYPAD_UP);

        kother=kmi;
    }
    else if(event==EVT_KEY_FIRST(kmi) || event== EVT_KEY_REPT(kmi) || (s_editMode && (event==EVT_KEY_FIRST(KEY_DOWN) || event== EVT_KEY_REPT(KEY_DOWN))) )
    {
        if ( ( read_keys() & 2 ) == 0 )
        {
             newval -= StepSize ;
        }
        else
        {
              newval -= 1 ;
        }

        audioDefevent(AU_KEYPAD_DOWN);

        kother=kpl;
    }
    if((kother != (uint8_t)-1) && keyState((EnumKeys)kother)){
        newval=-val;
        killEvents(kmi);
        killEvents(kpl);
    }
    if(i_min==0 && i_max==1)
    {
        if (event==EVT_KEY_FIRST(KEY_MENU) || event==EVT_KEY_BREAK(BTN_RE))
        {
            s_editMode = false;
            newval=!val;
            killEvents(event);
            if ( event==EVT_KEY_BREAK(BTN_RE) )
            {
                    RotaryState = ROTARY_MENU_UD ;
            }
            event = 0 ;
        }
        else
        {
                newval &= 1 ;
        }
    }


    //change values based on P1
#ifndef NOPOTSCROLL
    newval -= P1values.p1valdiff;
#endif
    if ( RotaryState == ROTARY_VALUE )
    {
            newval += ( ( read_keys() & 2 ) == 0 ) ? 20 * Rotary.Rotary_diff : Rotary.Rotary_diff ;
    }
    if(newval>i_max)
    {
        newval = i_max;
        killEvents(event);
        audioDefevent(AU_KEYPAD_UP);
    }
    else if(newval < i_min)
    {
        newval = i_min;
        killEvents(event);
        audioDefevent(AU_KEYPAD_DOWN);

    }
    if(newval != val)
    {
        if ( menuPressed() )
        {
                LongMenuTimer = 255 ;
        }
        if(newval==0) {
          	  pauseEvents(event);

            if (newval>val){
                audioDefevent(AU_KEYPAD_UP);
            } else {
                audioDefevent(AU_KEYPAD_DOWN);
            }

        }
        eeDirty(i_flags & (EE_GENERAL|EE_MODEL));
    }
    return newval;
}

extern uint8_t EditType ;

NOINLINE int8_t checkIncDec( int8_t i_val, int8_t i_min, int8_t i_max, uint8_t i_flags)
{
    return (int8_t)checkIncDec16( i_val,i_min,i_max,i_flags);
}

int8_t checkIncDec_i8( int8_t i_val, int8_t i_min, int8_t i_max)
{
    return checkIncDec( i_val,i_min,i_max,EditType);
}

int8_t checkIncDec_0( int8_t i_val, int8_t i_max)
{
    return checkIncDec( i_val,0,i_max,EditType) ;
}

int16_t checkIncDec_u0( int16_t i_val, uint8_t i_max)
{
  return checkIncDec16( i_val,0,i_max,EditType) ;
}



void popMenu(bool uppermost)
{
    if(g_menuStackPtr>0 || uppermost)
    {

        if(g_menuStack[g_menuStackPtr] ==  menuProcProtocol)
        {
          resetElrsMenuVariables();
        }
        g_menuStackPtr = uppermost ? 0 : g_menuStackPtr-1;
				EnterMenu = EVT_ENTRY_UP ;
    }
//		else{
//        alert(PSTR(STR_MSTACK_UFLOW));
//    }
}

void chainMenu(MenuFuncP newMenu)
{
    g_menuStack[g_menuStackPtr] = newMenu;
		EnterMenu = EVT_ENTRY ;
}
void pushMenu(MenuFuncP newMenu)
{

    if(g_menuStackPtr >= DIM(g_menuStack)-1)
    {
        alert((STR_MSTACK_OFLOW));
        return;
    }
    EnterMenu = EVT_ENTRY ;
    g_menuStack[++g_menuStackPtr] = newMenu;
}

uint8_t  g_vbat100mV ;
volatile uint8_t tick10ms = 0;
uint16_t g_LightOffCounter;
uint8_t  stickMoved = 0;

inline bool checkSlaveMode()
{
    // no power -> only phone jack = slave mode

#ifdef BUZZER_MOD
    return SlaveMode = SLAVE_MODE ;
#else
    static bool lastSlaveMode = false;

    static uint8_t checkDelay = 0;
    if (audio.busy()) {
        checkDelay = 20;
    }
    else if (checkDelay) {
        --checkDelay;
    }
    else {
        lastSlaveMode = SLAVE_MODE;//
    }
    return (SlaveMode = lastSlaveMode) ;
#endif
}



void backlightKey()
{
  uint8_t a = g_eeGeneral.lightAutoOff ;
  uint16_t b = a * 250 ;
  b <<= 1 ;				// b = a * 500, but less code
  if(b>g_LightOffCounter) g_LightOffCounter = b;
}

void doBackLightVoice(uint8_t evt)
{
    uint8_t a = 0;
    uint16_t b ;
    uint16_t lightoffctr ;
    if(evt) backlightKey() ; // on keypress turn the light on 5*100

    lightoffctr = g_LightOffCounter ;
    if(lightoffctr) lightoffctr--;
    if(stickMoved)
    {
          a = g_eeGeneral.lightOnStickMove ;
          b = a * 250 ;
          b <<= 1 ;				// b = a * 500, but less code
          if(b>lightoffctr) lightoffctr = b;
    }
    g_LightOffCounter = lightoffctr ;
    check_backlight_voice();
}


const static uint8_t rate[8] = { 0, 0, 100, 40, 16, 7, 3, 1 } ;

uint8_t calcStickScroll( uint8_t index )
{
	uint8_t direction ;
	int8_t value ;

	if ( ( g_eeGeneral.stickMode & 1 ) == 0 )
	{
		index ^= 3 ;
	}

	value = phyStick[index] ;
	value /= 8 ;

	direction = value > 0 ? 0x80 : 0 ;
	if ( value < 0 )
	{
		value = -value ;			// (abs)
	}
	uint8_t temp = value ;	// Makes the compiler save 4 bytes flash
	if ( temp > 7 )
	{
		temp = 7 ;
	}
	value = *(rate+temp) ;
	if ( value )
	{
		StickScrollTimer = STICK_SCROLL_TIMEOUT ;		// Seconds
	}
	return value | direction ;
}



struct t_inactivity Inactivity = {0} ;
extern uint8_t s_noHi ;
extern void timer() ;
extern void trace() ;

static void inactivityCheck()
{
	struct t_inactivity *PtrInactivity = &Inactivity ;
	FORCE_INDIRECT(PtrInactivity) ;

  if(s_noHi) s_noHi--;
  uint16_t tsum = stickMoveValue() ;
  if (tsum != PtrInactivity->inacSum)
	{
    PtrInactivity->inacSum = tsum;
    PtrInactivity->inacCounter = 0;
    stickMoved = 1;  // reset in perMain
  }
  else
  {
        uint8_t timer = g_eeGeneral.inactivityTimer + 10 ;
        if( ( timer) && (g_vbat100mV>49))
  	{
            if (++PtrInactivity->inacPrescale > 15 )
            {
            	PtrInactivity->inacCounter++;
            	PtrInactivity->inacPrescale = 0 ;
            	if(PtrInactivity->inacCounter>(uint16_t)((timer)*(100*60/16)))
                  if((PtrInactivity->inacCounter&0x1F)==1)
                  {
                    SystemOptions &= ~SYS_OPT_MUTE; 
                    setVolume(NUM_VOL_LEVELS-2) ;		// Nearly full volume
                    audioVoiceDefevent( AU_INACTIVITY, V_INACTIVE ) ;
                  }
            }
        }
  }
}


int8_t getGvarSourceValue( uint8_t src )
{
	int16_t value = 0 ;

	if ( src <= 4 )
	{
		value = getTrimValue( CurrentPhase, src - 1 ) ;
	}
	else if ( src <= 9 )	// Stick
	{
		value = calibratedStick[ src-5 - 1 ] / 8 ;
	}
	else if ( src <= 12 )	// Pot
	{
		value = calibratedStick[ ( src-6)] / 8 ;
	}
	else if ( src <= 28 )	// Chans
	{
		value = Ex_chans[src-13] / 10 ;
	}
//	else
//	{
//static uint8_t GvLastSw[MAX_GVARS] ;
//#if defined(CPUM128) || defined(CPUM2561)
//		uint8_t j = ( src - 29 ) ;
//		uint8_t k = j & 1 ;
//		j &= 0xFE ;
//		uint8_t sw0 = getSwitch00(DSW_SW1+j) ;
//		value = g_model.gvars[i].gvar ;
//		if ( !GvLastSw[i] && sw0 )
//		{
//      value += k ? -1 : 1 ;
//		}
//		GvLastSw[i] = sw0 ;
//		sw0 = getSwitch00(DSW_SW2+j) ;
//		if ( sw0 )
//		{
//      value = 0 ;
//		}
//#else
//		uint8_t j = ( src - 29 ) * 2 ;
//		uint8_t sw0 = getSwitch00(DSW_SW1+j) ;
//		value = g_model.gvars[i].gvar ;
//		if ( !GvLastSw[i] && sw0 )
//		{
//      value += 1 ;
//		}
//		GvLastSw[i] = sw0 ;
//		sw0 = getSwitch00(DSW_SW2+j) ;
//		if ( sw0 )
//		{
//      value = 0 ;
//		}
//#endif
//	}
	return validatePlusMinus125( value ) ; // limit( -125, value, 125 ) ;
}

static void perMain()
{
    static uint32_t lastTMR;
    uint32_t t10ms;
    t10ms =  g_tmr10ms;
    tick10ms = t10ms - lastTMR;
    lastTMR = t10ms;

    perOutPhase(g_chans512, 0);

    if(tick10ms == 0) return ; //make sure the rest happen only every 10ms.

    inactivityCheck() ;
    timer() ;
    trace(); //trace thr 0..32  (/32)

    if ( ppmInAvailable )
    {
       ppmInAvailable -= 1 ;
    }

    eeCheck();

    // Every 10mS update backlight output to external latch
    // Note: LcdLock not needed here as at tasking level

    lcd_clear();
    uint8_t evt=getEvent();


    evt = checkTrim(evt);
    if ( ( evt == 0 ) || ( evt == EVT_KEY_REPT(KEY_MENU) ) )
    {
      uint8_t timer = LongMenuTimer ;
      if ( menuPressed() )
      {
          if ( timer < 255 )
          {
            timer += 1 ;
          }
      }
      else
      {
        timer = 0 ;
      }
      if ( timer == 200 )
      {
        evt = EVT_TOGGLE_GVAR ;
        timer = 255 ;
      }
      LongMenuTimer = timer ;
    }

    #ifndef NOPOTSCROLL
    int16_t p1d ;

    struct t_p1 *ptrp1 ;
    ptrp1 = &P1values ;
    FORCE_INDIRECT(ptrp1) ;

    int16_t c6 = calibratedStick[6] ;
    p1d = ( ptrp1->p1val-c6 )/32;
    if(p1d) {
        p1d = (ptrp1->p1valprev-c6)/2;
        ptrp1->p1val = c6 ;
    }
    ptrp1->p1valprev = c6 ;
    if ( g_eeGeneral.disablePotScroll || (scroll_disabled) )
    {
       p1d = 0 ;
    }
    ptrp1->p1valdiff = p1d ;
    #endif

    struct t_rotary *protary = &Rotary ;
    FORCE_INDIRECT(protary) ;
    {
        int8_t x ;
        x = protary->RotCount - protary->LastRotaryValue ;
        if ( x == -1 )
        {
            x = 0 ;
        }
        protary->Rotary_diff = ( x ) / 2 ;
        protary->LastRotaryValue += protary->Rotary_diff * 2 ;
    }

    doBackLightVoice( evt | protary->Rotary_diff ) ;
    // Handle volume
    uint8_t requiredVolume ;
    requiredVolume = g_eeGeneral.volume+(NUM_VOL_LEVELS-1) ;

    if ( ( g_menuStack[g_menuStackPtr] == menuProc0) && ( PopupData.PopupActive == 0 ) )
    {
        if ( protary->Rotary_diff )
        {
            int16_t x = protary->RotaryControl ;
            x += protary->Rotary_diff ;
            protary->RotaryControl = validatePlusMinus125( x ) ;
            protary->Rotary_diff = 0 ;
        }

        if ( g_model.anaVolume )	// Only check if on main screen
        {
            uint16_t v ;
            uint16_t divisor ;
            if ( g_model.anaVolume < 4 )
            {
                v = calibratedStick[g_model.anaVolume+3] + 1024 ;
                divisor = 2048 ;
            }
            else
            {

                v = g_model.gvars[g_model.anaVolume-4+3].gvar + 125 ;	// +3 to get to GV4-GV7
                divisor = 250 ;
            }
             requiredVolume = v * (NUM_VOL_LEVELS-1) / divisor ;
        }
    }
    if ( requiredVolume != CurrentVolume )
    {
        setVolume( requiredVolume ) ;
    }

    if ( g_eeGeneral.stickScroll && StickScrollAllowed )
    {
        if ( StickScrollTimer )
        {
          static uint8_t repeater ;
          uint8_t direction ;
          uint8_t value ;

          if ( repeater < 128 )
          {
            repeater += 1 ;
          }
          value = calcStickScroll( 2 ) ;
          direction = value & 0x80 ;
          value &= 0x7F ;
          if ( value )
          {
              if ( repeater > value )
              {
                repeater = 0 ;
                if ( evt == 0 )
                {
                      if ( direction )
                      {
                              evt = EVT_KEY_FIRST(KEY_UP) ;
                      }
                      else
                      {
                              evt = EVT_KEY_FIRST(KEY_DOWN) ;
                      }
                }
              }
          }
          else
          {
            value = calcStickScroll( 3 ) ;
            direction = value & 0x80 ;
            value &= 0x7F ;
            if ( value )
            {
                if ( repeater > value )
                {
                      repeater = 0 ;
                      if ( evt == 0 )
                      {
                              if ( direction )
                              {
                                      evt = EVT_KEY_FIRST(KEY_RIGHT) ;
                              }
                              else
                              {
                                      evt = EVT_KEY_FIRST(KEY_LEFT) ;
                              }
                      }
                }
            }
         }
      }
    }
    else
    {
       StickScrollTimer = 0 ;		// Seconds
    }

    StickScrollAllowed = 1 ;

    //			uint8_t *p = GvarSource ;
    //			FORCE_INDIRECT(p) ;
    //			*p = 0 ;
    //			*(p+1) = 0 ;
    //			*(p+2) = 0 ;
    //			*(p+3) = 0 ;
    for( uint8_t i = 0 ; i < MAX_GVARS ; i += 1 )
    {
        if ( g_model.gvars[i].gvsource )
        {
          int16_t value ;
          uint8_t src = g_model.gvars[i].gvsource ;

   
          if ( src == 5 )	// REN
          {
                value = Rotary.RotaryControl ;
          }
          // The following is to action GVAR adjustment
          else
          {
              value = getGvarSourceValue( src ) ;
          }
          g_model.gvars[i].gvar = validatePlusMinus125( value ) ; // limit( -125, value, 125 ) ;
        }
    }
    
    


    static uint8_t alertKey ;
    if ( AlertMessage )
    {
        almess( AlertMessage, ALERT_TYPE ) ;
        uint8_t key = keyDown() ;
        if ( alertKey )
        {
          if( key == 0 )
          {
             AlertMessage = 0 ;
          }
        }
        else if ( key )
        {
           alertKey = 1 ;
        }
    }
    else
    {
        alertKey = 0 ;

        if ( EnterMenu )
        {
            evt = EnterMenu ;
            EnterMenu = 0 ;
            audioDefevent(AU_MENUS);
        }
        StepSize = 20 ;
        Tevent = evt ;

        g_menuStack[g_menuStackPtr](evt);
    }
    refreshDiplay();
   
    switch( g_blinkTmr10ms & 0x1f ) { //alle 10ms*32

          case 2:
          {
            //TODO check v-bat
            //        Calculation By Mike Blandford
            //        Resistor divide on battery voltage is 5K1 and 2K7 giving a fraction of 2.7/7.8
            //        If battery voltage = 10V then A2D voltage = 3.462V
            //        11 bit A2D count is 1417 (3.462/5*2048).
            //        1417*18/256 = 99 (actually 99.6) to represent 9.9 volts.
            //        Erring on the side of low is probably best.

            int16_t ab = anaIn(7);
            ab = ab * 16 + ab / 8 * g_eeGeneral.vBatCalib;
            ab = (uint16_t) ab / 330;
            g_vbat100mV = (ab + g_vbat100mV + 1) >> 1 ;  // Filter it a bit => more stable display

            static uint8_t s_batCheck;
            s_batCheck+=16 ;
            if((s_batCheck == 0) && (g_vbat100mV < g_eeGeneral.vBatWarn) /*&& (g_vbat100mV>49)*/)
            {

               audioVoiceDefevent(AU_TX_BATTERY_LOW, V_BATTERY_LOW);
                voice_numeric(g_vbat100mV,1, V_VOLTS);
                #ifndef MINIMISE_CODE
                if (g_eeGeneral.flashBeep)
                   g_LightOffCounter = FLASH_DURATION;
                #endif
            }
          }
          break;
    }


    stickMoved = 0; //reset this flag

}

int16_t g_ppmIns[8];
uint8_t ppmInState = 0; //0=unsync 1..8= wait for value i-1

#ifndef SIMU
uint16_t s_anaFilt[8] ;
uint16_t anaIn(uint8_t chan)
{
    // ana-in:   3 1 2 0 4 5 6 7
	uint8_t pchan = chan ;
	if ( chan == 3 )
	{
		pchan = 0 ;
	}
	else
	{
		if ( chan == 0 )
		{
			pchan = 3 ;
		}
	}
        uint16_t temp = s_anaFilt[pchan] ;
	if ( chan < 4 )	// A stick
	{
		if ( g_eeGeneral.stickReverse & ( 1 << chan ) )
		{
			temp = 2048 - temp ;
		}
	}
	return temp ;
}



#endif

volatile uint16_t g_tmr16KHz;
volatile uint16_t tmrEEPROM;


/*----------------------------------------------------------------------------*/
// Clocks every 128 uS
void ISR_TIMER2_OVF_vect(void) {
  if (audio.busy()) {  
    AUDIO_DRIVER(); // the tone generator
  }
#ifdef VARIO
  else
    Vario_driver();
#endif
}
/*----------------------------------------------------------------------------*/
// Clocks every 10 mS
#ifdef CPUM2561
ISR(TIMER0_COMPA_vect, ISR_NOBLOCK) // 10ms timer
#else
void ISR_TIMER0_COMP_vect(void) // 10ms timer
#endif
{

  AUDIO_HEARTBEAT(); // the queue processing

  per10ms();
#ifdef FRSKY
  check_frsky();
#endif
  heartbeat |= HEART_TIMER10ms;
  // See if time for alarm checking
  struct t_alarmControl *pac = &AlarmControl;
  FORCE_INDIRECT(pac);

  if (--pac->AlarmTimer == 0) {
    pac->AlarmTimer = 100; // Restart timer
                           //			pac->AlarmCheckFlag += 1 ;	// Flag time to check alarms
    pac->OneSecFlag = 1;
  }
  if (--pac->VoiceFtimer == 0) {
    pac->VoiceFtimer = 10;    // Restart timer
    pac->VoiceCheckFlag |= 1; // Flag time to check alarms
  }

  //  } // end 10ms event
}
/*----------------------------------------------------------------------------*/
// Timer3 used for PPM_IN pulse width capture. Counter running at 16MHz / 8 = 2MHz
// equating to one count every half millisecond. (2 counts = 1ms). Control channel
// count delta values thus can range from about 1600 to 4400 counts (800us to 2200us),
// corresponding to a PPM signal in the range 0.8ms to 2.2ms (1.5ms at center).
// (The timer is free-running and is thus not reset to zero at each capture interval.)
//void ISR_TIMER3_CAPT_vect(void)    // capture ppm in 16MHz / 8 = 2MHz
//{
//  uint16_t capture = GetPPMTimCapture();
//  static uint16_t lastCapt = 0;
//  uint16_t val        = ((uint16_t)(capture - lastCapt)) / 3;
//  lastCapt            = capture;
//  uint8_t lppmInState = ppmInState;
//  // We prcoess g_ppmInsright here to make servo movement as smooth as possible
//  //    while under trainee control
//  if (val > 4000 && val < 16000)    // G: Prioritize reset pulse. (Needed when less than 8 incoming pulses)
//    lppmInState = 1;                // triggered
//  else {
//    if (lppmInState && lppmInState <= 8) {
//      if (val > 800 && val < 2200) {
//        ppmInAvailable              = 100;
//        g_ppmIns[lppmInState++ - 1] = (int16_t)(val - 1500) * (uint8_t)(g_eeGeneral.PPM_Multiplier + 10) / 10;    //+-500 != 512, but close enough.

//      } else {
//        lppmInState = 0;    // not triggered
//      }
//    }
//  }
//  ppmInState = lppmInState;
//}
/*----------------------------------------------------------------------------*/
extern struct t_latency g_latency ;
//void main(void) __attribute__((noreturn));

#if STACK_TRACE
extern unsigned char __bss_end ;

unsigned int stack_free()
{
    unsigned char *p ;

    p = &__bss_end + 1 ;
    while ( *p == 0x55 )
    {
        p+= 1 ;
    }
    return p - &__bss_end ;
}
#endif

#ifdef CPUM2561
uint8_t SaveMcusr ;
#endif

void mainER(void)
{

extern uint8_t serialDat0 ;
	serialDat0 = 0xFF ;



#ifdef SERIAL_VOICE
	{
		serialVoiceInit(on_voice_cb) ;

	}
#endif


#ifdef BLIGHT_DEBUG
	{
		uint32_t x ;
		x = 0 ;
		for ( ;; )
		{
			for ( x = 0 ; x < 500000 ; x += 1 )
			{
				PORTB |= 0x80 ;	// Backlight on
	      wdt_reset() ;
				asm(" nop") ;											// delay to allow input to settle
				asm(" nop") ;											// delay to allow input to settle
			}
			for ( x = 0 ; x < 500000 ; x += 1 )
			{
				PORTB &= 0x7F ;	// Backlight off
	      wdt_reset() ;
				asm(" nop") ;											// delay to allow input to settle
				asm(" nop") ;											// delay to allow input to settle
				asm(" nop") ;											// delay to allow input to settle
				asm(" nop") ;											// delay to allow input to settle
				asm(" nop") ;											// delay to allow input to settle
				asm(" nop") ;											// delay to allow input to settle

			}
		}
	}
#endif




#if STACK_TRACE
    // Init Stack while interrupts are disabled
#define STACKPTR     _SFR_IO16(0x3D)
    {

        unsigned char *p ;
        unsigned char *q ;

        p = (unsigned char *) STACKPTR ;
        q = &__bss_end ;
        p -= 2 ;
        while ( p > q )
        {
            *p-- = 0x55 ;
        }
    }
#endif
    sei(); //damit alert in eeReadGeneral() nicht haengt

    g_menuStack[0] =  menuProc0;

    if (eeReadGeneral())
    {
        lcd_init() ;   // initialize LCD module after reading eeprom
    }
    else
    {
        eeGeneralDefault(); // init g_eeGeneral with default values
        lcd_init();         // initialize LCD module for ALERT box
        eeWriteGeneral();   // format/write back to eeprom
    }


      uint8_t in ;
	while ( (in = ~PIND() & 0xC3) == 0x40 )
	{
		SystemOptions = SYS_OPT_HARDWARE_EDIT ;

	}
	
        if ( in == 0x82 )
	{
		cli() ;
#ifndef BOOTL
		lcd_puts_Pleft( FH, PSTR("\005Stopped") ) ;
		refreshDiplay() ;
		for ( ;; )	// This allows the Megasound board to 'talk' to a PC over serial
		{
		}
#endif // nBOOTL

	}

	uint8_t cModel = g_eeGeneral.currModel;
	eeLoadModel( cModel ) ;


       checkQuickSelect();


  //we assume that startup is like pressing a switch and moving sticks.  Hence the lightcounter is set
  //if we have a switch on backlight it will be able to turn on the backlight.
	
        stickMoved = 1 ;
        doBackLightVoice(1) ;
        stickMoved = 0 ;
	
 // moved here and logic added to only play statup tone if splash screen enabled.
 // that way we save a bit, but keep the option for end users!
    setVolume(g_eeGeneral.volume + NUM_VOL_LEVELS-1) ;


    if(!g_eeGeneral.disableSplashScreen)
    {
        if( g_eeGeneral.speakerMode )		// Not just beeper
        {
                              audioVoiceDefevent( AU_TADA, V_HELLO ) ;
        }
        doSplash();

        checkMem();
        getADC_osmp();
        g_vbat100mV = anaIn(7) / 14 ;
        checkTHR();
        checkSwitches();
    #ifndef MINIMISE_CODE
        checkAlarm();
    #endif
        checkWarnings();
        clearKeyEvents(); //make sure no keys are down before proceeding
    	putVoiceQueueUpper( g_model.modelVoice ) ;
    }

    AlarmControl.VoiceCheckFlag |= 2 ;// Set switch current states
    CurrentPhase = 0 ;
    perOutPhase(g_chans512, 0 ) ;
    startPulses() ;
    wdt_enable(WDTO_500MS);

    g_menuStack[1] = menuProcModelSelect ;	// this is so the first instance of [MENU LONG] doesn't freak out!
    
    Main_running = 1 ;
    while(1){
        mainSequence() ;
    }
}


uint8_t isAgvar(uint8_t value)
{
	if ( value >= 62 )
	{
		if ( value <= 68 )
		{
			return 1 ;
		}
	}
	return 0 ;
}



void doVoiceAlarmSource( VoiceAlarmData *pvad )
{
	if ( pvad->source )
	{
		// SORT OTHER values here
		if ( pvad->source >= NUM_XCHNRAW )
		{
			voice_telem_item( pvad->source - NUM_XCHNRAW - 1 ) ;
		}
		else
		{
			int16_t value ;
			value = getValue( pvad->source - 1 ) ;
			voice_numeric( value, 0, 0 ) ;
		}
	}
}

void procOneVoiceAlarm(VoiceAlarmData *pvad, uint8_t i) {
  uint8_t curent_state;
  uint8_t play = 0;
  curent_state = 0;
  int16_t ltimer = Nvs_timer[i];

  if (pvad->func) // Configured
  {
    int16_t x;
    int16_t y = pvad->offset;
    x = getValue(pvad->source - 1);
    switch (pvad->func) {
    case 1:
      x = x > y;
      break;
    case 2:
      x = x < y;
      break;
    case 3:
    case 4:
      x = abs(x);
      x = (pvad->func == 3) ? x > y : x < y;
      break;
    case 5: {
      if (isAgvar(pvad->source)) {
        x *= 10;
        y *= 10;
      }
      x = abs(x - y) < 32;
    } break;
    case 6:
      x = x == y;
      break;
    }
    // Start of invalid telemetry detection
    //					if ( pvad->source > ( CHOUT_BASE - NUM_SKYCHNOUT ) )
    //					{ // Telemetry item
    //						if ( !telemItemValid( pvad->source - 1 - CHOUT_BASE - NUM_SKYCHNOUT ) )
    //						{
    //							x = 0 ;	// Treat as OFF
    //						}
    //					}
    // End of invalid telemetry detection
    if (pvad->swtch) {
      if (getSwitch00(pvad->swtch) == 0) {
        x = 0;
      }
    }
    if (x == 0) {
      ltimer = 0;
    } else {
      play = 1;
    }
  } else // No function
  {
    if (pvad->swtch) {
      curent_state = getSwitch00(pvad->swtch);
      if (curent_state == 0) {
        //							Nvs_state[i] = 0 ;
        ltimer = -1;
      }
    } else                 // No switch, no function
    {                      // Check for source with numeric rate
      if (pvad->rate >= 4) // A time
      {
        if (pvad->vsource) {
          play = 1;
        }
      }
    }
  }
  play |= curent_state;

  uint8_t l_nvsState = Nvs_state[i];

  if ((AlarmControl.VoiceCheckFlag & 2) == 0) {
    if (pvad->rate == 3) // All
    {
      uint8_t pos = switchPosition(pvad->swtch);
      if (l_nvsState != pos) {
        l_nvsState = pos;
        ltimer = 0;
        play = pos + 1;
      } else {
        play = 0;
      }
    } else {
      if (play == 1) {
        if (l_nvsState == 0) {                          // just turned ON
          if ((pvad->rate == 0) || (pvad->rate == 2)) { // ON
            ltimer = 0;
          }
        }
        l_nvsState = 1;
        if (pvad->rate == 1) {
          play = 0;
        }
      } else {
        if (l_nvsState == 1) {
          if ((pvad->rate == 1) || (pvad->rate == 2)) {
            ltimer = 0;
            play = 1;
            if (pvad->rate == 2) {
              play = 2;
            }
          }
        }
        l_nvsState = 0;
      }
      if (pvad->rate == 33) {
        play = 0;
        ltimer = -1;
      }
    }
  } else {
    l_nvsState = play;
    play = (pvad->rate == 33) ? 1 : 0;
    ltimer = -1;
  }

  Nvs_state[i] = l_nvsState;

  if (pvad->mute) {
    if (pvad->source > (CHOUT_BASE + NUM_CHNOUT)) { // Telemetry item
      if (!telemItemValid(pvad->source - 1 - CHOUT_BASE - NUM_CHNOUT)) {
        play = 0; // Mute it
      }
    }
  }

  if (play) {
    if (ltimer < 0) {
      if (pvad->rate >= 3) // A time or ONCE
      {
        ltimer = 0;
      }
    }
    if (ltimer == 0) {
      if (pvad->vsource == 1) {
        doVoiceAlarmSource(pvad);
      }
      if (pvad->fnameType == 0) // None
      {
        // Nothing!
      } else if (pvad->fnameType == 1) // Name
                                       //				{
                                       //					char name[10] ;
                                       //					char *p ;
                                       //					p = (char *)cpystr( (uint8_t *)name, pvad->file.name ) ;
                                       //					if ( play == 2 )
                                       //					{
                                       //						*(p-1) += 1 ;
                                       //					}
                                       //					putUserVoice( name, 0 ) ;
                                       //				}
                                       //				else if ( pvad->fnameType == 2 )	// Number
      {
        uint16_t value = pvad->vfile;
        if (value > 507) {
          value = calc_scaler(value - 508, 0, 0);
        } else if (value > 500) {
          value = g_model.gvars[value - 501].gvar;
        }
        putVoiceQueueLong(value + (play - 1));
        //				putVoiceQueueLong( pvad->vfile +( play - 1 ) ) ;
      } else { // Audio
        audioEvent(pvad->vfile, 0);
      }
      if (pvad->vsource == 2) {
        doVoiceAlarmSource(pvad);
      }
      if (pvad->haptic) {
        audioDefevent((pvad->haptic > 1) ? ((pvad->haptic == 3) ? AU_HAPTIC3 : AU_HAPTIC2) : AU_HAPTIC1);
      }
      if ((pvad->rate < 3) || (pvad->rate > 32)) // Not a time
      {
        ltimer = -1;
      } else {
        ltimer = 1;
      }
    } else if (ltimer > 0) {
      ltimer += 1;
      if (ltimer > ((pvad->rate - 2) * 10)) {
        ltimer = 0;
      }
    }
  }
  pvad += 1;
  Nvs_timer[i] = ltimer;
}

NOINLINE void processVoiceAlarms()
{
	uint8_t i ;
//	uint8_t curent_state ;
	VoiceAlarmData *pvad = &g_model.vad[0] ;
//	FORCE_INDIRECT(pvad) ;

	for ( i = 0 ; i < NUM_VOICE_ALARMS ; i += 1 )
	{
		procOneVoiceAlarm( pvad, i ) ;
		pvad += 1 ;
	}
}


void mainSequence()
{
	CalcScaleNest = 0 ;
      
	uint16_t t0 = g_tmr16KHz;


  getADC_osmp() ;


  perMain();  
  
  if(heartbeat == 0x3)
  {
      wdt_reset();
      heartbeat = 0;
  }
  t0 = g_tmr16KHz - t0;
  if ( t0 > g_latency.g_timeMain ) g_latency.g_timeMain = t0 ;

  if ( AlarmControl.VoiceCheckFlag )		// Every 100 mS
  {
		uint8_t i ;
		static uint16_t timer ;
#ifdef V2
		static uint8_t delayTimer = 0 ;
#endif // V2

#ifdef V2
#ifdef USE_ADJUSTERS
		processAdjusters() ;
#endif // 128/2561
#endif // V2

		timer += 1 ;
#ifdef V2
		if ( delayTimer )
		{
			delayTimer -= 1 ;
		}

#ifdef FRSKY    // htc
		uint8_t redAlert = 0 ;
		static uint8_t redCounter ;
		static uint8_t orangeCounter ;
		uint8_t rssiValue = FrskyHubData[FR_RXRSI_COPY] ;

		if ( frskyStreaming )
		{
			if ( g_model.frsky.rxRssiCritical == 0 )
			{
				if ( rssiValue && rssiValue < g_model.frsky.rxRssiCritical + 42 )
				{
					// Alarm
					redAlert = 1 ;
					orangeCounter += 1 ;
					if ( ++redCounter > 3 )
					{
						if ( delayTimer == 0 )
						{
							audioVoiceDefevent(AU_WARNING3, V_RSSI_CRITICAL );
							delayTimer = 40 ;	// 4 seconds
						}
						redCounter = 0 ;
					}
				}
				else
				{
					redCounter = 0 ;
				}
			}
			if ( ( redAlert == 0 ) && ( g_model.frsky.rxRssiCritical == 0 ) )
			{
				if ( rssiValue && rssiValue < g_model.frsky.rxRssiLow + 45 )
				{
					// Alarm
					if ( ++orangeCounter > 3 )
					{
						if ( delayTimer == 0 )
						{
							audioVoiceDefevent(AU_WARNING2, V_RSSI_WARN );
							delayTimer = 40 ;	// 4 seconds
						}
						orangeCounter = 0 ;
					}
				}
				else
				{
					orangeCounter = 0 ;
				}
			}
		}
#endif // FRSKY
#endif // V2

#ifdef V2
		for ( i = 0 ; i < EXTRA_VOICE_SW ; i += 1 )
		{
			uint8_t curent_state ;
			uint8_t mode ;
			uint8_t value ;
    	VoiceSwData *pvd = &g_model.voiceSw[i];

			mode = pvd->mode ;
			value = pvd->val ;
//			if ( mode <= 5 )
//			{
//				if ( value > 250 )
//				{
//					value = g_model.gvars[value-248].gvar ; //Gvars 3-7
//				}
//			}

			if ( pvd->swtch )		// Configured
			{
				curent_state = getSwitch00( pvd->swtch ) ;
				if ( ( AlarmControl.VoiceCheckFlag & 2) == 0 )
				{
					if ( ( mode == 0 ) || ( mode == 2 ) )
					{ // ON
						if ( ( Vs_state[i] == 0 ) && curent_state )
						{
							putVoiceQueue( value ) ;
						}
					}
					if ( ( mode == 1 ) || ( mode == 2 ) )
					{ // OFF
						if ( ( Vs_state[i] == 1 ) && !curent_state )
						{
//							uint8_t x ;
//							x = sd->opt.vs.vval ;
							if ( mode == 2 )
							{
								value += 1 ;
							}
							putVoiceQueue( value ) ;
						}
					}
					if ( mode == 3 )
					{ // ALL
			 			uint8_t pos = switchPosition( pvd->swtch ) ;
						if ( Vs_state[i] != pos )
						{
							putVoiceQueue( value + pos ) ;
						}
						curent_state = pos ;
					}
				}
				else
				{
					if ( mode == 4 )
					{ // ONCE
						curent_state = 1 ;
						putVoiceQueue( value ) ;
					}
				}
				Vs_state[i] = curent_state ;
			}
		}
		for ( i = 0 ; i < NUM_SCALERS ; i += 1 )
		{
			if ( g_model.eScalers[i].dest )
			{
				calc_scaler( i, 0, 0 ) ;
			}
		}
#else
//#if defined(CPUM128) || defined(CPUM2561)
#ifndef NOVOICE_SW
		uint16_t ltimer = timer ;
#ifndef V2
		uint8_t numSafety = 16 - g_model.numVoice ;
#endif // nV2
		for ( i = numSafety ; i < NUM_CHNOUT+EXTRA_VOICE_SW ; i += 1 )
//#else
//		for ( i = numSafety ; i < NUM_CHNOUT ; i += 1 )
//#endif
		{
			uint8_t curent_state ;
			uint8_t mode ;
			uint8_t value ;
    	SafetySwData *sd = &g_model.safetySw[i];
//#if defined(CPUM128) || defined(CPUM2561)
    	if ( i >= NUM_CHNOUT )
			{
				sd = &g_model.xvoiceSw[i-NUM_CHNOUT];
			}
//#endif

			mode = sd->opt.vs.vmode ;
			value = sd->opt.vs.vval ;
			if ( mode <= 5 )
			{
				if ( value > 250 )
				{
					value = g_model.gvars[value-248].gvar ; //Gvars 3-7
				}
			}

			if ( sd->opt.vs.vswtch )		// Configured
			{
				curent_state = getSwitch00( sd->opt.vs.vswtch ) ;
				if ( ( AlarmControl.VoiceCheckFlag & 2) == 0 )
				{
					if ( ( mode == 0 ) || ( mode == 2 ) )
					{ // ON
						if ( ( Vs_state[i] == 0 ) && curent_state )
						{
							putVoiceQueue( value ) ;
						}
					}
					if ( ( mode == 1 ) || ( mode == 2 ) )
					{ // OFF
						if ( ( Vs_state[i] == 1 ) && !curent_state )
						{
//							uint8_t x ;
//							x = sd->opt.vs.vval ;
							if ( mode == 2 )
							{
								value += 1 ;
							}
							putVoiceQueue( value ) ;
						}
					}
					if ( mode > 5 )
					{
						if ( ( Vs_state[i] == 0 ) && curent_state )
						{
							voice_telem_item( sd->opt.vs.vval ) ;
						}
					}
					else if ( mode > 2 )
					{ // 15, 30 or 60 secs
						if ( curent_state )
						{
							uint16_t mask ;
							mask = 150 ;
							if ( mode == 4 ) mask = 300 ;
							if ( mode == 5 ) mask = 600 ;
							if ( ltimer % mask == 0 )
							{
								putVoiceQueue( value ) ;
							}
						}
					}
				}
				Vs_state[i] = curent_state ;
			}
		}
#endif // nNOVOICE_SW
#endif // V2

#ifdef V2
		for ( i = 0 ; i < NUM_CSW+EXTRA_CSW ; i += 1 )
#else
		for ( i = 0 ; i < NUM_CSW ; i += 1 )
#endif // V2
		{
#ifdef V2
			CxSwData *cs = &g_model.customSw[i] ;
#else
			CSwData *cs = &g_model.customSw[i] ;
#endif // V2
    	uint8_t cstate = CS_STATE(cs->func);

    	if(cstate == CS_TIMER)
			{
				int16_t y ;
				y = CsTimer[i] ;
				if ( y == 0 )
				{
					int8_t z ;
					z = cs->v1 ;
					if ( z >= 0 )
					{
						z = -z-1 ;
						y = z * 10 ;
					}
					else
					{
						y = z ;
					}
				}
				else if ( y < 0 )
				{
					if ( ++y == 0 )
					{
						int8_t z ;
						z = cs->v2 ;
						if ( z >= 0 )
						{
							z += 1 ;
							y = z * 10 - 1  ;
						}
						else
						{
							y = -z-1 ;
						}
					}
				}
				else  // if ( CsTimer[i] > 0 )
				{
					y -= 1 ;
				}
				if ( cs->andsw )
				{
					int8_t x ;
					x = cs->andsw ;
					if ( x > 8 )
					{
						x += 1 ;
					}
	        if (getSwitch00( x) == 0 )
				  {
						y = -1 ;
					}
				}
				CsTimer[i] = y ;
			}
#ifdef VERSION3
			uint8_t lastSwitch = Last_switch[i] ;
			if ( cs->func == CS_LATCH )
			{
		    if (getSwitch00( cs->v1) )
				{
					lastSwitch = 1 ;
				}
				else
				{
			    if (getSwitch00( cs->v2) )
					{
						lastSwitch = 0 ;
					}
				}
			}
			if ( cs->func == CS_FLIP )
			{
		    if (getSwitch00( cs->v1) )
				{
					if ( ( lastSwitch & 2 ) == 0 )
					{
						// Clock it!
			      if (getSwitch00( cs->v2) )
						{
							lastSwitch = 3 ;
						}
						else
						{
							lastSwitch = 2 ;
						}
					}
				}
				else
				{
					lastSwitch &= ~2 ;
				}
			}
			Last_switch[i] = lastSwitch ;

#endif
		}

		processVoiceAlarms() ;
		AlarmControl.VoiceCheckFlag = 0 ;

#ifdef VARIO
                // Vario
                {
                  if (g_model.varioData.varioSource) // Vario enabled
                  {
                    if (getSwitch00(g_model.varioData.swtch)) {
                      int16_t vspd = 0;
                      switch (g_model.varioData.varioSource) {
                      case 1:
                        if (AFHDS2A_tel_status | ((uint64_t)1 << FST_IDX_VERTICAL_SPEED)) {
                          vspd = AFHDS2A_tel_data[FST_IDX_VERTICAL_SPEED];
                        }
                        break;
                      case 2:
                      case 3:
                      case 4:
                      case 5:
                        vspd = calc_scaler(g_model.varioData.varioSource - 2, 0, 0);
                        break;
                      default:
                        break;
                      }

                      //	int8_t index = pgm_read_byte( &TelemIndex[V_SC1] ) ;

                      Vario_play(vspd, true);

                    } else
                      Vario_play(0, false);
                  }
                }
#endif // Vario
  }

  if (AlarmControl.OneSecFlag) // Custom Switch Timers
  {
#ifndef V2
#ifndef NOSAFETY_A_OR_V
		uint8_t i ;
#endif
#endif // nV2
//      AlarmControl.AlarmCheckFlag = 0 ;
      // Check for alarms here
      // Including Altitude limit
//				Debug3 = 1 ;
//		MixRate = MixCounter ;
//		MixCounter = 0 ;
#ifdef FRSKY
      if (frskyUsrStreaming)
      {
					uint16_t total_volts = 0 ;
#if VOLT_THRESHOLD
					uint8_t audio_sounded = 0 ;
#endif
					uint8_t low_cell = 220 ;		// 4.4V
				  for (uint8_t k=0; k<FrskyBattCells; k++)
					{
						total_volts += FrskyVolts[k] ;
						if ( FrskyVolts[k] < low_cell )
						{
							low_cell = FrskyVolts[k] ;
						}

#if VOLT_THRESHOLD
						if ( audio_sounded == 0 )
						{
	        		if ( FrskyVolts[k] < g_model.frSkyVoltThreshold )
							{
	            	audioDefevent(AU_WARNING3);
								audio_sounded = 1 ;
			        }
						}
#endif
	  			}
					// Now we have total volts available
					FrskyHubData[FR_CELLS_TOT] = total_volts / 5 ;
					if ( low_cell < 220 )
					{
						FrskyHubData[FR_CELL_MIN] = low_cell ;
					}

      }


      // this var prevents and alarm sounding if an earlier alarm is already sounding
      // firing two alarms at once is pointless and sounds rubbish!
      // this also means channel A alarms always over ride same level alarms on channel B
      // up to debate if this is correct!
      //				bool AlarmRaisedAlready = false;

      if (frskyStreaming)
			{
					// Check for current alarm
#ifndef V2
#ifdef MAH_LIMIT
					if ( g_model.currentSource )
					{
						if ( g_model.frsky.frskyAlarmLimit )
						{
							if ( ( FrskyHubData[FR_AMP_MAH] >> 6 ) >= g_model.frsky.frskyAlarmLimit )
							{
								if ( g_eeGeneral.speakerMode & 2 )
								{
									putVoiceQueue( V_CAPACITY ) ;
								}
								else
								{
									audioDefevent( g_model.frsky.frskyAlarmSound ) ;
								}
							}
							uint16_t value ;
							value = g_model.frsky.frskyAlarmLimit ;
							value = 100 - ( ((uint16_t)FrskyHubData[FR_AMP_MAH] >> 6) * 100 / value ) ;
							FrskyHubData[FR_FUEL] = value ;
						}
					}
#endif // MAH_LIMIT
#endif // nV2
//					struct t_FrSkyChannelData *aaa = &g_model.frsky.channels[0] ;
//        	for (int i=0; i<2; i++)
//					{
//						// To be enhanced by checking the type as well
//       		  if (aaa->opt.alarm.ratio)
//						{
//     		      if ( aaa->opt.alarm.type == 3 )		// Current (A)
//							{
//								if ( g_model.frsky.frskyAlarmLimit )
//								{
//    		          if ( (  getTelemetryValue(FR_A1_MAH+i) >> 6 ) >= g_model.frsky.frskyAlarmLimit )
//									{
//										if ( g_eeGeneral.speakerMode & 2 )
//										{
//											putVoiceQueue( V_CAPACITY ) ;
//										}
//										else
//										{
//											audio.event( g_model.frsky.frskyAlarmSound ) ;
//										}
//									}
//								}
//							}
//       		  }
//						aaa += 1 ;
//        	}
      }
#endif

#ifndef V2
#ifndef NOSAFETY_A_OR_V
			// Now for the Safety/alarm switch alarms
			// Carried out evey 100 mS
			{
				static uint8_t periodCounter ;
				uint8_t pCounter = periodCounter ;

				pCounter += 0x11 ;
				if ( ( pCounter & 0x0F ) > 11 )
				{
					pCounter &= 0xF0 ;
				}
				periodCounter = pCounter ;
				uint8_t numSafety = 16 - g_model.numVoice ;
				for ( i = 0 ; i < numSafety ; i += 1 )
				{
    			SafetySwData *sd = &g_model.safetySw[i] ;
					if (sd->opt.ss.mode == 1)
					{
						if ( ( pCounter & 0x30 ) == 0 )
						{
							if(getSwitch00( sd->opt.ss.swtch))
							{
								audioDefevent( ((g_eeGeneral.speakerMode & 1) == 0) ? 1 : sd->opt.ss.val ) ;
							}
						}
					}
					if (sd->opt.ss.mode == 2)
					{
						if ( sd->opt.ss.swtch > MAX_DRSWITCH )
						{
							uint8_t temp = sd->opt.ss.swtch ;
							temp -= MAX_DRSWITCH +1 ;
							switch ( temp )
							{
								case 0 :
									if ( ( pCounter & 0x70 ) == 0 )
									{
										voice_telem_item( sd->opt.ss.val ) ;
									}
								break ;
								case 1 :
									if ( ( pCounter & 0x0F ) == 0 )
									{
										voice_telem_item( sd->opt.ss.val ) ;
									}
								break ;
								case 2 :
									if ( ( pCounter & 0xF0 ) == 0x20 )
									{
										voice_telem_item( sd->opt.ss.val ) ;
									}
								break ;
							}
						}
						else if ( ( pCounter & 0x30 ) == 0 )		// Every 4 seconds
						{
							if(getSwitch00( sd->opt.ss.swtch))
							{
								putVoiceQueue( sd->opt.ss.val + 128 ) ;
							}
						}
					}
				}
			}
#endif // NOSAFETY_A_OR_V
#endif // nV2


	// New switch voices
	// New entries, Switch, (on/off/both), voice file index

		AlarmControl.OneSecFlag = 0 ;
//		uint8_t i ;

		if ( StickScrollTimer )
		{
			StickScrollTimer -= 1 ;
		}
	}
}
//#endif


int16_t calc1000toRESX(int16_t x)  // improve calc time by Pat MacKenzie
{
    int16_t y = x>>5;
    x+=y;
    y=y>>2;
    x-=y;
    return x+(y>>2);
    //  return x + x/32 - x/128 + x/512;
}

int8_t REG100_100(int8_t x)
{
	return REG( x, -100, 100 ) ;
}

int8_t REG(int8_t x, int8_t min, int8_t max)
{
  int8_t result = x;
  if (x >= 126 || x <= -126) {
    x = (uint8_t)x - 126;
    result = g_model.gvars[x].gvar ;
    if (result < min) {
      g_model.gvars[x].gvar = result = min;
//      eeDirty( EE_MODEL | EE_TRIM ) ;
    }
    else if (result > max) {
      g_model.gvars[x].gvar = result = max;
//      eeDirty( EE_MODEL | EE_TRIM ) ;
    }
  }
  return result;
}

uint8_t IS_EXPO_THROTTLE( uint8_t x )
{
	if ( g_model.thrExpo )
	{
		return IS_THROTTLE( x ) ;
	}
	return 0 ;
}

int16_t calc100toRESX(int8_t x)
{
    return ((x*41)>>2) - x/64;
}