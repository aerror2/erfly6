#include "er9x.h"
#include "pulses.h"
#include "menus.h"
#include "iface_a7105.h"
#include "LoRa/ELRS.h"
#include "crossfire/crossfire.h"

/*****************************************************************************/



/*****************************************************************************/

extern struct t_latency g_latency ;

//#ifdef SBUS_PROTOCOL	
//#define PULSES_WORD_SIZE	115		// 72=((2+2*6)*10)/2+2
//#define PULSES_BYTE_SIZE	(PULSES_WORD_SIZE * 2)
//#else
//#define PULSES_WORD_SIZE	72		// 72=((2+2*6)*10)/2+2
//#define PULSES_BYTE_SIZE	(PULSES_WORD_SIZE * 2)
//#endif

#define BITLEN_SERIAL (8*2) //125000 Baud
#define BITLEN_SBUS (10*2) //100000 Baud




/*
union p2mhz_t 
{
    uint16_t pword[PULSES_WORD_SIZE] ;   // 72
    uint8_t pbyte[PULSES_BYTE_SIZE] ;   // 144
} pulses2MHz ;
*/
p2mhz_t pulses2MHz ;

struct t_pcm_control
{
	uint8_t PcmByte ;
	uint8_t PcmBitCount ;
	uint8_t *PcmPtr ;
	uint16_t PcmCrc ;
	uint8_t PcmOnesCount ;
	uint8_t Shift ;
	uint8_t Limit ;
} PcmControl ;

uint8_t *pulses2MHzptr = pulses2MHz.pbyte ;
uint8_t heartbeat;
uint8_t Current_protocol;
uint8_t pxxFlag = 0;					// also used for MULTI_PROTOCOL for bind flag
uint8_t PxxExtra ;
uint8_t PausePulses = 0 ;
static uint8_t *Serial_pulsePtr = pulses2MHz.pbyte ;

static uint8_t pass_bitlen ;

static uint8_t PulsePol ;
static uint8_t PulsePol16 ;

uint8_t serialDat0 ;	// Set to 0xFF in main()
#ifdef FAILSAFE  			
uint16_t FailsafeCounter ;
#endif

#define CTRL_END 0
#define CTRL_CNT 1
#define CTRL_REP_1CMD -3
#define CTRL_REP_2CMD -6

#define BindBit 0x80
#define RangeCheckBit 0x20
#define FranceBit 0x10
#define DsmxBit  0x08
#define BadData 0x40


void set_timer3_capture( void ) ;
void set_timer3_ppm( void ) ;
//void setupPulsesPPM16( uint8_t proto ) ;
void setupPulsesPPM( uint8_t proto ) ;
static void setupPulsesPXX( void ) ;
static void sendByteSerial(uint8_t b) ;

//uint16_t PulseTotal ;

//ISR(TIMER1_OVF_vect)
/*------------------------------------------------------------------------------*/
void SetStartPulsePol() {
  if (((!g_model.pulsePol) && (!(GetPPMOutState()))) || ((g_model.pulsePol) && (GetPPMOutState()))) {
    cli();
    SetPPMTimCompare(GetPPMTimCompare() + 1);
    while (!GetPPMTimCompareInterruptFlag()) {
    }
    ClearPPMTimCompareInterruptFlag();
    sei();
  }
}
/*------------------------------------------------------------------------------*/
void ISR_TIMER1_COMPA_vect(void)    // 3MHz pulse generation
{
  static uint16_t *pulsePtr = pulses2MHz.pword;
  SetPPMTimCompare(GetPPMTimCompare() + *pulsePtr);
  pulsePtr += 1;

  if (*pulsePtr == 0) {
    pulsePtr = pulses2MHz.pword;
    SetStartPulsePol();
    setupPulses();
  }
  heartbeat |= HEART_TIMER2Mhz;
}
/*------------------------------------------------------------------------------*/
void startPulses() {
	Current_protocol = g_model.protocol + 10;		// Not the same!
	PausePulses = 0;
	setupPulses();
}
/*------------------------------------------------------------------------------*/
/*
void setPpmTimers()
{
PPM_TMR_Enable(PPM_TMR_DeviceData);
TPM_PDD_WriteChannelValueReg(TPM2_BASE_PTR,0,60000);
}
*/
bool RadioActive;


void setupPulses() {

  uint8_t required_protocol;
  required_protocol = g_model.protocol;
  if (PausePulses) {
    required_protocol = PROTO_NONE;
  }
    BIND_DONE;
    PausePulses = 1;
  if (Current_protocol != required_protocol) {
    Current_protocol = required_protocol;
    // switch mode here
    EnablePPMTim();
    DisableGIO();
    DisablePRTTim();
    while(RadioActive); 
    AFHDS2A_tel_status = 0;
    switch (required_protocol) {
    case PROTO_PPM:
      crsf_shutdown();
      A7105_Sleep();
      EnablePPMOut();
      break;
    case PROTO_AFHDS2A:
      DisablePPMOut();
       crsf_shutdown();
    
      initAFHDS2A();
      EnablePRTTim();
      break;
    case PROTO_AFHDS:
      DisablePPMOut();
       crsf_shutdown();
      initAFHDS();
      EnablePRTTim();
      break;
#ifdef PROTO_ELRS1
    case PROTO_ELRS1:
#endif
     case PROTO_ELRS2:
        A7105_Sleep();
        DisablePPMOut();
        crsf_init();
        EnablePRTTim();

        break;
    }


  }
  switch (required_protocol) {
  case PROTO_PPM:
    setupPulsesPPM(PROTO_PPM); // Don't enable interrupts through here
    break;
  }

  PausePulses = 0;
}
uint16_t B3_comp_value ;

void setupPulsesPPM( uint8_t proto )
{
#define PPM_TMR_FREQ 3

#define PPM_CENTER 1500 * PPM_TMR_FREQ
	int16_t PPM_range;

	uint8_t startChan = g_model.ppmStart;

	//Total frame length = 22.5msec
	//each pulse is 0.7..1.7ms long with a 0.3ms stop tail
	//The pulse ISR is 2mhz that's why everything is multiplied by 2
	uint16_t *ptr;
	ptr = (proto == PROTO_PPM) ?
			pulses2MHz.pword : &pulses2MHz.pword[PULSES_WORD_SIZE / 2];
	uint8_t p = (/*(proto == PROTO_PPM16) ? 16 :*/ 8) + g_model.ppmNCH * 2; //Channels *2
	p += startChan;
	uint16_t q = (g_model.ppmDelay * 50 + 300) * PPM_TMR_FREQ; //Stoplen *2
	uint16_t rest = (22500u * PPM_TMR_FREQ) - q; //Minimum Framelen=22.5 ms
	rest += (int16_t(g_model.ppmFrameLength)) *1500;
	//    if(p>9) rest=p*(1720u*2 + q) + 4000u*2; //for more than 9 channels, frame must be longer
//	if ( proto != PROTO_PPM )
//	{
	*ptr++ = q;
//	}
	PPM_range = g_model.extendedLimits ? 640 * PPM_TMR_FREQ : 512 * PPM_TMR_FREQ; //range of 0.7..1.7msec
	for (uint8_t i = /*(proto == PROTO_PPM16) ? p - 8 :*/ startChan; i < p; i++) { //NUM_CHNOUT
//    int16_t v = max(min(g_chans512[i],PPM_range),-PPM_range) + PPM_CENTER;
		int16_t v = g_chans512[i] * 3 / 2;
		if (v > PPM_range) {
			v = PPM_range;
		}
		if (v < -PPM_range) {
			v = -PPM_range;
		}
		v += PPM_CENTER;

		rest -= v; // (*ptr + q);
		*ptr++ = v - q; /* as Pat MacKenzie suggests */
		*ptr++ = q;      //to here
	}
	if (rest < 9000) {
		rest = 9000;
	}
	*ptr++ = rest;
	if (proto != PROTO_PPM) {
		B3_comp_value = rest - 1000;		// 500uS before end of sync pulse
	}
//	if ( proto == PROTO_PPM )
//	{
//		*ptr++ = q ;
//	}

	*ptr = 0;
}


void ISR_TIMER1_CAPT_vect(void) //2MHz pulse generation
{
/*
	//      static uint8_t  pulsePol;
	uint8_t x ;
	uint8_t y ;
	PORTB ^=  (1<<OUT_B_PPM);
	x = *Serial_pulsePtr;      // Byte size
	y = x & 0x0F ;
	if ( y == 15 )
	{
		y = 255 ;
	}
	else
	{
		y *= pass_bitlen ;
		y -= 1 ;
	}
	ICR1 = y ;
	x >>= 4 ;
	if ( x == 0 )
	{
	  Serial_pulsePtr += 1 ;
	}
	else
	{
		*Serial_pulsePtr = x ;
	}

	if ( y > 200 )
	{
#ifdef SBUS_PROTOCOL	
		if ( g_model.protocol == PROTO_SBUS )
		{
       PORTB &= ~(1<<OUT_B_PPM) ;
		}
		else
#endif
		{
       PORTB |=  (1<<OUT_B_PPM);      // Make sure pulses are the correct way up      
		}
	}
	heartbeat |= HEART_TIMER2Mhz;
*/
}


void ISR_TIMER1_COMPC_vect(void) // DSM2&MULTI or PXX end of frame
{
/*

//#ifdef MULTI_PROTOCOL
//#ifdef SBUS_PROTOCOL	
//		if ( (g_model.protocol == PROTO_DSM2) || (g_model.protocol == PROTO_MULTI) || (g_model.protocol == PROTO_SBUS) )
//#else
//		if ( (g_model.protocol == PROTO_DSM2) || (g_model.protocol == PROTO_MULTI) )
//#endif // SBUS_PROTOCOL
//#else
//#ifdef SBUS_PROTOCOL	
//		if ( (g_model.protocol == PROTO_DSM2) || (g_model.protocol == PROTO_SBUS) )
//#else
//    if (g_model.protocol == PROTO_DSM2)
//#endif // SBUS_PROTOCOL
//#endif // MULTI_PROTOCOL
	if ( g_model.protocol != PROTO_PXX)
	{
		// DSM2
		uint16_t t = 41536 ; //next frame starts in 22 msec 41536 = 2*(22000 - 14*11*8)
#ifdef SBUS_PROTOCOL
		if ( g_model.protocol == PROTO_SBUS)
		{
			t = 16000 ; //next frame starts in 11 msec 16000 = 2*(11000 - 25*12*10)
		}
#endif // SBUS_PROTOCOL
#ifdef MULTI_PROTOCOL
		if ( g_model.protocol == PROTO_MULTI)
			t = 15760 ; //next frame starts in 11 msec 15760 = 2*(11000 - 26*12*10)
#endif // MULTI_PROTOCOL
    ICR1 = t ; //next frame start time
	 	if (OCR1C<255)
			OCR1C = t-5000 ;  //delay setup pulses to reduce sytem latency
		else
  	{
       OCR1C=200;
       setupPulses();
 		}
  }
  else		// must be PXX
  {
    setupPulses() ;
  }
*/
}

// This interrupt for PXX

void ISR_TIMER1_COMPB_vect(void) // PXX main interrupt
{
/*
    uint8_t x ;
    PORTB ^= (1<<OUT_B_PPM) ;
    x = *pulses2MHzptr;      // Byte size
    if ( ( x & 1 ) == 0 )
    {
        OCR1B += 32 ;
    }
    else
    {
        OCR1B += 16 ;
    }
    if ( (x >>= 1) == 0 )
    {
        if ( *(++pulses2MHzptr) == 0 )
        {
//            OCR1B -= 48 ; // = OCR1C + 3000 ;		// 1.5mS on from OCR1C
						// disable COMPB interrupt
#ifdef CPUM2561
    TIMSK1 &= ~(1<<OCIE1B) ;	// COMPB interrupt off
#else
    TIMSK &= ~(1<<OCIE1B) ;	// COMPB interrupt off
#endif
        }
    }
    else
    {
        *pulses2MHzptr = x ;
    }
    heartbeat |= HEART_TIMER2Mhz;
*/
}



void set_timer3_capture()
{
/*
#ifdef CPUM2561
    TIMSK3 &= ~( (1<<OCIE3A) | (1<<OCIE3B) | (1<<OCIE3C) ) ;	// Stop compare interrupts
#else
    ETIMSK &= ~( (1<<OCIE3A) | (1<<OCIE3B) | (1<<OCIE3C) ) ;	// Stop compare interrupts
#endif
    DDRE &= ~0x80;  PORTE |= 0x80 ;	// Bit 7 input + pullup

    TCCR3B = 0 ;			// Stop counter
    TCCR3A  = 0;
    TCCR3B  = (1<<ICNC3) | (2<<CS30);      //ICNC3 16MHz / 8
#ifdef CPUM2561
    TIMSK3 |= (1<<ICIE3);
#else
    ETIMSK |= (1<<TICIE3);
#endif
*/
}

void set_timer3_ppm()
{
/*
#ifdef CPUM2561
    TIMSK3 &= ~( 1<<ICIE3) ;	// Stop capture interrupt
#else
    ETIMSK &= ~( 1<<TICIE3) ;	// Stop capture interrupt
#endif
    DDRE |= 0x80;					// Bit 7 output

    TCCR3B = 0 ;			// Stop counter
    TCCR3A = (0<<WGM10);
    TCCR3B = (1 << WGM12) | (2<<CS10); // CTC OCR1A, 16MHz / 8

#ifdef CPUM2561
    TIMSK3 |= ( (1<<OCIE3A) | (1<<OCIE3B) ); 			// enable immediately before mainloop
#else
    ETIMSK |= ( (1<<OCIE3A) | (1<<OCIE3B) ); 			// enable immediately before mainloop
#endif
*/
}



void ISR_TIMER3_COMPA_vect(void) //2MHz pulse generation
{
/*
    static uint16_t *pulsePtr = &pulses2MHz.pword[PULSES_WORD_SIZE/2];
    uint16_t *xpulsePtr ;

    if(PulsePol16)
    {
        PORTE |= 0x80 ; // (1<<OUT_B_PPM);
        PulsePol16 = 0;
    }else{
        PORTE &= ~0x80 ;		// (1<<OUT_B_PPM);
        PulsePol16 = 1;
    }

		xpulsePtr = pulsePtr ;	// read memory once

    OCR3A  = *xpulsePtr++;
    OCR3B = B3_comp_value ;
    
    if( *xpulsePtr == 0)
    {
        xpulsePtr = &pulses2MHz.pword[PULSES_WORD_SIZE/2];
        PulsePol16 = !g_model.pulsePol;//0;     // changed polarity
    }
		pulsePtr = xpulsePtr ;	// write memory back
    heartbeat |= HEART_TIMER2Mhz;
*/
}

void ISR_TIMER3_COMPB_vect(void) //2MHz pulse generation
{
	uint8_t proto = g_model.protocol ;
    sei() ;
    if ( Current_protocol != proto )
		{
 /*   	if ( Current_protocol == PROTO_PPMSIM )
			{
				if ( ( !SlaveMode ) || ( g_eeGeneral.enablePpmsim == 0 ) )
				{
        	setupPulses();
					return ;
				}
				else
				{
					proto = PROTO_PPMSIM ;
				}
			}
*/		}
    setupPulsesPPM(proto) ;
}



//const prog_uint16_t APM CRCTable[]=
//{
//    0x0000,0x1189,0x2312,0x329b,0x4624,0x57ad,0x6536,0x74bf,
//    0x8c48,0x9dc1,0xaf5a,0xbed3,0xca6c,0xdbe5,0xe97e,0xf8f7,
//    0x1081,0x0108,0x3393,0x221a,0x56a5,0x472c,0x75b7,0x643e,
//    0x9cc9,0x8d40,0xbfdb,0xae52,0xdaed,0xcb64,0xf9ff,0xe876,
//    0x2102,0x308b,0x0210,0x1399,0x6726,0x76af,0x4434,0x55bd,
//    0xad4a,0xbcc3,0x8e58,0x9fd1,0xeb6e,0xfae7,0xc87c,0xd9f5,
//    0x3183,0x200a,0x1291,0x0318,0x77a7,0x662e,0x54b5,0x453c,
//    0xbdcb,0xac42,0x9ed9,0x8f50,0xfbef,0xea66,0xd8fd,0xc974,
//    0x4204,0x538d,0x6116,0x709f,0x0420,0x15a9,0x2732,0x36bb,
//    0xce4c,0xdfc5,0xed5e,0xfcd7,0x8868,0x99e1,0xab7a,0xbaf3,
//    0x5285,0x430c,0x7197,0x601e,0x14a1,0x0528,0x37b3,0x263a,
//    0xdecd,0xcf44,0xfddf,0xec56,0x98e9,0x8960,0xbbfb,0xaa72,
//    0x6306,0x728f,0x4014,0x519d,0x2522,0x34ab,0x0630,0x17b9,
//    0xef4e,0xfec7,0xcc5c,0xddd5,0xa96a,0xb8e3,0x8a78,0x9bf1,
//    0x7387,0x620e,0x5095,0x411c,0x35a3,0x242a,0x16b1,0x0738,
//    0xffcf,0xee46,0xdcdd,0xcd54,0xb9eb,0xa862,0x9af9,0x8b70,
//    0x8408,0x9581,0xa71a,0xb693,0xc22c,0xd3a5,0xe13e,0xf0b7,
//    0x0840,0x19c9,0x2b52,0x3adb,0x4e64,0x5fed,0x6d76,0x7cff,
//    0x9489,0x8500,0xb79b,0xa612,0xd2ad,0xc324,0xf1bf,0xe036,
//    0x18c1,0x0948,0x3bd3,0x2a5a,0x5ee5,0x4f6c,0x7df7,0x6c7e,
//    0xa50a,0xb483,0x8618,0x9791,0xe32e,0xf2a7,0xc03c,0xd1b5,
//    0x2942,0x38cb,0x0a50,0x1bd9,0x6f66,0x7eef,0x4c74,0x5dfd,
//    0xb58b,0xa402,0x9699,0x8710,0xf3af,0xe226,0xd0bd,0xc134,
//    0x39c3,0x284a,0x1ad1,0x0b58,0x7fe7,0x6e6e,0x5cf5,0x4d7c,
//    0xc60c,0xd785,0xe51e,0xf497,0x8028,0x91a1,0xa33a,0xb2b3,
//    0x4a44,0x5bcd,0x6956,0x78df,0x0c60,0x1de9,0x2f72,0x3efb,
//    0xd68d,0xc704,0xf59f,0xe416,0x90a9,0x8120,0xb3bb,0xa232,
//    0x5ac5,0x4b4c,0x79d7,0x685e,0x1ce1,0x0d68,0x3ff3,0x2e7a,
//    0xe70e,0xf687,0xc41c,0xd595,0xa12a,0xb0a3,0x8238,0x93b1,
//    0x6b46,0x7acf,0x4854,0x59dd,0x2d62,0x3ceb,0x0e70,0x1ff9,
//    0xf78f,0xe606,0xd49d,0xc514,0xb1ab,0xa022,0x92b9,0x8330,
//    0x7bc7,0x6a4e,0x58d5,0x495c,0x3de3,0x2c6a,0x1ef1,0x0f78
//};




const prog_uint16_t APM CRC_Short[]=
{
   0x0000, 0x1189, 0x2312, 0x329B, 0x4624, 0x57AD, 0x6536, 0x74BF,
   0x8C48, 0x9DC1, 0xAF5A, 0xBED3, 0xCA6C, 0xDBE5, 0xE97E, 0xF8F7 };

static uint16_t CRCTable(uint8_t val)
{
	uint16_t word ;
	word = pgm_read_word(&CRC_Short[val&0x0F]) ;
	val /= 16 ;
	return word ^ (0x1081 * val) ;
}


static void crc( uint8_t data )
{
  PcmControl.PcmCrc=(PcmControl.PcmCrc<<8) ^ CRCTable((PcmControl.PcmCrc>>8)^data) ;
//  PcmControl.PcmCrc=(PcmControl.PcmCrc<<8)^     pgm_read_word(&CRCTable[((uint8_t)(PcmControl.PcmCrc>>8)^data) & 0xFF]);
}




void putPcmPart( uint8_t value )
{
		struct t_pcm_control *ptrControl ;
		uint8_t x ;

		ptrControl = &PcmControl ;
		FORCE_INDIRECT(ptrControl) ;
    
		x = ptrControl->PcmByte ;
		x /= 4 ;	// Better code than a shift !!!!
		if ( ptrControl->Shift )
		{
			x >>= 2 ;
		}
    x |= value ;
		ptrControl->PcmByte = x ;
    if ( ++ptrControl->PcmBitCount >= ptrControl->Limit )
    {
        *ptrControl->PcmPtr++ = ptrControl->PcmByte ;
        ptrControl->PcmBitCount = ptrControl->PcmByte = 0 ;
    }
}


static void putPcmFlush()
{
  while ( PcmControl.PcmBitCount != 0 )
  {
  	putPcmPart( 0 ) ; // Empty
  }
  *PcmControl.PcmPtr = 0 ;				// Mark end
	asm("") ;
}

void putPcmBit( uint8_t bit )
{
    if ( bit )
    {
        PcmControl.PcmOnesCount += 1 ;
        putPcmPart( 0x80 ) ;
    }
    else
    {
        PcmControl.PcmOnesCount = 0 ;
        putPcmPart( 0xC0 ) ;
    }
    if ( PcmControl.PcmOnesCount >= 5 )
    {
        putPcmBit( 0 ) ;				// Stuff a 0 bit in
    }
}

void putPcmByte( uint8_t byte )
{
    uint8_t i ;

    crc( byte ) ;

    for ( i = 0 ; i < 8 ; i += 1 )
    {
        putPcmBit( byte & 0x80 ) ;
        byte <<= 1 ;
    }
}

void putPcmHead()
{
	uint8_t i ;
    // send 7E, do not CRC
    // 01111110
    putPcmPart( 0xC0 ) ;
		for ( i = 0 ; i < 6 ; i += 1 )
		{
    	putPcmPart( 0x80 ) ;
		}
    putPcmPart( 0xC0 ) ;
}

uint16_t scaleForPXX( uint8_t i )
{
/*
	int16_t value ;
	
	value = ( ( i < 16 ) ? g_chans512[i] *3 / 4 : 0 ) + 1024 ;
	return limit( 1, value, 2046 ) ;
*/
return 0;
}


static void setupPulsesPXX()
{
/*
    uint8_t i ;
    uint16_t chan ;
    uint16_t chan_1 ;
		uint8_t lpass = pass_bitlen ;

#ifdef CPUM2561
    TIMSK1 &= ~( (1<<OCIE1B) | (1<<OCIE1C) ) ;	// COMPC & B interrupts off
#else
    TIMSK &= ~(1<<OCIE1B) ;	// COMPB interrupt off
    ETIMSK &= ~(1<<OCIE1C) ;	// COMPC interrupt off
#endif

		{
			struct t_pcm_control *ptrControl ;

			ptrControl = &PcmControl ;
			FORCE_INDIRECT(ptrControl) ;
    
			pulses2MHzptr = pulses2MHz.pbyte ;
    	ptrControl->PcmPtr = pulses2MHz.pbyte + 1 ;		// past preamble
    	ptrControl->PcmCrc = 0 ;
    	ptrControl->PcmBitCount = ptrControl->PcmByte = 0 ;
    	ptrControl->PcmOnesCount = 0 ;
			ptrControl->Shift = 0 ;
			ptrControl->Limit = 4 ;
		}

			pulses2MHz.pbyte[0] = 0xFF ;		// Preamble
    putPcmHead(  ) ;  // sync byte
    putPcmByte( g_model.ppmNCH ) ;     // putPcmByte( g_model.rxnum ) ;  //

  uint8_t flag1;
  if (pxxFlag & PXX_BIND)
	{
    flag1 = (g_model.sub_protocol<< 6) | (g_model.country << 1) | pxxFlag ;
  }
  else
	{
    flag1 = (g_model.sub_protocol << 6) | pxxFlag ;
	}
#ifdef FAILSAFE
	if ( ( flag1 & (PXX_BIND | PXX_RANGE_CHECK )) == 0 )
	{
  	if (g_model.failsafeMode != FAILSAFE_NOT_SET && g_model.failsafeMode != FAILSAFE_RX )
		{
    	if ( FailsafeCounter )
			{
	    	if ( FailsafeCounter-- == 1 )
				{
    	  	flag1 |= PXX_SEND_FAILSAFE ;
				}
    		if ( ( FailsafeCounter == 1 ) && (g_model.sub_protocol == 0 ) )
				{
  	    	flag1 |= PXX_SEND_FAILSAFE ;
				}
			}
	    if ( FailsafeCounter == 0 )
			{
				if ( g_model.failsafeRepeat == 0 )
				{
					FailsafeCounter = 1000 ;
				}
			}
		}
	}
#endif		 
	 putPcmByte( flag1 ) ;     // First byte of flags

		putPcmByte( 0 ) ;     // Second byte of flags
		
		uint8_t startChan = g_model.ppmStart ;
		if ( lpass & 1 )
		{
			startChan += 8 ;			
		}
#ifdef FAILSAFE
		chan = 0 ;
    for ( i = 0 ; i < 8 ; i += 1 )		// First 8 channels only
		{
    	if (flag1 & PXX_SEND_FAILSAFE)
			{
				if ( g_model.failsafeMode == FAILSAFE_HOLD )
				{
					chan_1 = 2047 ;
				}
				else if ( g_model.failsafeMode == FAILSAFE_NO_PULSES )
				{
					chan_1 = 0 ;
				}
				else
				{
					// Send failsafe value
					int32_t value ;
#ifdef V2
					value = ( startChan < 16 ) ? g_model.Failsafe[startChan] : 0 ;
#else
					value = ( startChan < 8 ) ? g_model.Failsafe[startChan] : ( ( startChan < 16 ) ? g_model.XFailsafe[startChan-8] : 0 ) ;
#endif
					value = ( value *3933 ) >> 9 ;
					value += 1024 ;					
					chan_1 = limit( (int16_t)1, (int16_t)value, (int16_t)2046 ) ;
				}
			}
			else
			{
	     	chan_1 = scaleForPXX( startChan ) ;
			}
			if ( lpass & 1 )
			{
				chan_1 += 2048 ;
			}
			startChan += 1 ;
			if ( i & 1 )
			{
				putPcmByte( chan ) ; // Low byte of channel
				putPcmByte( ( ( chan >> 8 ) & 0x0F ) | ( chan_1 << 4) ) ;  // 4 bits each from 2 channels
      	putPcmByte( chan_1 >> 4 ) ;  // High byte of channel
			}
			else
			{
				chan = chan_1 ;
			}
		}
#else
    for ( i = 0 ; i < 4 ; i += 1 )		// First 8 channels only
    {																	// Next 8 channels would have 2048 added
      chan = scaleForPXX( startChan ) ;
			if ( lpass & 1 )
			{
				chan += 2048 ;
			}
      putPcmByte( chan ) ; // Low byte of channel
			startChan += 1 ;
      chan_1 = scaleForPXX( startChan ) ;
			if ( lpass & 1 )
			{
				chan_1 += 2048 ;
			}
			startChan += 1 ;
			putPcmByte( ( ( chan >> 8 ) & 0x0F ) | ( chan_1 << 4) ) ;  // 4 bits each from 2 channels
      putPcmByte( chan_1 >> 4 ) ;  // High byte of channel
    }
#endif
#if defined(CPUM128) || defined(CPUM2561) || defined(V2)
#if defined(CPUM128) || defined(CPUM2561)
		flag1 = 0 ;
		if ( PxxExtra & 1 )
		{
			flag1 = (1 << 2 ) ;
		}
		if ( PxxExtra & 2 )
		{
			flag1 |= (1 << 1 ) ;
		}
#else
		flag1 = 0 ;
#endif		
		if ( g_model.sub_protocol == 3 )	// R9M
		{
			flag1 |= g_model.r9mPower << 3 ;
		}
		putPcmByte( flag1 ) ;	// Extra flags
#else
		putPcmByte( 0 ) ;	// Extra flags
#endif
    chan = PcmControl.PcmCrc ;		        // get the crc
    putPcmByte( chan >> 8 ) ; // Checksum hi
    putPcmByte( chan ) ; 			// Checksum lo
    putPcmHead(  ) ;      // sync byte
    putPcmFlush() ;
		volatile uint16_t *ptr = &OCR1C ;
		FORCE_INDIRECT( ptr ) ;
		{
			uint16_t ocrc ;
			ocrc = *ptr ;
    	OCR1B = ocrc + 3000 ;		// 1.5mS on from OCR1C
			*ptr = ocrc + 18000 ;		// 18mS on, 9mS needed if 16 channels
		}
//    OCR1B = *ptr + 3000 ;		// 1.5mS on from OCR1C
//    *ptr += 18000 ;		// 18mS on, 9mS needed if 16 channels
    PORTB |= (1<<OUT_B_PPM);		// Idle is line high
#ifdef CPUM2561
    TIFR1 = (1<<OCF1B) | (1<<OCF1C) ;			// Clear pending interrupts
    TIMSK1 |= (1<<OCIE1B) | (1<<OCIE1C);	// Enable COMPB and COMPC
#else
    TIFR = (1<<OCF1B) ;			// Clear pending interrupt
    ETIFR = (1<<OCF1C) ;			// Clear pending interrupt
    TIMSK |= (1<<OCIE1B) ;	// Enable COMPB
    ETIMSK |= (1<<OCIE1C);	// Enable COMPC
#endif
		if (g_model.sub_protocol == 1 )		// D8
		{
			lpass = 0 ;
		}
		else
		{
			lpass += 1 ;
//#if defined(CPUM128) || defined(CPUM2561)
			if ( g_model.pxxChans == 1 )
			{
				lpass = 0 ;
			}
//#endif
		}
		pass_bitlen = lpass ;
//		PxxTime = TCNT1 - PxxStart ;
	asm("") ;
*/
}


// DSM2 protocol pulled from th9x - Thanks thus!!!

//http://www.rclineforum.de/forum/board49-zubeh-r-elektronik-usw/fernsteuerungen-sender-und-emp/neuer-9-kanal-sender-f-r-70-au/Beitrag_3897736#post3897736
//(dsm2( LP4DSM aus den RTF ( Ready To Fly ) Sendern von Spektrum.
//http://www.rcgroups.com/forums/showpost.php?p=18554028&postcount=237
// /home/thus/txt/flieger/PPMtoDSM.c
/*
  125000 Baud 8n1      _ xxxx xxxx - ---
#define DSM2_CHANNELS      6                // Max number of DSM2 Channels transmitted
#define DSM2_BIT (8*2)
bind:
  DSM2_Header = 0x80,0
static byte DSM2_Channel[DSM2_CHANNELS*2] = {
                ch
  0x00,0xAA,     0 0aa
  0x05,0xFF,     1 1ff
  0x09,0xFF,     2 1ff
  0x0D,0xFF,     3 1ff
  0x13,0x54,     4 354
  0x14,0xAA      5 0aa
};

normal:
  DSM2_Header = 0,0;
  DSM2_Channel[i*2]   = (byte)(i<<2) | highByte(pulse);
  DSM2_Channel[i*2+1] = lowByte(pulse);


 */

//static inline void _send_1(uint16_t v)
//{
//	uint8_t *ptr ;
//	ptr = pulses2MHzptr ;
//	*ptr++ = v ;
//  pulses2MHzptr = ptr ;
//}

// MULTI protocol definition
/*
Serial: 100000 Baud 8e2      _ xxxx xxxx p --
  Total of 26 bytes
  Stream[0]   = 0x55
   header
  Stream[1]   = sub_protocol|BindBit|RangeCheckBit|AutoBindBit;
   sub_protocol is 0..31 (bits 0..4)
   =>	Reserved	0
					Flysky		1
					Hubsan		2
					Frsky		3
					Hisky		4
					V2x2		5
					DSM2		6
					Devo		7
					YD717		8
					KN			9
					SymaX		10
					SLT			11
					CX10		12
					CG023		13
					Bayang		14
					FrskyX		15
					ESky		16
					MT99XX		17
					MJXQ		18
   BindBit=>		0x80	1=Bind/0=No
   AutoBindBit=>	0x40	1=Yes /0=No
   RangeCheck=>		0x20	1=Yes /0=No
  Stream[2]   = RxNum | Power | Type;
   RxNum value is 0..15 (bits 0..3)
   Type is 0..7 <<4     (bit 4..6)
		sub_protocol==Flysky
			Flysky	0
			V9x9	1
			V6x6	2
			V912	3
		sub_protocol==Hisky
			Hisky	0
			HK310	1
		sub_protocol==DSM2
			DSM2	0
			DSMX	1
		sub_protocol==YD717
			YD717	0
			SKYWLKR	1
			SYMAX4	2
			XINXUN	3
			NIHUI	4
		sub_protocol==KN
			WLTOYS	0
			FEILUN	1
		sub_protocol==SYMAX
			SYMAX	0
			SYMAX5C	1
		sub_protocol==CX10
			CX10_GREEN	0
			CX10_BLUE	1	// also compatible with CX10-A, CX12
			DM007		2
			Q282		3
			JC3015_1	4
			JC3015_2	5
			MK33041		6
			Q242		7
		sub_protocol==CG023
			CG023		0
			YD829		1
			H8_3D		2
		sub_protocol==MT99XX
			MT99		0
			H7			1
			YZ			2
		sub_protocol==MJXQ
			WLH08		0
			X600		1
			X800		2
			H26D		3
   Power value => 0x80	0=High/1=Low
  Stream[3]   = option_protocol;
   option_protocol value is -127..127
  Stream[4] to [25] = Channels
   16 Channels on 11 bits (0..2047)
	0		-125%
    204		-100%
	1024	   0%
	1843	+100%
	2047	+125%
   Channels bits are concatenated to fit in 22 bytes like in SBUS protocol
*/

//static void putSerialPart( uint8_t value )
//{
//	struct t_pcm_control *ptrControl ;

//	ptrControl = &PcmControl ;
//	FORCE_INDIRECT(ptrControl) ;

//	ptrControl->PcmByte >>= 4 ;
//  ptrControl->PcmByte |= value ;
//  if ( ++ptrControl->PcmBitCount >= 2 )
//  {
//    *ptrControl->PcmPtr++ = ptrControl->PcmByte ;
//    ptrControl->PcmBitCount = ptrControl->PcmByte = 0 ;
//  }
//}

static void sendByteSerial(uint8_t b) //max 10changes 0 10 10 10 10 1
{
/*
    bool    lev = 0;
	uint8_t parity = 0x80 ;
	uint8_t count = 8 ;
#if defined(SBUS_PROTOCOL) || defined(MULTI_PROTOCOL)
	uint8_t bitLen ;
	if ( g_model.protocol != PROTO_DSM2 )
	{ // SBUS & MULTI
		parity = 0 ;
		bitLen = b ;
		for( uint8_t i=0; i<8; i++)
		{
			parity += bitLen & 0x80 ;
			bitLen <<= 1 ;
		}
		parity &= 0x80 ;
		count = 9 ;
	}
#endif // SBUS_PROTOCOL & MULTI_PROTOCOL
    uint8_t len = 0x10 ; // 1 << 4

	for( uint8_t i=0; i<=count; i++)
	{ //8Bits + Stop=1
		bool nlev = b & 1; //lsb first
		if(lev == nlev)
		{
			len += 0x10 ;
		}
		else
		{
			putPcmPart( len ) ;
			len = 0x10 ;
			lev = nlev ;
		}
		b = (b>>1) | parity ; //shift in parity or stop bit
		parity = 0x80 ;				// Now a stop bit
	}
	putPcmPart( len + 0x10 ) ; // 2 stop bits
*/
}

void setupPulsesSerial(void)
{
/*
	struct t_pcm_control *ptrControl ;
#ifdef MULTI_PROTOCOL
	uint8_t packetType ;
	packetType = ( ( (g_model.sub_protocol+1) & 0x3F) > 31 ) ? 0x54 : 0x55 ;
#endif

	ptrControl = &PcmControl ;
	FORCE_INDIRECT(ptrControl) ;
    
	ptrControl->PcmPtr = pulses2MHz.pbyte ;
 	ptrControl->PcmBitCount = ptrControl->PcmByte = 0 ;
	ptrControl->PcmCrc = 0 ;
	ptrControl->Shift = 1 ;
	ptrControl->Limit = 2 ;

//    uint8_t counter ;
	uint8_t serialdat0copy;
	//	CSwData &cs = g_model.customSw[NUM_CSW-1];

//    pulses2MHzptr = pulses2MHz.pbyte ;
    
    // If more channels needed make sure the pulses union/array is large enough

	serialdat0copy = serialDat0 ;		// Fetch byte once, saves flash
#if defined(SBUS_PROTOCOL) || defined(MULTI_PROTOCOL)
	uint8_t protocol = g_model.protocol ;
	if( protocol == PROTO_DSM2)
	{
#endif // SBUS_PROTOCOL & MULTI_PROTOCOL
		pass_bitlen = BITLEN_SERIAL ;
    if (serialdat0copy&BadData)  //first time through, setup header
    {
			if ( g_model.sub_protocol == LPXDSM2 )
			{
				serialdat0copy= 0x80;
			}
			else if ( g_model.sub_protocol == DSM2only )
			{
				serialdat0copy= 0x90;
			}
			else
			{
				serialdat0copy=0x98;  //dsmx, bind mode
			}
    }
    if((serialdat0copy&BindBit)&&(!keyState(SW_Trainer)))  serialdat0copy&=~BindBit;		//clear bind bit if trainer not pulled
    if ((!(serialdat0copy&BindBit))&&getSwitch00(MAX_DRSWITCH-1)) serialdat0copy|=RangeCheckBit;	//range check function
    	else serialdat0copy&=~RangeCheckBit;
		sendByteSerial(serialdat0copy);
		serialDat0 = serialdat0copy ;		// Put byte back
		sendByteSerial(g_model.ppmNCH);
		for(uint8_t i=0; i<6; i++)
		{
			uint16_t pulse = limit(0, ((g_chans512[i]*13)>>5)+512,1023) ;
			sendByteSerial((i<<2) | ((pulse>>8)&0x03));
			sendByteSerial(pulse & 0xff);
		}
#if defined(SBUS_PROTOCOL) || defined(MULTI_PROTOCOL)
	}
	else
	{ // SBUS & MULTI
		pass_bitlen = BITLEN_SBUS ;
		uint8_t outputbitsavailable = 0 ;
		uint32_t outputbits = 0 ;
		uint8_t i ;
#ifdef MULTI_PROTOCOL
		if ( protocol == PROTO_SBUS )
#endif // MULTI_PROTOCOL
			sendByteSerial(0x0F) ;
#ifdef MULTI_PROTOCOL
		else
		{
#ifdef FAILSAFE
  		if (g_model.failsafeMode != FAILSAFE_NOT_SET && g_model.failsafeMode != FAILSAFE_RX )
			{
  		  if ( FailsafeCounter )
				{
			    if ( FailsafeCounter-- == 1 )
					{
						packetType += 2 ;	// Failsafe packet
					}
				}
			  if ( FailsafeCounter == 0 )
				{
					if ( g_model.failsafeRepeat == 0 )
					{
						FailsafeCounter = 1000 ;
					}
				}
			}
#endif			
			sendByteSerial( packetType ) ;
			serialdat0copy= (g_model.sub_protocol+1) & 0x5F;
			if (pxxFlag & PXX_BIND)			serialdat0copy|=BindBit;		//set bind bit if BIND menu is pressed
			if (pxxFlag & PXX_RANGE_CHECK)	serialdat0copy|=RangeCheckBit;	//set range bit if RANGE menu is pressed
			sendByteSerial(serialdat0copy);
			serialDat0 = serialdat0copy ;		// Put byte back
			sendByteSerial(g_model.ppmNCH);
			sendByteSerial(g_model.option_protocol);
		}
#endif // MULTI_PROTOCOL
		
		uint8_t startChan = g_model.ppmStart ;

		for ( i = 0 ; i < 16 ; i += 1 )
		{
			int16_t x = g_chans512[startChan] ;
#ifdef FAILSAFE			
			if ( packetType & 2 )
			{
				if ( g_model.failsafeMode == FAILSAFE_HOLD )
				{
					x = 2047 ;
				}
				else if ( g_model.failsafeMode == FAILSAFE_NO_PULSES )
				{
					x = 0 ;
				}
				else
				{
					// Send failsafe value
					int16_t value ;
#ifdef V2
					value = ( startChan < 16 ) ? g_model.Failsafe[startChan] : 0 ;
#else
					value = ( startChan < 8 ) ? g_model.Failsafe[startChan] : ( ( startChan < 16 ) ? g_model.XFailsafe[startChan-8] : 0 ) ;
#endif
					x = value * 8 ;
					x += value >> 3 ;
					x += 1024 ;
					x = limit( (int16_t)1, (int16_t)x, (int16_t)2046 ) ;
				}
			}
			else
#endif			
			{			
				x *= 4 ;
				x += x > 0 ? 4 : -4 ;
				x /= 5 ;
#ifdef MULTI_PROTOCOL
			if ( protocol == PROTO_MULTI )
				x += 0x400 ;
			else
#endif // MULTI_PROTOCOL
				x += 0x3E0 ;
			}
			startChan += 1 ;
			if ( x < 0 )
			{
				x = 0 ;
			}
			if ( x > 2047 )
			{
				x = 2047 ;
			}
			outputbits |= (uint32_t)x << outputbitsavailable ;
			outputbitsavailable += 11 ;
			while ( outputbitsavailable >= 8 )
			{
				uint8_t j = outputbits ;
				sendByteSerial(j) ;
				outputbits >>= 8 ;
				outputbitsavailable -= 8 ;
			}
		}
#ifdef MULTI_PROTOCOL
		if ( protocol == PROTO_SBUS )
#endif // MULTI_PROTOCOL
		{
			sendByteSerial(0);
			sendByteSerial(0);
		}
	}
#endif // SBUS_PROTOCOL & MULTI_PROTOCOL
		
  if ( ptrControl->PcmBitCount )
	{
		*ptrControl->PcmPtr = 0x0F ;
	}
	else
	{
		*(ptrControl->PcmPtr - 1) |= 0xF0 ;
	}
	Serial_pulsePtr = pulses2MHz.pbyte ;
*/
}