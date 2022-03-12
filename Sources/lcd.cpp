/*
 * lcd.cpp
 *
 *  Created on: 31  2019 .
 *      Author: KOSTYA
 */
#include "lcd.h"
#include <stdint.h>
#include <stdlib.h>
#include "er9x.h"
volatile uint8_t LcdLock ;

#define DBL_FONT_SMALL	1

uint8_t Lcd_lastPos;
uint8_t DisplayBuf[DISPLAY_W*DISPLAY_H/8];
#define DISPLAY_END (DisplayBuf+sizeof(DisplayBuf))

const static uint8_t _bitmask[]= { 1,2,4,8,16,32,64,128 } ;
#define XBITMASK(bit)  *( _bitmask + bit )


const static unsigned char Lcdinit[] =
{
		  0xE2, //Initialize the internal functions
		  0xAE, //DON = 0: display OFF
		  0xA1, //ADC = 1: reverse direction(SEG132->SEG1)
		  0xA6, //REV = 0: non-reverse display
		  0xA4, //EON = 0: normal display. non-entire
		  0xA2, //Select LCD bias=0
		  0xC0, //SHL = 0: normal direction (COM1->COM64)
		  0x2F, //Control power circuit operation VC=VR=VF=1
		  0x25, //Select int resistance ratio R2 R1 R0 =5
		  0x81, //Set reference voltage Mode
		  0x22, //24 SV5 SV4 SV3 SV2 SV1 SV0 = 0x18
		  0xAF  //DON = 1: display ON
} ;
const static unsigned char font[] = {
//#include "font.lbm"
#include "font_05x07K.lbm"
};



uint8_t EepromActive ;
#define font_5x8_x20_x7f (font)
#ifdef SMALL_DBL
#include "font12x8test.lbm"
#define font_10x16_x20_x7f (font_12x8)
#else
const unsigned char font_dblsize[] = {
#include "font_dblsize.lbm"
};
#define font_10x16_x20_x7f (font_dblsize)
#endif // SMALL_DBL

static void lcdSendData(uint8_t val) {
  LCD_RS_1;
  LCD_DATA(val);
  LCD_RD_1;
  asm("nop");
  asm("nop");
  asm("nop");
  asm("nop");
  LCD_RD_0;
}
/*---------------------------------------------------------------------------*/
static void lcdSendCtl(uint8_t val) {
  LCD_RS_0;
  LCD_DATA(val);
  LCD_RD_1;
  asm("nop");
  asm("nop");
  asm("nop");
  asm("nop");
  LCD_RD_0;
}
/*---------------------------------------------------------------------------*/
void lcdSetRefVolt(uint8_t val) {
  LcdLock = 1;    // Lock LCD data lines
  lcdSendCtl(0x81);
  lcdSendCtl(val);
  LcdLock = 0;    // Free LCD data lines
}
/*---------------------------------------------------------------------------*/
void lcdSetContrast() {
  lcdSetRefVolt(g_eeGeneral.contrast);
}
/*---------------------------------------------------------------------------*/
void lcd_clear() {
  memset(DisplayBuf, 0, sizeof(DisplayBuf));
}
/*---------------------------------------------------------------------------*/
void lcd_init() {
  LcdLock = 1;    // Lock LCD data lines
  LCD_RST_0;
  mDelay(1);
  LCD_RST_1;
  mDelay(2);
  LCD_RW_0;
  LCD_RD_0;
  LCD_CS_0;
  LCD_RS_1;
  for (uint8_t i = 0; i < sizeof(Lcdinit); i++) {
    lcdSendCtl(Lcdinit[i]);
  }
  lcdSetContrast();
  //  LcdLock = 0 ;            // Free LCD data lines
  lcd_clear();
}
/*---------------------------------------------------------------------------*/
static uint8_t *dispBufAddress(uint8_t x, uint8_t y) {
  return &DisplayBuf[(y & 0xF8) * 16 + x];
}
/*---------------------------------------------------------------------------*/

uint8_t lcd_putcAtt(uint8_t x, uint8_t y, const char d, uint8_t mode) {
  uint8_t i;
  uint8_t c = d;
  uint8_t *p = dispBufAddress(x, y);

  //#if (DISPLAY_W==128)
  //  uint8_t *p  = &DisplayBuf[ (y & 0xF8) * 16 + x ];
  //#else
  //	uint8_t *p  = &DisplayBuf[ y / 8 * DISPLAY_W + x ];
  //#endif
  //uint8_t *pmax = &DisplayBuf[ DISPLAY_H/8 * DISPLAY_W ];
  if (c < 22) // Move to specific x position (c)*FW
  {
    x = c * FW;
    //  		if(mode&DBLSIZE)
    //			{
    //				x += x ;
    //			}
    return x;
  }
  x += FW;
  const prog_uchar *q = &font_5x8_x20_x7f[((unsigned char)c - 0x20) * 5];
  bool inv = (mode & INVERS) ? true : (mode & BLINK ? BLINK_ON_PHASE : false);
  if (mode & DBLSIZE) {
#ifdef SMALL_DBL
    if ((c != 0x2E))
      x += 2; //check for decimal point
#else
    if (c != 0x2E)
      x += FW; //check for decimal point
#endif
    /* each letter consists of ten top bytes followed by
	 * five bottom by ten bottom bytes (20 bytes per
	 * char) */
    unsigned char c_mapped;

#ifdef DBL_FONT_SMALL
    if (c >= ',' && c <= ':') {
      c_mapped = c - ',' + 1;
    } else if (c >= 'A' && c <= 'Z') {
      c_mapped = c - 'A' + 0x10;
    } else if (c >= 'a' && c <= 'z') {
      c_mapped = c - 'a' + 0x2B;
    } else if (c == '_') {
      c_mapped = 0x2A;
    } else {
      c_mapped = 0;
    }
#else
    c_mapped = c - 0x20;
#endif
#ifdef SMALL_DBL
    q = &font_10x16_x20_x7f[(c_mapped)*14]; // + ((c-0x20)/16)*160];
#ifdef DBL_FONT_SMALL
#if defined(CPUM128) || defined(CPUM2561)
    if ((c_mapped == ('i' - 'a' + 0x2B)) || (c_mapped == ('l' - 'a' + 0x2B))) {
      p -= 1;
      x -= 2;
    }
#endif
#endif
    for (char i = 7; i >= 0; i--) {
      uint8_t b1;
      uint8_t b3;
      b1 = pgm_read_byte(q);
      b3 = pgm_read_byte(q + 7);
      if (i == 0) {
        b1 = 0;
        b3 = 0;
      }
#else
    q = &font_10x16_x20_x7f[(c_mapped)*20]; // + ((c-0x20)/16)*160];
    for (int8_t i = 11; i >= 0; i--) {
      /*top byte*/
      uint8_t b1 = i > 1 ? pgm_read_byte(q) : 0;
      /*bottom byte*/
      uint8_t b3 = i > 1 ? pgm_read_byte(10 + q) : 0;
      /*top byte*/
      //            uint8_t b2 = i>0 ? pgm_read_byte(++q) : 0;
      /*bottom byte*/
//            uint8_t b4 = i>0 ? pgm_read_byte(10+q) : 0;
#endif // SMALL_DBL
      q++;
      if (inv) {
        b1 = ~b1;
        //                b2=~b2;
        b3 = ~b3;
        //                b4=~b4;
      }

      if (p < DISPLAY_END - (DISPLAY_W + 1)) {
        p[0] = b1;
        //                p[1]=b2;
        p[DISPLAY_W] = b3;
        //                p[DISPLAY_W+1] = b4;
        p += 1;
      }
    }
    //        q = &dbl_font[(c-0x20)*20];
    //        for(char i=0; i<10; i++){
    //            uint8_t b = pgm_read_byte(q++);
    //            if((p+DISPLAY_W)<DISPLAY_END) *(p+DISPLAY_W) = inv ? ~b : b;
    //            b = pgm_read_byte(q++);
    //            if(p<DISPLAY_END) *p = inv ? ~b : b;
    //            p++;
    //        }
    //        if(p<DISPLAY_END) *p = inv ? ~0 : 0;
    //        if((p+DISPLAY_W)<DISPLAY_END) *(p+DISPLAY_W) = inv ? ~0 : 0;
  } else {
    uint8_t condense = 0;

    if (mode & CONDENSED) {
      *p = inv ? ~0 : 0;
      p += 1;
      condense = 1;
      x += FWNUM - FW;
    }

#if defined(CPUM128) || defined(CPUM2561)
    y &= 7;
    if (y) { // off grid
      for (i = 5; i != 0; i--) {
        uint16_t b = pgm_read_byte(q++);
        b <<= y;
        if (p < DISPLAY_END)
          *p ^= b;
        if (&p[DISPLAY_W] < DISPLAY_END) {
          p[DISPLAY_W] ^= b >> 8;
        }
        p += 1;
      }
    } else
#endif
    {
      for (i = 5; i != 0; i--) {
        uint8_t b = pgm_read_byte(q++);
        if (condense && i == 4) {
          /*condense the letter by skipping column 4 */
          continue;
        }
        if (p < DISPLAY_END)
          *p = inv ? ~b : b;
        p += 1;
      }
      if (p < DISPLAY_END)
        *p++ = inv ? ~0 : 0;
    }
  }
  return x;
}
/*---------------------------------------------------------------------------*/
unsigned char lcd_putsAtt(unsigned char x, unsigned char y, const char *s, unsigned char mode) {
  uint8_t source;
  source = mode & BSS;
  while (1) {
    char c = (source) ? *s++ : *(s++);
    if (!c)
      break;
    if (c == 31) {
      if ((y += FH) >= DISPLAY_H) // Screen height
      {
        break;
      }
      x = 0;
    } else {
      x = lcd_putcAtt(x, y, c, mode);
    }
  }
  return x;
}
/*---------------------------------------------------------------------------*/
	void lcd_putsnAtt(unsigned char x,unsigned char y,const char * s,unsigned char len,unsigned char mode)
	{
		uint8_t source ;
		source = mode & BSS ;
	  while(len!=0) {
	    char c = (source) ? *s++ : *(s++);
			if ( c == 0 )
			{
				return ;
			}
	    x = lcd_putcAtt(x,y,c,mode);
	    len--;
	  }
	}
/*---------------------------------------------------------------------------*/

	// Puts sub-string from string options
	// First byte of string is sub-string length
	// idx is index into string (in length units)
	// Output length characters
	void lcd_putsAttIdx(unsigned char x,unsigned char y,const char * s,unsigned char idx,unsigned char att)
	{
		uint8_t length ;
		length = *(s++) ;

	  lcd_putsnAtt(x,y,s+length*idx,length,att) ;
	}
/*---------------------------------------------------------------------------*/
	uint8_t lcd_putc(uint8_t x,uint8_t y,const char c )
	{
	  return lcd_putcAtt(x,y,c,0);
	}
/*---------------------------------------------------------------------------*/
	void lcd_puts_Pleft(uint8_t y,const char * s)
	{
	  lcd_putsAtt( 0, y, s, 0);
	}
/*---------------------------------------------------------------------------*/
	// This routine skips 'skip' strings, then displays the rest
	void lcd_puts_Pskip(uint8_t y,const char * s, uint8_t skip)
	{
		while ( skip )
		{
	    char c = *(s++);
	    if(!c) return ;
			if ( c == 31 )
			{
				skip -= 1 ;
			}
		}
	  lcd_putsAtt( 0, y, s, 0);
	}
/*---------------------------------------------------------------------------*/
	void lcd_puts_P(uint8_t x,uint8_t y,const char * s)
	{
	  lcd_putsAtt( x, y, s, 0);
	}
/*---------------------------------------------------------------------------*/
	void lcd_putsn_P(uint8_t x,uint8_t y,const char * s,uint8_t len)
	{
	  lcd_putsnAtt( x,y,s,len,0);
	}
/*---------------------------------------------------------------------------*/
	void lcd_outhex4(uint8_t x,uint8_t y,uint16_t val)
	{
		uint8_t i ;
	  x+=FWNUM*4;
	  for(i=0; i<4; i++)
	  {
	    x-=FWNUM;
	    char c = val & 0xf;
	    c = c>9 ? c+'A'-10 : c+'0';
	    lcd_putcAtt(x,y,c,c>='A'?CONDENSED:0);
	    val>>=4;
	  }
	}
/*---------------------------------------------------------------------------*/
        void lcd_write_bits(uint8_t *p, uint8_t mask) {
          if (p < DISPLAY_END) {
            uint8_t temp = *p;
            if (plotType != PLOT_XOR) {
              temp |= mask;
            }
            if (plotType != PLOT_BLACK) {
              temp ^= mask;
            }
            *p = temp;
          }
        }
/*---------------------------------------------------------------------------*/
	void lcd_plot(uint8_t x,uint8_t y)
	{
		uint8_t *p = dispBufAddress( x, y ) ;
		lcd_write_bits( p, XBITMASK(y%8) ) ;
	}
/*---------------------------------------------------------------------------*/
	void lcd_vline(uint8_t x,uint8_t y, int8_t h)
	{
	//    while ((y+h)>=DISPLAY_H) h--;
	  if (h<0) { y+=h; h=-h; }
		uint8_t *p = dispBufAddress( x, y ) ;
	//#if (DISPLAY_W==128)
	//  uint8_t *p  = &DisplayBuf[ (y & 0xF8) * 16 + x ];
	//#else
	//	uint8_t *p  = &DisplayBuf[ y / 8 * DISPLAY_W + x ];
	//#endif
	  y &= 0x07 ;
		if ( y )
		{
	    uint8_t msk = ~(XBITMASK(y)-1) ;
	    h -= 8-y ;
	    if (h < 0)
	      msk -= ~(XBITMASK(8+h)-1) ;
			lcd_write_bits( p, msk ) ;
	    p += DISPLAY_W ;
		}

	  while( h >= 8 )
		{
			h -= 8 ;
			lcd_write_bits( p, 0xFF ) ;
	    p += DISPLAY_W ;
	  }

		if ( h > 0 )
		{
	  	lcd_write_bits( p, (XBITMASK(h)-1) ) ;
		}
		asm("") ;
	}

/*---------------------------------------------------------------------------*/
	void lcd_hlineStip(unsigned char x,unsigned char y, signed char w,uint8_t pat)
	{
	  if(w<0) {x+=w; w=-w;}
      uint8_t *p = dispBufAddress( x, y ) ;
      uint8_t msk = XBITMASK(y%8);
	  while(w){
	    if ( p>=DISPLAY_END)
	    {
	      break ;
	    }
	    if(pat&1) {
				lcd_write_bits( p, msk ) ;
	      pat = (pat >> 1) | 0x80;
	    }else{
	      pat = pat >> 1;
	    }
	    w--;
	    p++;
	  }
	}
/*---------------------------------------------------------------------------*/
	void lcd_hline(uint8_t x,uint8_t y, int8_t w)
	{
	  lcd_hlineStip(x,y,w,0xff);
	}
/*---------------------------------------------------------------------------*/
	void lcd_2_digits( uint8_t x, uint8_t y, uint8_t value, uint8_t attr )
	{
                uint8_t prec     = PREC(attr);
		lcd_outdezNAtt( x, y, value, attr + LEADING0, 2, prec ) ;
	}
/*---------------------------------------------------------------------------*/

        uint8_t lcd_outdezNAtt(uint8_t x, uint8_t y, int32_t val, uint8_t mode, int8_t len, uint8_t prec) {
          uint8_t fw       = FWNUM;
         
          uint8_t negative = 0;
          uint8_t xn       = 0;
          uint8_t ln       = 2;
          char c;
          uint8_t xinc;
          uint8_t fullwidth = 0;

          mode &= ~NO_UNIT;
          if (len < 0) {
            fullwidth = 1;
            len       = -len;
          }

          if (val < 0) {
            val      = -val;
            negative = 1;
          }

          if (mode & DBLSIZE) {
#ifdef SMALL_DBL
            fw          = 8;
            xinc        = 8;
            Lcd_lastPos = 8;
#else
            fw += FWNUM;
            xinc        = 2 * FWNUM;
            Lcd_lastPos = 2 * FW;
#endif
          } else {
            xinc        = FWNUM;
            Lcd_lastPos = FW;
          }

          if (mode & LEFT) {
            //    if (val >= 10000)
            //      x += fw;
            if (negative) {
              x += fw;
            }

            if (val >= 1000)
              x += fw;
            if (val >= 100)
              x += fw;
            if (val >= 10)
              x += fw;
         

           if(prec)
           {
              if (prec == 2) {
                if (val < 100) {
                  x += fw;
                }
              }
              if (val < 10) {
                x += fw;
              }
           }
            
          } else {
            x -= xinc;
          }
          Lcd_lastPos += x;

          //if (prec > 1) {
          //  mode -= LEADING0;    // Can't have PREC2 and LEADING0
          //}

          for (uint8_t i = 1; i <= len; i++) {
            div_t qr;
            qr = div(val, 10);
            c  = (qr.rem) + '0';
            lcd_putcAtt(x, y, c, mode);
            if (prec == i) {
              if (mode & DBLSIZE) {
                xn = x;
                if (c <= '3' && c >= '1')
                  ln++;
                uint8_t tn = (qr.quot) % 10;
                if (tn == 2 || tn == 4) {
                  if (c == '4') {
                    xn++;
                  } else {
                    xn--;
                    ln++;
                  }
                }
              } else {
                x -= 2;
                if (mode & INVERS)
                  lcd_vline(x + 1, y, 7);
                else
                  lcd_plot(x + 1, y + 6);
              }
              if (qr.quot)
                prec = 0;
            }
            val = qr.quot;
            if (!val) {
              if (prec) {
               if (i > prec-1)
               {
                  prec = 0;
               } 
                //if (prec == 2) {
                //  if (i > 1) {
                //    prec = 0;
                //  }
                //} else {
                //  prec = 0;
                //}
              } else if (mode & LEADING0) {
                if (fullwidth == 0) {
                  mode -= LEADING0;
                }
              } else
                break;
            }
            x -= fw;
          }
          if (xn) {
#ifdef SMALL_DBL
            lcd_hline(xn - 1, y + 2 * FH - 4, ln);
            lcd_hline(xn - 1, y + 2 * FH - 3, ln);
#else
            lcd_hline(xn, y + 2 * FH - 4, ln);
            lcd_hline(xn, y + 2 * FH - 3, ln);
#endif    // SMALL_DBL
          }
          if (negative)
            lcd_putcAtt(x - fw, y, '-', mode);
          asm("");
          return 0;    // Stops compiler creating two sets of POPS, saves flash
        }
/*---------------------------------------------------------------------------*/
void lcd_outdezAtt( uint8_t x, uint8_t y, int16_t val, uint8_t mode )
{
  uint8_t prec     = PREC(mode);
  lcd_outdezNAtt( x,y,val,mode,5,prec);
}
/*---------------------------------------------------------------------------*/

void lcd_outdez( uint8_t x, uint8_t y, int16_t val)
{
  lcd_outdezAtt(x,y,val,0);
}
/*---------------------------------------------------------------------------*/
void lcd_char_inverse( uint8_t x, uint8_t y, uint8_t w, uint8_t blink )
{
	if ( blink && BLINK_ON_PHASE )
	{
		return ;
	}
	uint8_t end = x + w ;
	uint8_t *p = dispBufAddress( x, y ) ;
	{
		while ( x < end )
		{
			*p++ ^= 0xFF ;
			x += 1 ;
		}
	}
}
/*---------------------------------------------------------------------------*/




/*---------------------------------------------------------------------------*/
uint8_t plotType = PLOT_XOR ;

void lcd_rect_xor(uint8_t x, uint8_t y, uint8_t w, uint8_t h )
{
  lcd_vline(x, y, h ) ;
	if ( w > 1 )
	{
  	lcd_vline(x+w-1, y, h ) ;
	}
 	lcd_hline(x, y+h-1, w ) ;
 	lcd_hline(x, y, w ) ;
}
/*---------------------------------------------------------------------------*/
void lcd_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h )
{
	uint8_t oldPlotType = plotType ;
	plotType = PLOT_BLACK ;
	lcd_rect_xor( x, y, w, h ) ;
	plotType = oldPlotType ;
}
/*---------------------------------------------------------------------------*/
void lcd_hbar(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t percent) {
  uint8_t solid;
  if (percent > 100) percent = 100;
  
  solid = (w - 2) * percent / 100;
  lcd_rect(x, y, w, h);

  if (solid) {
    w = y + h - 1;
    y += 1;
    x += 1;
    while (y < w) {
      lcd_hline(x, y, solid);
      y += 1;
    }
  }
}
/*---------------------------------------------------------------------------*/
void refreshDiplay() {
  if (EepromActive && BLINK_ON_PHASE) {
    lcd_hline(0, 0, EepromActive - '0' + 6);
  }

  LcdLock = 1;                  // Lock LCD data lines
#define column_start_lo 0x04    // skip first 4 columns for normal ST7565

  uint8_t *p = DisplayBuf;
  for (uint8_t y = 0xB0; y < 0xB8; y++) {
    lcdSendCtl(column_start_lo);
    lcdSendCtl(0x10);    // column addr 0
    lcdSendCtl(y);       // page addr y

    for (uint8_t x = 128; x > 0; x--)
      lcdSendData(*p++);
  }
  LcdLock = 0;    // Free LCD data lines
}
/*---------------------------------------------------------------------------*/
	void lcd_img(uint8_t i_x,uint8_t i_y,const unsigned char * imgdat,uint8_t idx)
	{
	  const unsigned char  *q = imgdat;

	  uint8_t w    = *(q++);
	  uint8_t hb   = *(q++) ;
		hb += 7 ;
		hb /= 8 ;
	  uint8_t sze1 = *(q++);
	  q += idx*sze1;
	  for(uint8_t yb = 0; yb < hb; yb++){
	    uint8_t   *p = &DisplayBuf[ (i_y / 8 + yb) * DISPLAY_W + i_x ];
	    for(uint8_t x=0; x < w; x++){
	      uint8_t b = *(q++);
	      *p++ = b;
	    }
	  }
	}
/*---------------------------------------------------------------------------*/
	void putsTime(uint8_t x,uint8_t y,int16_t tme,uint8_t att,uint8_t att2)
	{
		div_t qr ;
	#ifdef SMALL_DBL
		uint8_t z = FWNUM*6-2 ;
		if ( att&DBLSIZE )
		{
			x += 3 ;
			z = FWNUM*5-2 ;
		}

		if ( tme<0 )
		{
			lcd_putcAtt( x - ((att&DBLSIZE) ? z : FWNUM*3),    y, '-',att);
			tme = -tme;
		}

		lcd_putcAtt( x, y, ':',att&att2);
		qr = div( tme, 60 ) ;

		if ( att&DBLSIZE )
		{
			x += 2 ;
		}
		lcd_2_digits( x, y, (uint16_t)qr.quot, att ) ;

		if ( att&DBLSIZE )
		{
			x += FWNUM*5-4 ;
		}
		else
		{
			x += FW*3-3 ;
		}
		lcd_2_digits( x, y, (uint16_t)qr.rem, att2 ) ;
	#else
		if ( tme<0 )
		{
			lcd_putcAtt( x - ((att&DBLSIZE) ? FWNUM*6-2 : FWNUM*3),    y, '-',att);
			tme = -tme;
		}
		lcd_putcAtt(x, y, ':',att&att2);
		qr = div( tme, 60 ) ;
		lcd_2_digits( x, y, (uint16_t)qr.quot, att ) ;
		x += (att&DBLSIZE) ? FWNUM*6-4 : FW*3-3;
		lcd_2_digits( x, y, (uint16_t)qr.rem, att2 ) ;
	#endif
	}
/*---------------------------------------------------------------------------*/
	void putsVolts(uint8_t x,uint8_t y, int16_t volts, uint8_t att)
	{
		lcd_outdezAtt(x, y, volts, att|PREC1);
		if(!(att&NO_UNIT)) lcd_putcAtt(Lcd_lastPos, y, 'v', att);
	}

/*---------------------------------------------------------------------------*/
	void putsVBat(uint8_t x,uint8_t y,uint8_t att)
	{
	#ifndef MINIMISE_CODE
		att |= g_vbat100mV < g_eeGeneral.vBatWarn ? BLINK : 0;
	#endif
		putsVolts(x, y, g_vbat100mV, att);
	}
/*---------------------------------------------------------------------------*/