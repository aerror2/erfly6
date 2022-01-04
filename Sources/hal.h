
#ifndef __HAL_H
#define __HAL_H

#ifdef __cplusplus
extern "C" { 
#endif

#include <stdint.h>

extern char TX_name[]; 

#define prog_char     char
#define prog_uchar    unsigned char
#define prog_int8_t   int8_t
#define prog_uint8_t  uint8_t
#define prog_uint16_t uint16_t
#define APM

#define pgm_read_adr(address_short)  *(address_short)
#define pgm_read_byte(adr)  *(adr)
#define pgm_read_word(adr)  *(adr)

#define OUT_B_LIGHT   7
// Buttons PINB
#define INP_B_KEY_LFT 6
#define INP_B_KEY_RGT 5
#define INP_B_KEY_UP  4
#define INP_B_KEY_DWN 3
#define INP_B_KEY_EXT 2
#define INP_B_KEY_MEN 1
// Trimmers PIND
#define INP_D_TRM_LH_UP   7
#define INP_D_TRM_LH_DWN  6
#define INP_D_TRM_RV_DWN  5
#define INP_D_TRM_RV_UP   4
#define INP_D_TRM_LV_DWN  3
#define INP_D_TRM_LV_UP   2
#define INP_D_TRM_RH_DWN  1
#define INP_D_TRM_RH_UP   0
//PINE
#define INP_E_PPM_IN  7
#define INP_E_ID2     6
#define INP_E_Trainer 5
#define INP_E_Gear    4
#define OUT_E_BUZZER  3
#define INP_E_ElevDR  2
#define INP_E_AileDR  1
#define INP_E_ThrCt   0
//PING
#define OUT_G_SIM_CTL  4 //1 : phone-jack=ppm_in
#define INP_G_ID1      3
#define INP_G_RF_POW   1
#define INP_G_RuddDR   0





#define READBIT(A, B) ((A >> (B & 7)) & 1)
#define SETBIT(T, B, V) (T = V ? T | (1<<B) : T & ~(1<<B))

/******************************************************************************/
/*           Global definitions for all transmitters                          */
/******************************************************************************/

void HW_Init(void);  // hardware initialization
void mDelay(uint32_t Delay); // delay ms
extern uint16_t s_anaFilt[8]; // analog data array
extern volatile uint16_t g_tmr16KHz;
extern volatile uint16_t tmrEEPROM;

/***********************/
/* s_anaFilt[0] = AIL  */
/* s_anaFilt[1] = THR  */
/* s_anaFilt[2] = ELE  */
/* s_anaFilt[3] = RUD  */
/* s_anaFilt[4] = VrA  */
/* s_anaFilt[5] = VrB  */
/* s_anaFilt[6] = SwC  */
/* s_anaFilt[7] = BATT */ 
/***********************/
void getADC_osmp();  // ADC polling 

uint8_t PINB(void);  //simulated PINB
uint8_t PIND(void);  //simulated PIND
uint8_t PINE(void);  //simulated PINE
uint8_t PING(void);  //simulated PING

/******************************************************************************/
/*                           Buzzer control                                   */
/******************************************************************************/
void Buzzer_SetVal(void);
void Buzzer_ClrVal(void);
void Buzzer_ToggleVal(void);
/******************************************************************************/
/*                           LCD control                                      */
/******************************************************************************/
void backlight_on(void); 
void backlight_off(void); 

void rd_1(void);
void rd_0(void);
void cs_1(void);
void cs_0(void);
void rw_1(void);
void rw_0(void);
void rs_1(void);
void rs_0(void);
void rst_1(void);
void rst_0(void);
void LCD_DATA(uint8_t Data);

#define BACKLIGHT_ON  backlight_on() 
#define BACKLIGHT_OFF backlight_off() 

#define LCD_RD_1 rd_1()
#define LCD_RD_0 rd_0()
#define LCD_CS_1 cs_1()
#define LCD_CS_0 cs_0()
#define LCD_RW_1 rw_1()
#define LCD_RW_0 rw_0()
#define LCD_RS_1 rs_1()
#define LCD_RS_0 rs_0()
#define LCD_RST_1 rst_1()
#define LCD_RST_0 rst_0()

#define USE_IE_UART_TX 0
#define USE_DMA_UART  0

/******************************************************************************/
/*                           Radio control                                    */
/******************************************************************************/
void SPI_RADIO_SendBlock(uint8_t *BufferPtr, uint16_t Size);
void SPI_RADIO_ReceiveBlock(uint8_t *BufferPtr, uint16_t Size);
void a7105_csn_on(void); 
void a7105_csn_off(void);
void RF0_SetVal(void);
void RF0_ClrVal(void);
void RF1_SetVal(void);
void RF1_ClrVal(void);
void TX_RX_PutVal(uint32_t Val);
void EnableGIO(void);
void DisableGIO(void);
void initAFHDS2A();
void ActionAFHDS2A();
void initAFHDS();
void ActionAFHDS();

#define A7105_CSN_ON a7105_csn_on()  
#define A7105_CSN_OFF a7105_csn_off()

void lora_csn_on();
void lora_csn_off();
void lora_tx_switch(int s);
void lora_rx_switch(int s);
void lora_pa_switch(int s);
void lora_reset_switch(int s);
void lora_spi_set_frequency(uint32_t freq);
typedef void ( *LORA_GIO_ISR_FUNC)();
void lora_attach_gio_isr(LORA_GIO_ISR_FUNC isr);
void lora_detach_gio_isr();



typedef   int (*crsf_read_cb_t)(uint8_t x);
void setup_crsf_serial_port(uint32_t baud,crsf_read_cb_t read_cb);
void crsf_send_data(uint8_t *buf, uint32_t len);
//void crsf_wait_and_read();
void shutdown_crsf_serial_port();

 #if USE_IE_UART_TX ||USE_DMA_UART
int  crsf_is_sending();
#endif


/*----------------------------------------------------------------------------*/
/******************************************************************************/
/*                           i2c EEPROM                                       */
/******************************************************************************/
#define I2C_TX	0
#define I2C_RX	1
/* Structure for storing I2C transfer data */
typedef struct {
	uint16_t tx_index; /* TX index */
	uint16_t rx_index; /* RX index */
	uint8_t data_present; /* Data present flag */
	uint16_t length; /* Length of the buffer in bytes */
	uint8_t* buf; /* Data buffer */
} Ti2c_buffer;

#define I2C_OK    0
#define I2C_BUSY  1
#define I2C_NOACK 2

extern Ti2c_buffer i2c_buffer;
extern const uint8_t AFHDS2A_A7105_regs[];
extern const uint8_t AFHDS_A7105_regs[];
extern void ISR_TIMER0_COMP_vect(void);
extern void ISR_TIMER2_OVF_vect(void);
extern void ISR_TIMER1_COMPA_vect(void);
extern void ISR_TIMER3_CAPT_vect(void);
extern void ActionAFHDS2A();

uint8_t i2c_master(uint8_t mode, uint16_t slave_address);
/*---------------------PPM Timer------------------------------------------------*/
void EnablePPMTim(void);
void DisablePPMTim(void);
uint16_t GetPPMTimCapture(void);
void SetPPMTimCompare(uint16_t val);
uint16_t GetPPMTimCompare(void);
uint32_t GetPPMTimCompareInterruptFlag(void);
uint32_t GetPPMOutState(void);
uint32_t GetPPMTimCompareInterruptFlag(void);
void ClearPPMTimCompareInterruptFlag(void);
void EnablePPMOut(void);
void DisablePPMOut(void);
/*---------------------PRT Timer------------------------------------------------*/
void EnablePRTTim(void);
void DisablePRTTim(void);
uint16_t GetPRTTimVal(void);
void SetPRTTimPeriod(uint8_t prot);

uint32_t GetChipID(void);
void sei(void);
void cli(void);
#ifdef __cplusplus
}  /* extern "C" */
#endif 

#endif