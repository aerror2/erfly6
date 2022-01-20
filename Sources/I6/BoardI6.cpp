




#include "BoardI6.h"
#include "../er9x.h"
#include "../hal.h"
#include "../iface_a7105.h"
#include "../LoRa/ELRS.h"
#include "../crossfire/crossfire.h"
#include "../pulses.h"
#include "../../drivers/fsl_uart_dma.h"
#include "../../drivers/fsl_lpsci_dma.h"



char TX_name[] = "FlySky-I6";  
//volatile uint32_t g_tickcount =0;
#define DEFAULT_SYSTEM_CLOCK            48000000u
void HW_Init(void) {
  /* Enable clock gate for ports to enable pin routing */
  SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK |
                SIM_SCGC5_PORTD_MASK |
                SIM_SCGC5_PORTC_MASK |
                SIM_SCGC5_PORTB_MASK |
                SIM_SCGC5_PORTA_MASK;

  /*-----------------------ADC_Init--------------------------------------------------*/
  SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK;
  /* ADC0_CFG2: MUXSEL=1 */
  ADC0->CFG2 |= ADC_CFG2_MUXSEL_MASK;
  /* PORTD_PCR6: ISF=0,MUX=0 */
  PORTD->PCR[6] &= (uint32_t) ~(uint32_t)((PORT_PCR_ISF_MASK | PORT_PCR_MUX(0x07)));
  /* PORTD_PCR1: ISF=0,MUX=0 */
  PORTD->PCR[1] &= (uint32_t) ~(uint32_t)((PORT_PCR_ISF_MASK | PORT_PCR_MUX(0x07)));
  /* PORTD_PCR5: ISF=0,MUX=0 */
  PORTD->PCR[5] &= (uint32_t) ~(uint32_t)((PORT_PCR_ISF_MASK | PORT_PCR_MUX(0x07)));
  /* PORTC_PCR2: ISF=0,MUX=0 */
  PORTC->PCR[2] &= (uint32_t) ~(uint32_t)((PORT_PCR_ISF_MASK | PORT_PCR_MUX(0x07)));
  /* PORTE_PCR22: ISF=0,MUX=0 */
  PORTE->PCR[22] &= (uint32_t) ~(uint32_t)((PORT_PCR_ISF_MASK | PORT_PCR_MUX(0x07)));
  /* PORTE_PCR29: ISF=0,MUX=0 */
  PORTE->PCR[29] &= (uint32_t) ~(uint32_t)((PORT_PCR_ISF_MASK | PORT_PCR_MUX(0x07)));
  /* PORTC_PCR0: ISF=0,MUX=0 */
  PORTC->PCR[0] &= (uint32_t) ~(uint32_t)((PORT_PCR_ISF_MASK | PORT_PCR_MUX(0x07)));
  ADC0->CFG1 = ADC_CFG1_ADIV(0x00) |
               ADC_CFG1_ADLSMP_MASK |
               ADC_CFG1_MODE(0x01) |
               ADC_CFG1_ADICLK(0x02);

  /* ADC0_CFG2: ADACKEN=0,ADHSC=0,ADLSTS=1 */
  ADC0->CFG2 = (uint32_t)((ADC0->CFG2 & (uint32_t) ~(uint32_t)(
                                            ADC_CFG2_ADACKEN_MASK |
                                            ADC_CFG2_ADHSC_MASK |
                                            ADC_CFG2_ADLSTS(0x02))) |
                          (uint32_t)(
                              ADC_CFG2_ADLSTS(0x01)));

  ADC0->SC2 = ADC_SC2_REFSEL(0x00);
  ADC0->SC3 = (ADC_SC3_CALF_MASK | ADC_SC3_AVGE_MASK | ADC_SC3_AVGS(0x03) | ADC_ONE_CONVERSION);
  /*-----------------------ADC_Calibrate-----------------------------------------------*/
  ADC0->SC1[0] = 0x1FU;
  ADC0->SC3 = (uint32_t)(((uint32_t)(ADC0->SC3 | ADC_SC3_CAL_MASK)) &
                         ((uint32_t)(~(uint32_t)ADC_SC3_CALF_MASK)));
  while (!(ADC0->SC1[0] & ADC_SC1_COCO_MASK)) {
  }

  /*-----------------------BTN_L_Init-------------------------------------------------*/
  /* Configure pin directions */
  /* GPIOB_PDDR: PDD&=~0x000F0000 */
  PTB->PDDR &= (uint32_t) ~(uint32_t)(GPIO_PDDR_PDD(BTN_L_PORT_MASK));
  /* Initialization of Port Control register */
  /* PORTB_PCR16: ISF=0,MUX=1 */
  PORTB->PCR[16] = (uint32_t)(PORT_PCR_MUX(0x01));
  /* PORTB_PCR17: ISF=0,MUX=1 */
  PORTB->PCR[17] = (uint32_t)(PORT_PCR_MUX(0x01));
  /* PORTB_PCR18: ISF=0,MUX=1 */
  PORTB->PCR[18] = (uint32_t)(PORT_PCR_MUX(0x01));
  /* PORTB_PCR19: ISF=0,MUX=1 */
  PORTB->PCR[19] = (uint32_t)(PORT_PCR_MUX(0x01));
  /*-----------------------BTN_R_Init-------------------------------------------------*/
  /* Configure pin directions */
  /* GPIOB_PDDR: PDD|=0x0E */
  PTB->PDDR |= GPIO_PDDR_PDD(BTN_R_PORT_MASK);
  /* Initialization of Port Control register */
  /* PORTB_PCR1: ISF=0,MUX=1 */
  PORTB->PCR[1] = (uint32_t)(PORT_PCR_MUX(0x01));
  /* PORTB_PCR2: ISF=0,MUX=1 */
  PORTB->PCR[2] = (uint32_t)(PORT_PCR_MUX(0x01));
  /* PORTB_PCR3: ISF=0,MUX=1 */
  PORTB->PCR[3] = (uint32_t)(PORT_PCR_MUX(0x01));
  /*-----------------------BIND_Init-------------------------------------------------*/
  PTA->PDDR = (uint32_t) ~(uint32_t)(GPIO_PDDR_PDD(0x20));
  PORTA->PCR[5] = (uint32_t)(PORT_PCR_MUX(0x01));
  /*-----------------------BACKLIGHT_Init---------------------------------------------*/
  PTB->PDDR |= GPIO_PDDR_PDD(0x01);
  PTB->PDOR &= (uint32_t) ~(uint32_t)(GPIO_PDOR_PDO(0x01));
  PORTB->PCR[0] = (uint32_t)(PORT_PCR_MUX(0x01));
  /*-----------------------LD_RD_Init-------------------------------------------------*/
  PTE->PDDR |= GPIO_PDDR_PDD(0x40000000);
  PTE->PDOR &= (uint32_t) ~(uint32_t)(GPIO_PDOR_PDO(0x40000000));
  PORTE->PCR[30] = (uint32_t)(PORT_PCR_MUX(0x01));
  /*-----------------------LD_CS_Init-------------------------------------------------*/
  PTA->PDDR |= GPIO_PDDR_PDD(0x10);
  PTA->PDOR |= GPIO_PDOR_PDO(0x10);
  PORTA->PCR[4] = (uint32_t)(PORT_PCR_MUX(0x01));
  /*-----------------------LD_RW_Init-------------------------------------------------*/
  PTE->PDDR |= GPIO_PDDR_PDD(0x80000000);
  PTE->PDOR |= GPIO_PDOR_PDO(0x80000000);
  PORTE->PCR[31] = (uint32_t)(PORT_PCR_MUX(0x01));
  /*-----------------------LD_RS_Init-------------------------------------------------*/
  PTA->PDDR |= GPIO_PDDR_PDD(0x2000);
  PTA->PDOR |= GPIO_PDOR_PDO(0x2000);
  PORTA->PCR[13] = (uint32_t)(PORT_PCR_MUX(0x01));
  /*-----------------------LD_RST_Init------------------------------------------------*/
  PTD->PDDR |= GPIO_PDDR_PDD(0x80);
  PTD->PDOR |= GPIO_PDOR_PDO(0x80);
  PORTD->PCR[7] = (uint32_t)(PORT_PCR_MUX(0x01));
  /*-----------------------LD_DATA_LO_Init-------------------------------------------*/
  PTE->PDDR |= GPIO_PDDR_PDD(0x003C0000);
  PTE->PDOR &= (uint32_t) ~(uint32_t)(GPIO_PDOR_PDO(0x003C0000));
  PORTE->PCR[18] = (uint32_t)(PORT_PCR_MUX(0x01));
  PORTE->PCR[19] = (uint32_t)(PORT_PCR_MUX(0x01));
  PORTE->PCR[20] = (uint32_t)(PORT_PCR_MUX(0x01));
  PORTE->PCR[21] = (uint32_t)(PORT_PCR_MUX(0x01));

  //CONFIG PTE16 AS USART2 TX
 //   /* PORTE_PCR16: ISF=0,MUX=3 */
  //PORTE->PCR[16] =  (uint32_t)((PORTE->PCR[16] & (uint32_t)~(uint32_t)(
  //                PORT_PCR_ISF_MASK |
  //                PORT_PCR_MUX(0x04)
  //               )) | (uint32_t)(
  //                PORT_PCR_MUX(0x03)
  //               ));
  
  // PTE->PDDR |= 1<<16; 

  PORTE->PCR[16] |= PORT_PCR_MUX(0x03); //CONFIG AS TX2
  PORTE->PCR[17] |= PORT_PCR_MUX(0x03);//CONFIG AS RX2

  /*-----------------------LD_DATA_HI_Init--------------------------------------*/
  PTC->PDDR |= GPIO_PDDR_PDD(0x0F00);
  PTC->PDOR &= (uint32_t) ~(uint32_t)(GPIO_PDOR_PDO(0x0F00));
  PORTC->PCR[8] = (uint32_t)(PORT_PCR_MUX(0x01));
  PORTC->PCR[9] = (uint32_t)(PORT_PCR_MUX(0x01));
  PORTC->PCR[10] = (uint32_t)(PORT_PCR_MUX(0x01));
  PORTC->PCR[11] = (uint32_t)(PORT_PCR_MUX(0x01));
  /*-----------------------SwA_Init--------------------------------------*/
  PTD->PDDR &= (uint32_t) ~(uint32_t)(GPIO_PDDR_PDD(0x01));
  PORTD->PCR[0] = (uint32_t)(PORT_PCR_MUX(0x01));
  /*-----------------------SwB_Init--------------------------------------*/
  PTC->PDDR &= (uint32_t) ~(uint32_t)(GPIO_PDDR_PDD(0x02));
  PORTC->PCR[1] = (uint32_t)(PORT_PCR_MUX(0x01));
  /*-----------------------SwD_Init--------------------------------------*/
  PTD->PDDR &= (uint32_t) ~(uint32_t)(GPIO_PDDR_PDD(0x10));
  PORTD->PCR[4] = (uint32_t)(PORT_PCR_MUX(0x01));
  /*-----------------------Buzzer_Init------------------------------------*/
  PTA->PDDR |= GPIO_PDDR_PDD(0x1000);
  PTA->PDOR &= (uint32_t) ~(uint32_t)(GPIO_PDOR_PDO(0x1000));
  PORTA->PCR[12] = (uint32_t)(PORT_PCR_MUX(0x01));
  /*-----------------------SPI_RADIO_Init---------------------------------*/
  /* SIM_SCGC4: SPI0=1 */
  SIM->SCGC4 |= SIM_SCGC4_SPI0_MASK;
  /* PORTC_PCR6: ISF=0,MUX=2 */
  PORTC->PCR[6] = (uint32_t)(PORT_PCR_MUX(0x02));
  /* PORTC_PCR5: ISF=0,MUX=2 */
  PORTC->PCR[5] = (uint32_t)(PORT_PCR_MUX(0x02));
  /* SPI0_C1: SPIE=0,SPE=0,SPTIE=0,MSTR=1,CPOL=0,CPHA=0,SSOE=1,LSBFE=0 */
  SPI0->C1 = (SPI_C1_MSTR_MASK | SPI_C1_SSOE_MASK); /* Set configuration register */
  /* SPI0_C2: SPMIE=0,SPIMODE=0,TXDMAE=0,MODFEN=1,BIDIROE=0,RXDMAE=0,SPISWAI=0,SPC0=1 */
  SPI0->C2 = SPI_C2_MODFEN_MASK | SPI_C2_SPC0_MASK | SPI_C2_BIDIROE_MASK; /* pin mode is bidirectional */
  /* SPI0_BR: ??=0,SPPR=0,SPR=1 */
  SPI0->BR = (SPI_BR_SPPR(0x00) | SPI_BR_SPR(0x01)); /* Set baud rate register */
  /* SPI0_C1: SPE=1 */
  NVIC_SetPriority(SPI0_IRQn, 0);
  NVIC_EnableIRQ(SPI0_IRQn);
  SPI0->C1 |= SPI_C1_SPE_MASK; /* Enable SPI module */
  /*-----------------------RADIO_SCN_Init-----------------------------------------*/
  PTC->PDDR |= GPIO_PDDR_PDD(0x10);
  PTC->PDOR |= GPIO_PDOR_PDO(0x10);
  PORTC->PCR[4] = (uint32_t)(PORT_PCR_MUX(0x01));

  /*-----------------------RADIO_LORA_SCN_Init-----------------------------------------*/
  PTD->PDDR |= GPIO_PDDR_PDD(0x4);   //PTD2 DIRECTION OUTPUT
  PTD->PDOR  &= (uint32_t) ~(uint32_t)(GPIO_PDOR_PDO(0x04));
  PORTD->PCR[2] = (uint32_t)(PORT_PCR_MUX(0x1)); //MODE PULL
  FPTD->PSOR = LORA_SCN_PORT_MASK;

  /*------------------------------RADIO_RF0_Init----------------------------------*/
  PTC->PDDR |= GPIO_PDDR_PDD(0x80);
  PTC->PDOR |= GPIO_PDOR_PDO(0x80);
  PORTC->PCR[7] = (uint32_t)(PORT_PCR_MUX(0x01));
  /*------------------------------RADIO_RF1_Init-----------------------------------*/
  PTC->PDDR |= GPIO_PDDR_PDD(0x08);
  PTC->PDOR &= (uint32_t) ~(uint32_t)(GPIO_PDOR_PDO(0x08));
  PORTC->PCR[3] = (uint32_t)(PORT_PCR_MUX(0x01));
  /*------------------------------RADIO_TX_RX_Init---------------------------------*/
  PTE->PDDR |= GPIO_PDDR_PDD(0x03);
  PTE->PDOR &= (uint32_t) ~(uint32_t)(GPIO_PDOR_PDO(0x03));
  PORTE->PCR[0] = (uint32_t)(PORT_PCR_MUX(0x01));
  PORTE->PCR[1] = (uint32_t)(PORT_PCR_MUX(0x01));


  /*-----------------------RADIO_GIO1_Init--------------------------------------*/
  /* PORTD_PCR3: ISF=0,MUX=1 */
  PORTD->PCR[3] = (uint32_t)(PORT_PCR_MUX(0x01));
  NVIC_SetPriority(PORTC_PORTD_IRQn, 2);
  NVIC_EnableIRQ(PORTC_PORTD_IRQn);
  /*-----------------------PIT_TIMER_Init--------------------------------------*/
  SIM->SCGC6 |= SIM_SCGC6_PIT_MASK;

  PIT->CHANNEL[0].LDVAL = 1499; // For timer16 kHz
  PIT->CHANNEL[0].TCTRL = PIT_TCTRL_TEN_MASK | PIT_TCTRL_TIE_MASK;
  PIT->CHANNEL[1].LDVAL = 159; // For timer 10ms
  PIT->CHANNEL[1].TCTRL = PIT_TCTRL_TEN_MASK | PIT_TCTRL_TIE_MASK | PIT_TCTRL_CHN_MASK;
  NVIC_SetPriority(PIT_IRQn, 2);
  NVIC_EnableIRQ(PIT_IRQn);
  PIT->MCR = 0x00;
  /*------------------------------TPM Clock------------------------------------*/
  SIM->SOPT2 = (SIM_SOPT2_TPMSRC(0x01) | SIM_SOPT2_PLLFLLSEL_MASK);
  /*------------Audio_TIMER_Init(TPM1 Interrupt period 128 uS)-----------------*/
  SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK;
  TPM1->MOD = TPM_MOD_MOD(3071);
  NVIC_SetPriority(TPM1_IRQn, 0);
  NVIC_EnableIRQ(TPM1_IRQn);
  TPM1->SC = (TPM_SC_CMOD(0x01) | TPM_SC_PS(0x01) | TPM_SC_TOIE_MASK);
  /*------------------Radio_Protocol_Timer_Init(3860 uS TPM0)------------------*/
  SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK;
  TPM0->MOD = TPM_MOD_MOD(11579);
  NVIC_SetPriority(TPM0_IRQn, 2);
  NVIC_EnableIRQ(TPM0_IRQn);
  TPM0->SC = 0x00;
  /*------------------PPM_Timer_Init(3 mHz counter frequency TPM2)-------------*/
  SIM->SCGC6 |= SIM_SCGC6_TPM2_MASK;
  TPM2->MOD = TPM_MOD_MOD(0xFFFF);      
  TPM2->CONTROLS[0].CnSC = 0x00;// (TPM_CnSC_CHIE_MASK | TPM_CnSC_MSA_MASK | TPM_CnSC_ELSA_MASK); // Compare
  TPM2->CONTROLS[0].CnV = TPM_CnV_VAL(0xEA60);      
  //TPM2->CONTROLS[1].CnSC = (TPM_CnSC_CHIE_MASK | TPM_CnSC_ELSB_MASK); // Capture
  PORTA->PCR[1] = (uint32_t)(PORT_PCR_MUX(0x03));
  
  SIM->SOPT2 |= (SIM_SOPT2_UART0SRC(0x01)); //01 MCGFLLCLK clock, SIM_SOPT2_PLLFLLSEL_MASK  DIVIDED BY 2
  PORTA->PCR[2] = (uint32_t)(PORT_PCR_MUX(0x02)); //config as UART0  TX

  NVIC_SetPriority(TPM2_IRQn, 2);
  NVIC_EnableIRQ(TPM2_IRQn);
  TPM2->SC = 0x00; 
  /*------------------------------I2C_Init------------------------------------*/
  SIM->SCGC4 |= SIM_SCGC4_I2C0_MASK;
  PORTE->PCR[25] = (uint32_t)(PORT_PCR_MUX(0x05));
  PORTE->PCR[24] = (uint32_t)(PORT_PCR_MUX(0x05));
  I2C0->F = (I2C_F_MULT(0x00) | I2C_F_ICR(0x12)); /* Set prescaler bits */
  I2C0->C1 = I2C_C1_IICEN_MASK;
  /*------------------------------SysTick_Init---------------------------------*/

  SysTick->LOAD  = (uint32_t)((DEFAULT_SYSTEM_CLOCK / 1000U) - 1UL);  /* set reload register */
  SysTick->VAL   = 0UL;                                    /* Load the SysTick Counter Value */
  SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk |
                   SysTick_CTRL_ENABLE_Msk;                /* Enable the Systick Timer */
}
/*----------------------------------------------------------------------------*/
#define MAX_DELAY                  0xFFFFFFFFU
void mDelay(uint32_t Delay)
{
  __IO uint32_t  tmp = SysTick->CTRL;  /* Clear the COUNTFLAG first */
  /* Add this code to indicate that local variable is not used */
  ((void)tmp);

  /* Add a period to guaranty minimum wait */
  if (Delay < MAX_DELAY)
  {
    Delay++;
  }

  while (Delay)
  {
    if ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) != 0U)
    {
      Delay--;
    }
  }
}

/******************************************************************************/
/*                           Get ADC channels                                 */
/******************************************************************************/
/*----------------------------------------------------------------------------*/
static const uint8_t ChannelToPin[] = {/* Channel to pin conversion table */
    ADC_AIL_VrB,
    ADC_ELE,
    ADC_THR,
    ADC_RUD,
    ADC_VrA,
    ADC_BATT,
    ADC_SwC};
void getADC_osmp() {
  uint16_t AnaInput;

  for (uint8_t i = 0; i < 7; i++) {
    ADC0->SC1[0] = ChannelToPin[i];
    while (!(ADC0->SC1[0] & ADC_SC1_COCO_MASK)) {
    }
    AnaInput = ADC0->R[0];
    s_anaFilt[i] = AnaInput >> 1;
  }
  s_anaFilt[7] = s_anaFilt[5];
  ADC0->CFG2 &= (uint32_t) ~(uint32_t)(ADC_CFG2_MUXSEL_MASK);
  ADC0->SC1[0] = ChannelToPin[0];
  while (!(ADC0->SC1[0] & ADC_SC1_COCO_MASK)) {
  }
  AnaInput = ADC0->R[0];
  s_anaFilt[5] = AnaInput >> 1;
  ADC0->CFG2 |= ADC_CFG2_MUXSEL_MASK;
}
/*------------------------------------------------------------------------------*/
void backlight_on(void) { FPTB->PSOR = LED_PORT_MASK; }
void backlight_off(void) { FPTB->PCOR = LED_PORT_MASK; }

void rd_1(void) { FPTE->PSOR = LCD_RD_PORT_MASK; }
void rd_0(void) { FPTE->PCOR = LCD_RD_PORT_MASK; }
void cs_1(void) { FPTA->PSOR = LCD_CS_PORT_MASK; }
void cs_0(void) { FPTA->PCOR = LCD_CS_PORT_MASK; }
void rw_1(void) { FPTE->PSOR = LCD_RW_PORT_MASK; }
void rw_0(void) { FPTE->PCOR = LCD_RW_PORT_MASK; }
void rs_1(void) { FPTA->PSOR = LCD_RS_PORT_MASK; }
void rs_0(void) { FPTA->PCOR = LCD_RS_PORT_MASK; }
void rst_1(void) { FPTD->PSOR = LCD_RST_PORT_MASK; }
void rst_0(void) { FPTD->PCOR = LCD_RST_PORT_MASK; }
/*------------------------------------------------------------------------------*/

uint8_t PINB(void) {
#define FS_DOWN 0
#define FS_UP 1
#define FS_OK 2
#define FS_CANCEL 3
  uint8_t pinb = 0x7E;
  uint32_t in;
  BTN_R_ClrBit(2);
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  in = BTN_L_GetVal();
  BTN_R_PutVal(0x0F);

  SETBIT(pinb, INP_B_KEY_MEN, READBIT(in, FS_OK));
  SETBIT(pinb, INP_B_KEY_EXT, READBIT(in, FS_CANCEL));
  if (BIND_GetVal() == 0) {
    SETBIT(pinb, INP_B_KEY_RGT, READBIT(in, FS_UP));
    SETBIT(pinb, INP_B_KEY_LFT, READBIT(in, FS_DOWN));
  } else {
    SETBIT(pinb, INP_B_KEY_UP, READBIT(in, FS_UP));
    SETBIT(pinb, INP_B_KEY_DWN, READBIT(in, FS_DOWN));
  }
  return pinb;
}

/*---------------------------------------------------------------------------*/
uint8_t PIND(void) {
#define FS_TRM_LV_UP 0
#define FS_TRM_LV_DWN 1
#define FS_TRM_LH_UP 2
#define FS_TRM_LH_DWN 3
#define FS_TRM_RH_UP 0
#define FS_TRM_RH_DWN 1
#define FS_TRM_RV_UP 2
#define FS_TRM_RV_DWN 3
  uint8_t pind = 0;
  uint32_t in;
  BTN_R_ClrBit(1);
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  in = BTN_L_GetVal();
  BTN_R_PutVal(0x0F);
  SETBIT(pind, INP_D_TRM_LV_UP, READBIT(in, FS_TRM_LV_UP));
  SETBIT(pind, INP_D_TRM_LV_DWN, READBIT(in, FS_TRM_LV_DWN));
  SETBIT(pind, INP_D_TRM_LH_UP, READBIT(in, FS_TRM_LH_UP));
  SETBIT(pind, INP_D_TRM_LH_DWN, READBIT(in, FS_TRM_LH_DWN));
  BTN_R_ClrBit(0);
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  in = BTN_L_GetVal();
  BTN_R_PutVal(0x0F);
  SETBIT(pind, INP_D_TRM_RH_UP, READBIT(in, FS_TRM_RH_UP));
  SETBIT(pind, INP_D_TRM_RH_DWN, READBIT(in, FS_TRM_RH_DWN));
  SETBIT(pind, INP_D_TRM_RV_UP, READBIT(in, FS_TRM_RV_UP));
  SETBIT(pind, INP_D_TRM_RV_DWN, READBIT(in, FS_TRM_RV_DWN));
  return pind;
}
/*---------------------------------------------------------------------------*/
#define ONE_THIRD 2048 / 3
uint8_t PINE(void) {
  uint8_t pine = 0 | (1 << INP_E_ID2);
  if (s_anaFilt[5] >= ONE_THIRD * 2)
    SETBIT(pine, INP_E_ID2, 0);

  if (s_anaFilt[6] <= ONE_THIRD)
    SETBIT(pine, INP_E_AileDR, 1);
  else if ((s_anaFilt[6] > ONE_THIRD) & (s_anaFilt[6] < ONE_THIRD * 2))
    SETBIT(pine, INP_E_Gear, 1);
  else if (s_anaFilt[6] >= ONE_THIRD * 2)
    SETBIT(pine, INP_E_Trainer, 1);

  SETBIT(pine, INP_E_ThrCt, !SWA_GetVal());
  SETBIT(pine, INP_E_ElevDR, !SWD_GetVal());
  return pine;
}
/*---------------------------------------------------------------------------*/
uint8_t PING(void) {
  uint8_t ping = 0;
  if (s_anaFilt[5] > ONE_THIRD)
    SETBIT(ping, INP_G_ID1, 1);

  SETBIT(ping, INP_G_RuddDR, SWB_GetVal());
  return ping;
}
/*---------------------------------------------------------------------------*/
void LCD_DATA(uint8_t Data){
  FPTE->PCOR = (LCD_DATA_LO_PORT_MASK);
  FPTE->PSOR = ((uint32_t)(Data & 0x0F) << LCD_DATA_LO_PIN_ALLOC_0_INDEX);
  FPTC->PCOR = (LCD_DATA_HI_PORT_MASK);
  FPTC->PSOR = ((uint32_t)(Data >> 4) << LCD_DATA_HI_PIN_ALLOC_0_INDEX);
}
/*---------------------------------------------------------------------------*/
void Buzzer_SetVal(void) { FPTA->PSOR = Buzzer_PORT_MASK; }
void Buzzer_ClrVal(void) { FPTA->PCOR = Buzzer_PORT_MASK; }
void Buzzer_ToggleVal(void) { FPTA->PTOR = Buzzer_PORT_MASK; }
/*---------------------------------------------------------------------------*/
/******************************************************************************/
/*                           Radio control                                    */
/******************************************************************************/
const uint8_t AFHDS2A_A7105_regs[] = {
	0xFF, 0x42 | (1<<5), 0x00, 0x25, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00,0x01, 0x3c, 0x05, 0x00, 0x50, // 00 - 0f
	0x9e, 0x4b, 0x00, 0x02, 0x16, 0x2b, 0x12, 0x4f, 0x62, 0x80, 0xFF, 0xFF, 0x2a, 0x32, 0xc3, 0x1E/*0x1f*/,	 // 10 - 1f
	0x1e, 0xFF, 0x00, 0xFF, 0x00, 0x00, 0x3b, 0x00, 0x17, 0x47, 0x80, 0x03, 0x01, 0x45, 0x18, 0x00,		 // 20 - 2f
	0x01, 0x0f // 30 - 31
};

const uint8_t AFHDS_A7105_regs[] = {
       /* 00    01    02    03    04    05    06    07    08    09    0A    0B    0C    0D    0E    0F  */
	0xff, 0x42, 0x00, 0x14, 0x00, 0xff, 0xff ,0x00, 0x00, 0x00, 0x00, 0x01, 0x21, 0x05, 0x00, 0x50,	// 00 - 0f
	0x9e, 0x4b, 0x00, 0x02, 0x16, 0x2b, 0x12, 0x00, 0x62, 0x80, 0x80, 0x00, 0x0a, 0x32, 0xc3, 0x0f,	// 10 - 1f
/*0x13*/0x1c, 0xc3, 0x00, 0xff, 0x00, 0x50/*0x00*/, 0x3b, 0x00, 0x17, 0x47, 0x80, 0x03, 0x01, 0x45, 0x18, 0x00,	// 20 - 2f
	0x01, 0x0f // 30 - 31
};

volatile uint8_t *SPI_BufferPtr;
volatile uint16_t SPI_Size;
void SPI_RADIO_SendBlock(uint8_t *BufferPtr, uint16_t Size) {
  SPI_BufferPtr = BufferPtr;
  SPI_Size = Size;
  SPI0->C1 |= SPI_C1_SPTIE_MASK; // enable transmit irq
  while (SPI0->C1 & SPI_C1_SPTIE_MASK) {
  }
  for (uint8_t i = 0; i < 10; i++) {
    asm("nop");
  }
}
/*---------------------------------------------------------------------------*/
void SPI_RADIO_ReceiveBlock(uint8_t *BufferPtr, uint16_t Size) {
  SPI_BufferPtr = BufferPtr;
  SPI_Size      = Size;
  uint8_t dummy_read;
  while (!(SPI0->S & SPI_S_SPTEF_MASK)) {
  }
  SPI0->C2 &= ~(SPI_C2_BIDIROE_MASK);    // SPI I/O pin enabled as an input
  if (SPI0->S & SPI_S_SPRF_MASK)
    dummy_read = SPI0->DL;
  (void)dummy_read;

  SPI0->C1 |= SPI_C1_SPIE_MASK;    // enable receive irq
  SPI0->DL = 0x00;
  while (SPI0->C1 & SPI_C1_SPIE_MASK) {
  }
  SPI0->C2 |= SPI_C2_BIDIROE_MASK;    // SPI I/O pin enabled as an output
}
/*---------------------------------------------------------------------------*/
void a7105_csn_on(void) { FPTC->PSOR = SCN_PORT_MASK; }
void a7105_csn_off(void) { FPTC->PCOR = SCN_PORT_MASK; }
void RF0_SetVal(void) { FPTC->PSOR = RF0_PORT_MASK; }
void RF0_ClrVal(void) { FPTC->PCOR = RF0_PORT_MASK; }
void RF1_SetVal(void) { FPTC->PSOR = RF1_PORT_MASK; }
void RF1_ClrVal(void) { FPTC->PCOR = RF1_PORT_MASK; }
void TX_RX_PutVal(uint32_t Val) {
  FPTE->PCOR = (TX_RX_PORT_MASK);
  FPTE->PSOR = ((uint32_t)(Val) << TX_RX_PIN_ALLOC_0_INDEX);
}

void EnableGIO(void){
  /* Clear interrupt status flag */
  PORTD->ISFR = PORT_ISFR_ISF(GIO1_PIN_MASK);
  PORTD->PCR[3] |= PORT_INTERRUPT_ON_FALLING;
}
void DisableGIO(void){
  PORTD->PCR[3] &= ~PORT_INTERRUPT_ON_FALLING;
}
uint32_t GetGIO1State(void) {
  return PTD->PDIR & GIO1_PIN_MASK;
}

/******************************************************************************/
/*                         EEPROM I2C                                         */
/******************************************************************************/

uint8_t i2c_master(uint8_t mode, uint16_t slave_address) {
  uint8_t dummy_read;
  uint32_t busy_counter = DEFAULT_SYSTEM_CLOCK / 1000;

  /* Clear IBCR.IBDIS, disable interrupts */
  I2C0->C1 = 0x80;
  /* Reset index for TX and RX buffers */
  i2c_buffer.tx_index = 0;
  i2c_buffer.rx_index = 0;
  if (mode == I2C_TX) {
    /* Make sure bus is idle */
    while (I2C0->S & I2C_S_BUSY_MASK) {
      if (!(busy_counter--))
        return I2C_BUSY;
    }
    /*Put module in master TX mode (generates START)*/
    I2C0->C1 |= (I2C_C1_MST_MASK | I2C_C1_TX_MASK);
    /* Put target address into IBDR */
    I2C0->D = ((slave_address << 1) | I2C_TX);
    /* Wait for address transfer to complete */
    while (!(I2C0->S & I2C_S_IICIF_MASK)) {
      if (!(busy_counter--))
        return I2C_BUSY;
    }
    I2C0->S |= I2C_S_IICIF_MASK;
    if (I2C0->S & I2C_S_RXAK_MASK) {
      return I2C_NOACK;
    }

    /* Send the contents of the TX buffer */
    while (i2c_buffer.length > 0) {
      I2C0->D = *i2c_buffer.buf++;
      i2c_buffer.length--;
      /* Wait for transfer to complete */
      while (!(I2C0->S & I2C_S_IICIF_MASK)) {
      if (!(busy_counter--))
        return I2C_BUSY;
       }
      I2C0->S |= I2C_S_IICIF_MASK;
    }
    /* Restore module to it's idle (but active) state */
    I2C0->C1 = 0x80;
    while ((I2C0->S & I2C_S_BUSY_MASK)) {
      if (!(busy_counter--))
        return I2C_BUSY;
     }
    return I2C_OK;
  } else if (mode == I2C_RX) {
    /* Make sure bus is idle */
    while (I2C0->S & I2C_S_BUSY_MASK) {
      if (!(busy_counter--))
        return I2C_BUSY;
     }
    /* Put module in master TX mode (generates START) */
    I2C0->C1 |= (I2C_C1_MST_MASK | I2C_C1_TX_MASK);
    /* Put target address into IBDR */
    I2C0->D = ((slave_address << 1) | I2C_RX);
    /* Wait for address transfer to complete */
    while (!(I2C0->S & I2C_S_IICIF_MASK)) {
     if (!(busy_counter--))
        return I2C_BUSY;
     }
    I2C0->S |= I2C_S_IICIF_MASK;
    /* Clear TX/RX bit in order to set receive mode */
    I2C0->C1 &= ~I2C_C1_TX_MASK;
    if (i2c_buffer.length == 1) {
      /* Disable Acknowledge, generate STOP after next byte transfer */
      I2C0->C1 |= I2C_C1_TXAK_MASK;
    }
    /* Dummy read of IBDR to signal the module is ready for the next byte */
    dummy_read = I2C0->D;
    /* Receive data from slave */
    while (i2c_buffer.length > 0) {
      /* Wait for transfer to complete */
      while (!(I2C0->S & I2C_S_IICIF_MASK)) {
     if (!(busy_counter--))
        return I2C_BUSY;
       }
      I2C0->S |= I2C_S_IICIF_MASK;
      /* Check for second-to-last and last byte transmission.  After second-to-last
			 byte is received, it is required to disable the ACK bit in order to signal
			 to the slave that the last byte has been received.  The actual NAck does not
			 take place until after the last byte has been received. */
      if (i2c_buffer.length == 2) {
        /* Disable Acknowledge, generate STOP after next byte transfer */
        I2C0->C1 |= I2C_C1_TXAK_MASK;
      }

      if (i2c_buffer.length == 1) {
        // Generate STOP
        I2C0->C1 &= ~I2C_C1_MST_MASK;
        while ((I2C0->S & I2C_S_BUSY_MASK)) {
      if (!(busy_counter--))
        return I2C_BUSY;
         }
      }
      /* Store received data in RX buffer */
      *i2c_buffer.buf++ = I2C0->D;
      i2c_buffer.length--;
    }
    /* Restore module to it's idle (but active) state */
    I2C0->C1 = 0x80;
    return I2C_OK;
  }
  return I2C_OK;
}
/*-----------------------PPM Timer---------------------------------------------*/
//uint16_t GetPPMTimCapture(void) {
//  return (uint16_t)TPM2->CONTROLS[1].CnV;
//}
uint32_t GetPPMOutState(void) {
  return PTA->PDIR & 0x02;
}
void SetPPMTimCompare(uint16_t val) {
  TPM2->CONTROLS[0].CnV = (uint16_t)val;
}
uint16_t GetPPMTimCompare(void) {
  return (uint16_t)TPM2->CONTROLS[0].CnV;
}
uint32_t GetPPMTimCompareInterruptFlag(void) {
  return TPM2->CONTROLS[0].CnSC & TPM_CnSC_CHF_MASK;
}
void ClearPPMTimCompareInterruptFlag(void) {
  TPM2->CONTROLS[0].CnSC |= TPM_CnSC_CHF_MASK;
}
void EnablePPMTim(void) {
  TPM2->SC = (TPM_SC_CMOD(0x01) | TPM_SC_PS(0x04));
}
void DisablePPMTim(void) {
  TPM2->SC = 0x00;
}
void EnablePPMOut(void){
  TPM2->CONTROLS[0].CnSC = (TPM_CnSC_CHIE_MASK | TPM_CnSC_MSA_MASK | TPM_CnSC_ELSA_MASK); // Compare
}
void DisablePPMOut(void){
  TPM2->CONTROLS[0].CnSC = 0x00; // Compare
}
/*----------------------PRT Timer----------------------------------------------*/
uint16_t GetPRTTimVal(void) {
  return (uint16_t)TPM0->CNT;
}
void EnablePRTTim(void) {
  TPM0->SC = (TPM_SC_CMOD(0x01) | TPM_SC_PS(0x04) | TPM_SC_TOIE_MASK);
}
void DisablePRTTim(void) {
  TPM0->SC = 0x00;
}

void SetPRTTimPeriod(uint8_t prot) {
  switch (prot) {
  case PROTO_AFHDS2A:
    TPM0->MOD = TPM_MOD_MOD(11579);
    break;
  case PROTO_AFHDS:
    TPM0->MOD = TPM_MOD_MOD(/*0x11B1*/4499);
    break;
#ifdef PROTO_ELRS1
  case PROTO_ELRS1:
#endif
  case PROTO_ELRS2:
    TPM0->MOD = TPM_MOD_MOD(CROSSFIRE_PERIOD*3000-1);
    break;

  default:
    break;
  }
}
/*----------------------------------------------------------------------------*/
uint32_t GetChipID(void){
    return SIM->UIDMH ^ SIM->UIDML ^ SIM->UIDL ^ SIM->SDID;
}
/*----------------------------------------------------------------------------*/
void sei(void) {
  __enable_irq();
}
void cli(void) {
  __disable_irq();
}
/*----------------------------------------------------------------------------*/

/******************************************************************************/
/*                    Interrupt handlers                                      */
/******************************************************************************/

/*--------------handler for Timer16kHz and Timer10ms--------------------------*/
void PIT_IRQHandler(void) {
  if (PIT->CHANNEL[0].TFLG & PIT_TFLG_TIF_MASK) { //16kHz
    PIT->CHANNEL[0].TFLG = PIT_TFLG_TIF_MASK;     //Clear IF
    g_tmr16KHz++;
    if (tmrEEPROM != 0xFFFF)
      tmrEEPROM++;
  }
  if (PIT->CHANNEL[1].TFLG & PIT_TFLG_TIF_MASK) { //10ms
    PIT->CHANNEL[1].TFLG = PIT_TFLG_TIF_MASK;     //Clear IF
    ISR_TIMER0_COMP_vect();
  }
}
/*--------------handler for AudioTimer 128uS----------------------------------*/
void TPM1_IRQHandler(void) {
  if (TPM1->SC & TPM_SC_TOF_MASK) {
    TPM1->SC |= TPM_SC_TOF_MASK;
    ISR_TIMER2_OVF_vect();
  }
}
/*-------------handler for RADIO SPI------------------------------------------*/
void SPI0_IRQHandler(void) {
  uint8_t status = SPI0->S;
  if (SPI0->C1 & SPI_C1_SPTIE_MASK) {
    SPI0->DL = *SPI_BufferPtr++;
    SPI_Size--;
    if (SPI_Size == 0) {
      SPI0->C1 &= ~SPI_C1_SPTIE_MASK;    // disable transmit irq
    }
  }

  if (SPI0->C1 & SPI_C1_SPIE_MASK) {
    *SPI_BufferPtr++ = SPI0->DL;
    SPI_Size--;
    if (SPI_Size == 0) {
      SPI0->C1 &= ~SPI_C1_SPIE_MASK;    // disable receive irq
      return;
    }
    SPI0->DL = 0x00;
  }
}
  /*-------------handler for RADIO GIO1 (FALLING  EDGE)---------------------------*/
  void PORTC_PORTD_IRQHandler(void) {
    if (PORTD->ISFR & PORT_ISFR_ISF(GIO1_PIN_MASK)) {
      PORTD->ISFR = PORT_ISFR_ISF(GIO1_PIN_MASK);    // Clear IF
      DisableGIO();
      SETBIT(RadioState, CALLER, GPIO_CALL);
      ActionAFHDS2A();
    }
  }
  /*------------handler for Radio_Protocol_Timer 3860 or 1500 uS-----------------*/
  void TPM0_IRQHandler(void) {
    if (TPM0->SC & TPM_SC_TOF_MASK) {
      TPM0->SC |= TPM_SC_TOF_MASK;

      if (PausePulses) {
        return;
      }
      switch (g_model.protocol) {
      case PROTO_AFHDS2A:
        SETBIT(RadioState, CALLER, TIM_CALL);
        ActionAFHDS2A();
        break;
      case PROTO_AFHDS:
        ActionAFHDS();
        break;
#ifdef PROTO_ELRS1
      case PROTO_ELRS1:
#endif
      case PROTO_ELRS2:
        crsf_action();
        break;
      default:
        break;
      }
    }
  }
  /*------------handler for PPM Timer-------------------------------------------*/
  void TPM2_IRQHandler(void) {
    if (TPM2->CONTROLS[0].CnSC & TPM_CnSC_CHF_MASK) {    // Compare PPM-OUT
      TPM2->CONTROLS[0].CnSC |= TPM_CnSC_CHF_MASK;
      ISR_TIMER1_COMPA_vect();
    }

    //if (TPM2->CONTROLS[1].CnSC & TPM_CnSC_CHF_MASK) {    // Capture PPM-IN
    //  TPM2->CONTROLS[1].CnSC |= TPM_CnSC_CHF_MASK;
    //    ISR_TIMER3_CAPT_vect();
    //}
  }


#ifdef USE_LORA_SPI 
  
void lora_csn_on()
{
FPTD->PSOR = LORA_SCN_PORT_MASK;

}
void lora_csn_off()
{
FPTD->PCOR = LORA_SCN_PORT_MASK;
}


void lora_tx_switch(int s)
{

}

void lora_rx_switch(int s)
{

}

void lora_pa_switch(int s)
{

}
void lora_reset_switch(int s)
{

}


void lora_spi_set_frequency(uint32_t freq)
{

}



void lora_attach_gio_isr(LORA_GIO_ISR_FUNC isr)
{
  
}
void lora_detach_gio_isr()
{

}


#endif


#if 0
enum _uart_interrupt_enable
{
#if defined(FSL_FEATURE_UART_HAS_LIN_BREAK_DETECT) && FSL_FEATURE_UART_HAS_LIN_BREAK_DETECT
    kUART_LinBreakInterruptEnable = (UART_BDH_LBKDIE_MASK), /*!< LIN break detect interrupt. */
#endif
    kUART_RxActiveEdgeInterruptEnable = (UART_BDH_RXEDGIE_MASK),   /*!< RX active edge interrupt. */
    kUART_TxDataRegEmptyInterruptEnable = (UART_C2_TIE_MASK << 8), /*!< Transmit data register empty interrupt. */
    kUART_TransmissionCompleteInterruptEnable = (UART_C2_TCIE_MASK << 8), /*!< Transmission complete interrupt. */
    kUART_RxDataRegFullInterruptEnable = (UART_C2_RIE_MASK << 8),         /*!< Receiver data register full interrupt. */
    kUART_IdleLineInterruptEnable = (UART_C2_ILIE_MASK << 8),             /*!< Idle line interrupt. */
    kUART_RxOverrunInterruptEnable = (UART_C3_ORIE_MASK << 16),           /*!< Receiver overrun interrupt. */
    kUART_NoiseErrorInterruptEnable = (UART_C3_NEIE_MASK << 16),          /*!< Noise error flag interrupt. */
    kUART_FramingErrorInterruptEnable = (UART_C3_FEIE_MASK << 16),        /*!< Framing error flag interrupt. */
    kUART_ParityErrorInterruptEnable = (UART_C3_PEIE_MASK << 16),         /*!< Parity error flag interrupt. */
#if defined(FSL_FEATURE_UART_HAS_FIFO) && FSL_FEATURE_UART_HAS_FIFO
    kUART_RxFifoOverflowInterruptEnable = (UART_CFIFO_RXOFE_MASK << 24),  /*!< RX FIFO overflow interrupt. */
    kUART_TxFifoOverflowInterruptEnable = (UART_CFIFO_TXOFE_MASK << 24),  /*!< TX FIFO overflow interrupt. */
    kUART_RxFifoUnderflowInterruptEnable = (UART_CFIFO_RXUFE_MASK << 24), /*!< RX FIFO underflow interrupt. */
#endif
    kUART_AllInterruptsEnable =
#if defined(FSL_FEATURE_UART_HAS_LIN_BREAK_DETECT) && FSL_FEATURE_UART_HAS_LIN_BREAK_DETECT
      kUART_LinBreakInterruptEnable |
#endif
      kUART_RxActiveEdgeInterruptEnable | kUART_TxDataRegEmptyInterruptEnable |
      kUART_TransmissionCompleteInterruptEnable | kUART_RxDataRegFullInterruptEnable |
      kUART_IdleLineInterruptEnable | kUART_RxOverrunInterruptEnable | kUART_NoiseErrorInterruptEnable |
      kUART_FramingErrorInterruptEnable | kUART_ParityErrorInterruptEnable
#if defined(FSL_FEATURE_UART_HAS_FIFO) && FSL_FEATURE_UART_HAS_FIFO
      | kUART_RxFifoOverflowInterruptEnable | kUART_TxFifoOverflowInterruptEnable
      | kUART_RxFifoUnderflowInterruptEnable
#endif
        ,
};


void UART_EnableInterrupts(UART_Type *base, uint32_t mask)
{
    mask &= kUART_AllInterruptsEnable;

    /* The interrupt mask is combined by control bits from several register: ((CFIFO<<24) | (C3<<16) | (C2<<8) |(BDH))
     */
    base->BDH |= mask;
    base->C2 |= (mask >> 8);
    base->C3 |= (mask >> 16);

#if defined(FSL_FEATURE_UART_HAS_FIFO) && FSL_FEATURE_UART_HAS_FIFO
    base->CFIFO |= (mask >> 24);
#endif
}

void UART_DisableInterrupts(UART_Type *base, uint32_t mask)
{
    mask &= kUART_AllInterruptsEnable;

    /* The interrupt mask is combined by control bits from several register: ((CFIFO<<24) | (C3<<16) | (C2<<8) |(BDH))
     */
    base->BDH &= ~mask;
    base->C2 &= ~(mask >> 8);
    base->C3 &= ~(mask >> 16);

#if defined(FSL_FEATURE_UART_HAS_FIFO) && FSL_FEATURE_UART_HAS_FIFO
    base->CFIFO &= ~(mask >> 24);
#endif
}
#endif


static crsf_read_cb_t g_crsf_read_callback = NULL;

#if USE_DMA_UART


#define crsf_rx_buf_size  64

#define TX_DMA_CHANNEL 0
#define VOICE_DMA_TX_CH 1
#define VOICE_DMA_RX_CH 2


static uint8_t g_crsf_rx_buf[crsf_rx_buf_size];

lpsci_dma_handle_t  g_lpsci_dma_handle;
dma_handle_t        g_lpsci_tx_dma_handle;
dma_handle_t        g_lpsci_rx_dma_handle;
lpsci_transfer_t    g_lpsci_transfer_rx ;

void set_lpsci_direction(bool bTx)
{
    DMAMUX_DisableChannel(DMAMUX0,TX_DMA_CHANNEL);
    DMAMUX_SetSource(DMAMUX0,TX_DMA_CHANNEL,bTx?kDmaRequestMux0UART0Tx:kDmaRequestMux0UART0Rx);
    if(bTx)
    {
         DMAMUX_EnablePeriodTrigger(DMAMUX0,TX_DMA_CHANNEL);
     }
     else
     {
        DMAMUX_DisablePeriodTrigger(DMAMUX0,TX_DMA_CHANNEL);
     }
    DMAMUX_EnableChannel(DMAMUX0,TX_DMA_CHANNEL);
    if(bTx)
    {
        UART0->C3 |= UART_C3_TXDIR_MASK;
        LPSCI_EnableTx(UART0,true);
        LPSCI_EnableRx(UART0,false);
    }
    else 
    {
        UART0->C3 &= ~(UART_C3_TXDIR_MASK); //SET AS READ DIR
        LPSCI_EnableTx(UART0,false);
        LPSCI_EnableRx(UART0,true);
    }
}



void submit_and_start_dma_transfer(DMA_Type * base, int channel, uint32_t src, uint32_t dst, uint32_t len, bool sinc, bool dinc)
{
    
    /* clear all status bit */
    base->DMA[channel].DSR_BCR |= DMA_DSR_BCR_DONE(true);
    /* clear all registers */
    base->DMA[channel].SAR = 0;
    base->DMA[channel].DAR = 0;
    base->DMA[channel].DSR_BCR = 0;
    /* enable cycle steal and enable auto disable channel request */
    base->DMA[channel].DCR = DMA_DCR_D_REQ(true) | DMA_DCR_CS(true);


      uint32_t tmpreg;

    /* Set source address */
    base->DMA[channel].SAR = src;
    /* Set destination address */
    base->DMA[channel].DAR =  dst;
    /* Set transfer bytes */
    base->DMA[channel].DSR_BCR = DMA_DSR_BCR_BCR(len);
    /* Set DMA Control Register */
    tmpreg = base->DMA[channel].DCR;
    tmpreg &= ~(DMA_DCR_DSIZE_MASK | DMA_DCR_DINC_MASK | DMA_DCR_SSIZE_MASK | DMA_DCR_SINC_MASK);
    tmpreg |= (DMA_DCR_DSIZE(kDMA_Transfersize8bits) | DMA_DCR_DINC(dinc) |
               DMA_DCR_SSIZE(kDMA_Transfersize8bits) | DMA_DCR_SINC(sinc));
    base->DMA[channel].DCR = tmpreg;

    base->DMA[channel].DCR |= DMA_DCR_EINT(true);
    
    base->DMA[channel].DCR |= DMA_DCR_ERQ_MASK;   

}

void start_lpsci_DMA_receive()
{
    set_lpsci_direction(false);
  
   

     submit_and_start_dma_transfer(DMA0,TX_DMA_CHANNEL,(uint32_t)&UART0->D,(uint32_t)g_crsf_rx_buf,sizeof(g_crsf_rx_buf),0,1);
        
    /* Enable LPSCI TX DMA. */
    LPSCI_EnableRxDMA(UART0, true);


}



void setup_crsf_serial_port(uint32_t baudRate_Bps,crsf_read_cb_t read_cb)
{
    if(g_crsf_read_callback!=0)
    {
        return ;
    }

    g_crsf_read_callback = read_cb;
   
    uint8_t temp;
    uint16_t sbr = 0;
    uint16_t sbrTemp;
    uint32_t osr = 0;
    uint32_t osrTemp;
    uint32_t tempDiff, calculatedBaud, baudDiff;

    /* This LPSCI instantiation uses a slightly different baud rate calculation
     * The idea is to use the best OSR (over-sampling rate) possible
     * Note, OSR is typically hard-set to 16 in other LPSCI instantiations
     * loop to find the best OSR value possible, one that generates minimum baudDiff
     * iterate through the rest of the supported values of OSR */

    baudDiff = baudRate_Bps;
    for (osrTemp = 4; osrTemp <= 32; osrTemp++)
    {
        /* calculate the temporary sbr value   */
        sbrTemp = (DEFAULT_SYSTEM_CLOCK / (baudRate_Bps * osrTemp));
        /* set sbrTemp to 1 if the sourceClockInHz can not satisfy the desired baud rate */
        if (sbrTemp == 0)
        {
            sbrTemp = 1;
        }
        /* Calculate the baud rate based on the temporary OSR and SBR values */
        calculatedBaud = (DEFAULT_SYSTEM_CLOCK / (osrTemp * sbrTemp));

        tempDiff = calculatedBaud - baudRate_Bps;

        /* Select the better value between srb and (sbr + 1) */
        if (tempDiff > (baudRate_Bps - (DEFAULT_SYSTEM_CLOCK / (osrTemp * (sbrTemp + 1)))))
        {
            tempDiff = baudRate_Bps - (DEFAULT_SYSTEM_CLOCK / (osrTemp * (sbrTemp + 1)));
            sbrTemp++;
        }

        if (tempDiff <= baudDiff)
        {
            baudDiff = tempDiff;
            osr = osrTemp; /* update and store the best OSR value calculated*/
            sbr = sbrTemp; /* update store the best SBR value calculated*/
        }
    }

    /* Enable LPSCI clock */
    SIM->SCGC4 |= SIM_SCGC4_UART0_MASK;// Enable the clock to the selected UART

    /* Disable TX RX before setting. */
    UART0->C2 &= ~(UART0_C2_TE_MASK | UART0_C2_RE_MASK);


    /* program the osr value (bit value is one less than actual value)*/
    UART0->C4 = ((UART0->C4 & ~UART0_C4_OSR_MASK) | (osr - 1));

    /* program the sbr (divider) value obtained above*/
    UART0->BDH = ((UART0->C4 & ~UART0_BDH_SBR_MASK) | (uint8_t)(sbr >> 8));
    UART0->BDL = (uint8_t)sbr;

    /* set parity mode */
    temp = UART0->C1 & ~(UART0_C1_PE_MASK | UART0_C1_PT_MASK | UART0_C1_M_MASK);
    /* set stop bit per char */
    UART0->BDH &= ~UART0_BDH_SBNS_MASK;

    /* Configure single wire ,  8-bit mode, no parity*/
    UART0->C1  = (UART_C1_LOOPS_MASK | UART_C1_RSRC_MASK);    
    //uart inverted 
    UART0->C3 |= UART_C3_TXINV_MASK;
    UART0->S2 |= UART_S2_RXINV_MASK; 

    SIM->SCGC7 |= SIM_SCGC7_DMA_MASK;   //ENABLE DMA
    SIM->SCGC6 |= SIM_SCGC6_DMAMUX_MASK; //ENABLE DMA MUX

    NVIC_EnableIRQ(DMA0_IRQn);
  
    NVIC_SetPriority(DMA0_IRQn,0);

}


bool stop_dma_receive()
{
      bool ret = UART0->C5 & UART0_C5_RDMAE_MASK;
      if(ret)
      {
          /* Disable LPSCI TX DMA. */
          LPSCI_EnableRxDMA(UART0, false);

          /* Stop transfer. */
          DMA0->DMA[TX_DMA_CHANNEL].DCR &= ~DMA_DCR_ERQ_MASK;
           /* clear all status bit */
          DMA0->DMA[TX_DMA_CHANNEL].DSR_BCR |= DMA_DSR_BCR_DONE(true);

      }
      return ret;
}



void process_read_dma_data()
{
   int nread = crsf_rx_buf_size - DMA_GetRemainingBytes(DMA0, TX_DMA_CHANNEL);

  if(nread > 0)
  {
      for(int i=0;i<nread ;i ++)
      {
          g_crsf_read_callback(g_crsf_rx_buf[i]);
      }
  }
}



void crsf_send_data(uint8_t *buf, uint32_t len)
{


    if(stop_dma_receive())
    {
        process_read_dma_data();
    }

    
    //SET DIR
    set_lpsci_direction(true);

    submit_and_start_dma_transfer(DMA0,TX_DMA_CHANNEL,(uint32_t)buf, (uint32_t)&UART0->D,len,1,0);

        /* Enable LPSCI TX DMA. */
    LPSCI_EnableTxDMA(UART0, true);


}

void DMA0_IRQHandler()
{
     DMA0->DMA[TX_DMA_CHANNEL].DSR_BCR |= DMA_DSR_BCR_DONE(true);
    if(crsf_is_sending())
    {
     
        /* Disable LPSCI TX DMA. */
        LPSCI_EnableTxDMA(UART0, false);

        /* Disable interrupt. */
        DMA_DisableInterrupts(DMA0, TX_DMA_CHANNEL);

        
    }
    else
    {
           /* Disable LPSCI RX DMA. */
        LPSCI_EnableRxDMA(UART0, false);
          /* Disable interrupt. */
        DMA_DisableInterrupts(DMA0, TX_DMA_CHANNEL);
        process_read_dma_data();
    }
     
     start_lpsci_DMA_receive();

}
void shutdown_crsf_serial_port()
{
  if(  g_crsf_read_callback != NULL)
  {
     g_crsf_read_callback = NULL;
     //UART0->C4 &= ~UART_C4_TDMAS_MASK;
     //UART0->C2 &= ~UART_C2_TIE_MASK;
     //DMA0->DMA[TX_DMA_CHANNEL].DCR &= ~DMA_DCR_EINT_MASK;
     //NVIC_DisableIRQ(DMA0_IRQn);
     //SIM->SCGC7 &= ~SIM_SCGC7_DMA_MASK;//DISABLE DMA
     //SIM->SCGC6 &= ~SIM_SCGC6_DMAMUX_MASK; //DISABLE DMA MUX
     //SIM->SCGC4 &=~SIM_SCGC4_UART0_MASK;
  }
}

#else 


void setup_crsf_serial_port(uint32_t baud,crsf_read_cb_t read_cb)
{

  if(g_crsf_read_callback==0)
  {
        switch_elrs_tx(1);
        g_crsf_read_callback = read_cb;


      //uart_single_init(bdrate,DEFAULT_SYSTEM_CLOCK,bdrate);//Set single wire mode.
      SIM->SCGC4 |= SIM_SCGC4_UART0_MASK;// Enable the clock to the selected UART
  
      /*
      Make sure that the transmitter and receiver are disabled while we 
         * change settings.*/
      UART0->C2 &= ~(UART_C2_TE_MASK|UART_C2_RE_MASK);
        /* Configure single wire ,  8-bit mode, no parity*/
      UART0->C1  |= (UART_C1_LOOPS_MASK | UART_C1_RSRC_MASK);    
   
      /* Calculate baud settings */
      uint16_t  sbr = (uint16_t)(DEFAULT_SYSTEM_CLOCK/(baud *16));
  
      /* Save off the current value of the UARTx_BDH except for the SBR field */
        /* Save off the current value of the UARTx_BDH except for the SBR field */

        uint8_t temp =   UART0->BDH & (~UART_BDH_SBR(0x1F)); 
        UART0->BDH= temp |  UART_BDH_SBR(((sbr & 0x1F00) >> 8));
        UART0->BDL = (uint8_t)(sbr & UART_BDL_SBR_MASK);

        #if 0
        /* Determine if a fractional divider is needed to get closer to the baud rate */
        uint16_t brfa = (((DEFAULT_SYSTEM_CLOCK*32000)/(baud * 16)) - (sbr * 32));

        /* Save off the current value of the UARTx_C4 register except for the BRFA field */
        temp = UART0->C4 & ~(0x1F);
        UART0->C4 = temp |  brfa;    
        #endif

        /* Enable receiver and transmitter, and enable receive */
        UART0->C2 |= (UART_C2_RE_MASK);
    
        //uart inverted 
        UART0->C3 |= UART_C3_TXINV_MASK;
        UART0->S2 |= UART_S2_RXINV_MASK;
        NVIC_EnableIRQ(UART0_IRQn);
    }
    
}
bool uart_clear_error()
{
   bool has_error = false;
  if(UART0->S1 & UART_S1_OR_MASK  )
    {
        //To clear OR,write a logic 1 to the OR flag.
        UART0->S1  |= UART_S1_OR_MASK;
        has_error = true ;
    }
    //clear all 
    if(  UART0->S1 & UART_S1_FE_MASK  )
    {
        UART0->S1  |= UART_S1_FE_MASK;
         has_error = true ;
    }

   if(  UART0->S1 & UART_S1_PF_MASK  )
    {
        UART0->S1  |= UART_S1_PF_MASK;
         has_error = true ;
    }

    
   if(  UART0->S1 & UART_S1_NF_MASK  )
    {
        UART0->S1  |=UART_S1_NF_MASK;
         has_error = true ;
    }
    return has_error;
}

uint8_t  *g_crsf_send_buf= 0;
uint32_t g_crsf_send_len = 0;
uint32_t g_crsf_send_pos = 0;


void crsf_send_data(uint8_t *buf, uint32_t len)
{


uint8_t dat;
  cli();
    //while(!(UART0->S1&UART_S1_TDRE_MASK));
    //while(!(UART0->S1 & UART_S1_TC_MASK));
  //  PTE->PDDR |= 1<<16; 
    UART0->C2 &=~(UART_C2_RIE_MASK|UART_C2_RE_MASK);
  //  UART0->C3 &=~(UART_C3_ORIE_MASK);
    UART0->C2 |= (UART_C2_TE_MASK);

    UART0->C3 |= UART_C3_TXDIR_MASK;
#if USE_IE_UART_TX
    g_crsf_send_buf = buf;
    g_crsf_send_len = len;
    g_crsf_send_pos =0;
    UART0->C2 |= UART_C2_TIE_MASK;
#else
    for(int i=0;i<len;i++)
    {
      while(!(UART0->S1&UART_S1_TDRE_MASK));
      UART0->D = buf[i];
    
    }
    
    while(!(UART0->S1 & UART_S1_TC_MASK))
    {
    }
    
   
     UART0->C3 &= ~UART_C3_TXDIR_MASK;
      UART0->C2 &= ~(UART_C2_TE_MASK);
    /* Configure the module1 TXD pin as an input */
 //   PTE->PDDR &= ~(1<<16); 
 //   UART0->C3 &= ~UART_C3_TXDIR_MASK;
    UART0->C2 |=UART_C2_RIE_MASK|UART_C2_RE_MASK;
 #endif
    // UART_EnableInterrupts(UART0, kUART_RxDataRegFullInterruptEnable| kUART_RxOverrunInterruptEnable);
     sei();
}

#if 1
void UART0_IRQHandler(void )
{
#if USE_IE_UART_TX
     if(!UART0->C3 & UART_C3_TXDIR_MASK)
  #endif
    {
      uint8_t dat;
      if(uart_clear_error())
      {
        dat = UART0->D;
        return;
      }
    }

    if(UART0->S1 & UART_S1_RDRF_MASK)
    {
        uint8_t dat = UART0->D;
        if(g_crsf_read_callback)
        {
            (*g_crsf_read_callback)(dat);
        }

    }
 #if USE_IE_UART_TX
    else if(UART0->S1 & UART_S1_TDRE_MASK)
    {
         if(g_crsf_send_pos < g_crsf_send_len)
         {
            UART0->D = g_crsf_send_buf[g_crsf_send_pos++];
         }
         else{
             while(!(UART0->S1 & UART_S1_TC_MASK))
            {
            }
    
           UART0->C2 &=  ~UART_C2_TIE_MASK;
           UART0->C3 &= ~UART_C3_TXDIR_MASK;
           UART0->C2 |=UART_C2_RIE_MASK;
         }
       
    }
 #endif
  
 
}
#endif

void shutdown_crsf_serial_port()
{
  if(  g_crsf_read_callback != NULL)
  {
     switch_elrs_tx(0);
     g_crsf_read_callback = NULL;
     UART0->C2 &= ~(UART_C2_RE_MASK|UART_C2_RIE_MASK|UART_C2_TE_MASK|UART_C2_TIE_MASK);
  //   UART0->C3 &=~(UART_C3_ORIE_MASK);
     NVIC_DisableIRQ(UART0_IRQn);
     SIM->SCGC4 &=~SIM_SCGC4_UART0_MASK;
  }
}
#endif

#if USE_IE_UART_TX ||USE_DMA_UART
int  crsf_is_sending()
{
    if(UART2->C3 & UART_C3_TXDIR_MASK)
        return 1;
    return 0;//Logic level 1
}
#endif



void switch_elrs_tx(int val)
{

if(val)
  FPTD->PSOR = LORA_SCN_PORT_MASK;
else
  FPTD->PCOR = LORA_SCN_PORT_MASK;

}



voice_serial_read_cb_t g_voice_cb = 0;

uint8_t voice_uart_recv_buf[10];
uint8_t voice_rx_idx = 0;



#if 0




void process_voice_read_dma_data()
{
   int nread = sizeof(voice_uart_recv_buf) - DMA_GetRemainingBytes(DMA0, VOICE_DMA_RX_CH);

  if(nread > 0)
  {
      for(int i=0;i<nread ;i ++)
      {
          g_voice_cb(voice_uart_recv_buf,sizeof(voice_uart_recv_buf));
      }
  }
}


void start_uart_dma_receive()
{
    submit_and_start_dma_transfer(DMA0,VOICE_DMA_RX_CH,(uint32_t)&UART2->D,(uint32_t)voice_uart_recv_buf,sizeof(voice_uart_recv_buf),0,1);    
     /* Enable LPSCI TX DMA. */
    UART_EnableRxDMA(UART2, true);
}
void  serialVoiceInit(voice_serial_read_cb_t cb)
{

    if(g_voice_cb==0)
    {
       g_voice_cb = cb;
    //uart_single_init(bdrate,DEFAULT_SYSTEM_CLOCK,bdrate);//Set single wire mode.
      SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;// Enable the clock to the selected UART
      /*
      Make sure that the transmitter and receiver are disabled while we 
         * change settings.*/
      UART2->C2 &= ~(UART_C2_TE_MASK|UART_C2_RE_MASK);
        /* Configure single wire ,  8-bit mode, no parity*/
      // UART2->C1  |= (UART_C1_LOOPS_MASK);    
    //  UART2->C1   = 0;

      /* Calculate baud settings */
      uint16_t  sbr = (uint16_t)(DEFAULT_SYSTEM_CLOCK/2/(9600 *16));
  
      /* Save off the current value of the UARTx_BDH except for the SBR field */
        /* Save off the current value of the UARTx_BDH except for the SBR field */

        uint8_t temp =   UART2->BDH & (~UART_BDH_SBR(0x1F)); 
        UART2->BDH= temp |  UART_BDH_SBR(((sbr & 0x1F00) >> 8));
        UART2->BDL = (uint8_t)(sbr & UART_BDL_SBR_MASK);

       

        SIM->SCGC7 |= SIM_SCGC7_DMA_MASK;   //ENABLE DMA
        SIM->SCGC6 |= SIM_SCGC6_DMAMUX_MASK; //ENABLE DMA MUX
        

       DMAMUX_DisableChannel(DMAMUX0,VOICE_DMA_RX_CH);
       DMAMUX_DisableChannel(DMAMUX0,VOICE_DMA_TX_CH);
       DMAMUX_SetSource(DMAMUX0,VOICE_DMA_RX_CH,kDmaRequestMux0UART2Rx);
       DMAMUX_SetSource(DMAMUX0,VOICE_DMA_TX_CH,kDmaRequestMux0UART2Tx);

         DMAMUX_DisablePeriodTrigger(DMAMUX0,VOICE_DMA_RX_CH);
       DMAMUX_DisablePeriodTrigger(DMAMUX0,VOICE_DMA_TX_CH);
     

       DMAMUX_EnableChannel(DMAMUX0,VOICE_DMA_RX_CH);
       DMAMUX_EnableChannel(DMAMUX0,VOICE_DMA_TX_CH);

      /* Enable receiver and transmitter, and enable receive */
       UART2->C2 |= (UART_C2_TE_MASK|UART_C2_RE_MASK);

       NVIC_EnableIRQ(DMA1_IRQn);
       NVIC_EnableIRQ(DMA2_IRQn);

     
   }
}


void DMA1_IRQHandler()
{
    DMA0->DMA[VOICE_DMA_TX_CH].DSR_BCR |= DMA_DSR_BCR_DONE(true);
    /* Disable LPSCI TX DMA. */
    UART_EnableTxDMA(UART2, false);
    /* Disable interrupt. */
    DMA_DisableInterrupts(DMA0, VOICE_DMA_TX_CH);

   
}


void DMA2_IRQHandler()
{
  

    /* Stop transfer. */
    //DMA0->DMA[VOICE_DMA_RX_CH].DCR &= ~DMA_DCR_ERQ_MASK;


    DMA0->DMA[VOICE_DMA_RX_CH].DSR_BCR |= DMA_DSR_BCR_DONE(true);

       /* Disable LPSCI RX DMA. */
    UART_EnableRxDMA(UART2, false);
      /* Disable interrupt. */
    DMA_DisableInterrupts(DMA0, VOICE_DMA_RX_CH);
    process_voice_read_dma_data();

    //DO IT AGAIN
    //start_uart_dma_receive();
}

void sendSerialVoiceData(uint8_t *buf, uint32_t len)
{
     submit_and_start_dma_transfer(DMA0,VOICE_DMA_TX_CH,(uint32_t)buf, (uint32_t)&UART2->D,len,1,0);
     UART_EnableTxDMA(UART2,true);
      start_uart_dma_receive();
}


#else

uint8_t voice_uart_send_buf[10];
uint8_t voice_tx_idx = 0;
void  serialVoiceInit(voice_serial_read_cb_t cb)
{

    if(g_voice_cb==0)
    {
     g_voice_cb = cb;
    //uart_single_init(bdrate,DEFAULT_SYSTEM_CLOCK,bdrate);//Set single wire mode.
      SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;// Enable the clock to the selected UART
      /*
      Make sure that the transmitter and receiver are disabled while we 
         * change settings.*/
      UART2->C2 &= ~(UART_C2_TE_MASK|UART_C2_RE_MASK);
        /* Configure single wire ,  8-bit mode, no parity*/
      // UART2->C1  |= (UART_C1_LOOPS_MASK);    
    //  UART2->C1   = 0;

      /* Calculate baud settings */
      uint16_t  sbr = (uint16_t)(DEFAULT_SYSTEM_CLOCK/2/(9600 *16));
  
      /* Save off the current value of the UARTx_BDH except for the SBR field */
        /* Save off the current value of the UARTx_BDH except for the SBR field */

        uint8_t temp =   UART2->BDH & (~UART_BDH_SBR(0x1F)); 
        UART2->BDH= temp |  UART_BDH_SBR(((sbr & 0x1F00) >> 8));
        UART2->BDL = (uint8_t)(sbr & UART_BDL_SBR_MASK);

      
      
        NVIC_EnableIRQ(UART2_IRQn);
          /* Enable receiver and transmitter, and enable receive */
        UART2->C2 |= (UART_C2_TE_MASK|UART_C2_RE_MASK|UART_C2_RIE_MASK);

        UART2->C2 |= UART_C2_RIE_MASK;
    }

}


void stopSerialVoice( void )
{

  if(g_voice_cb)
  {
       g_voice_cb = NULL;
      UART2->C2 &= ~(UART_C2_RE_MASK|UART_C2_RIE_MASK|UART_C2_TE_MASK|UART_C2_TIE_MASK);
       UART2->C3 &=~(UART_C3_ORIE_MASK);
       NVIC_DisableIRQ(UART2_IRQn);
       SIM->SCGC4 &=~SIM_SCGC4_UART2_MASK;
  }
}

void sendSerialVoiceData(uint8_t *buf, uint32_t len)
{


    memcpy(voice_uart_send_buf,buf,10);
    voice_tx_idx = 0;
    UART2->C2 |= UART_C2_TIE_MASK;

    //for(int i=0;i<len;i++)
    //{
    //  while(!(UART2->S1&UART_S1_TDRE_MASK));
    //  UART2->D = buf[i];
    
    //}
    
    //while(!(UART2->S1 & UART_S1_TC_MASK))
    //{
      
    //}
}


bool uart2_clear_error()
{
   bool has_error = false;
  if(UART2->S1 & UART_S1_OR_MASK  )
    {
        //To clear OR,write a logic 1 to the OR flag.
        UART2->S1  |= UART_S1_OR_MASK;
        has_error = true ;
    }
    //clear all 
    if(  UART2->S1 & UART_S1_FE_MASK  )
    {
        UART2->S1  |= UART_S1_FE_MASK;
         has_error = true ;
    }

   if(  UART2->S1 & UART_S1_PF_MASK  )
    {
        UART2->S1  |= UART_S1_PF_MASK;
         has_error = true ;
    }

    
   if(  UART2->S1 & UART_S1_NF_MASK  )
    {
        UART2->S1  |=UART_S1_NF_MASK;
         has_error = true ;
    }
    return has_error;
}


void UART2_IRQHandler(void)
{
    //uart2_clear_error();

    if(UART2->S1 & UART_S1_RDRF_MASK)
    {
        uint8_t dat = UART2->D;
        voice_uart_recv_buf[voice_rx_idx++] = dat;
        if(voice_rx_idx==10)
        {
             voice_rx_idx = 0;
            if(g_voice_cb) g_voice_cb(voice_uart_recv_buf,10);
        }
    }
    else if(UART2->S1 & UART_S1_TDRE_MASK)
    {
         if(voice_tx_idx < 10)
         {
            UART2->D = voice_uart_send_buf[voice_tx_idx++];
         }
         else{
            UART2->C2 &=  ~UART_C2_TIE_MASK;
         }
       
    }
}

#endif