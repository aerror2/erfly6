
#include "BoardI6X.h"
#include "../er9x.h"
#include "../hal.h"
#include "../iface_a7105.h"

char TX_name[] = "FlySky-I6X";  

void HW_Init(void) {
  /*------------------------------SystemClock_Config----------------------------*/
  MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, FLASH_ACR_LATENCY);
  while (READ_BIT(FLASH->ACR, FLASH_ACR_LATENCY) != FLASH_ACR_LATENCY) {
  }
  // RCC_HSE_Enable
  SET_BIT(RCC->CR, RCC_CR_HSEON);
  /* Wait till HSE is ready */
  while ((READ_BIT(RCC->CR, RCC_CR_HSERDY) == (RCC_CR_HSERDY)) != 1) {
  }
  // RCC_PLL_ConfigDomain_SYS
  MODIFY_REG(RCC->CFGR, RCC_CFGR_PLLSRC | RCC_CFGR_PLLMUL, RCC_CFGR_PLLSRC_HSE_PREDIV | RCC_CFGR_PLLMUL6);
  MODIFY_REG(RCC->CFGR2, RCC_CFGR2_PREDIV, RCC_CFGR2_PREDIV_DIV1);
  // RCC_PLL_Enable
  SET_BIT(RCC->CR, RCC_CR_PLLON);
  /* Wait till PLL is ready */
  while ((READ_BIT(RCC->CR, RCC_CR_PLLRDY) == (RCC_CR_PLLRDY)) != 1) {
  }
  // RCC_SetAHBPrescaler
  MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE, RCC_CFGR_HPRE_DIV1);
  // RCC_SetAPB1Prescaler
  MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE, RCC_CFGR_PPRE_DIV1);
  // RCC_SetSysClkSource
  MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_PLL);
  /* Wait till System clock is ready */
  while (READ_BIT(RCC->CFGR, RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {
  }
  /*------------------------------SysTick_Init---------------------------------*/
  SysTick->LOAD = (uint32_t)((48000000U / 1000U) - 1UL);                /* set reload register */
  SysTick->VAL  = 0UL;                                                  /* Load the SysTick Counter Value */
  SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk; /* Enable the Systick Timer */
  /*-------------------- GPIO Ports Clock Enable-------------------------------*/
  __IO uint32_t tmpreg;
  SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOAEN);
  /* Delay after an RCC peripheral clock enabling */
  tmpreg = READ_BIT(RCC->AHBENR, RCC_AHBENR_GPIOAEN);

  SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOBEN);
  /* Delay after an RCC peripheral clock enabling */
  tmpreg = READ_BIT(RCC->AHBENR, RCC_AHBENR_GPIOBEN);

  SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOCEN);
  /* Delay after an RCC peripheral clock enabling */
  tmpreg = READ_BIT(RCC->AHBENR, RCC_AHBENR_GPIOCEN);

  SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIODEN);
  /* Delay after an RCC peripheral clock enabling */
  tmpreg = READ_BIT(RCC->AHBENR, RCC_AHBENR_GPIODEN);

  SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOEEN);
  /* Delay after an RCC peripheral clock enabling */
  tmpreg = READ_BIT(RCC->AHBENR, RCC_AHBENR_GPIOEEN);

  SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOFEN);
  /* Delay after an RCC peripheral clock enabling */
  tmpreg = READ_BIT(RCC->AHBENR, RCC_AHBENR_GPIOFEN);

  /*------------------------------LCD_Control_Init-----------------------------*/
  //LCD_DATA output
  LCD_DATA_GPIO_PORT->MODER |= GPIO_MODER_MODER0_0 |
                               GPIO_MODER_MODER1_0 |
                               GPIO_MODER_MODER2_0 |
                               GPIO_MODER_MODER3_0 |
                               GPIO_MODER_MODER4_0 |
                               GPIO_MODER_MODER5_0 |
                               GPIO_MODER_MODER6_0 |
                               GPIO_MODER_MODER7_0 ;
  
  //LCD_RS output
  LCD_RS_GPIO_PORT->MODER |= GPIO_MODER_MODER3_0;
  //LCD_RD output
  LCD_RD_GPIO_PORT->MODER |= GPIO_MODER_MODER7_0;
  //LCD_CS output
  LCD_CS_GPIO_PORT->MODER |= GPIO_MODER_MODER2_0;
  //LCD_RST output
  LCD_RST_GPIO_PORT->MODER |= GPIO_MODER_MODER4_0;
  //LCD_RW output
  LCD_RW_GPIO_PORT->MODER |= GPIO_MODER_MODER5_0;
  //BACK_LIGHT output
  BACK_LIGHT_GPIO_PORT->MODER |= GPIO_MODER_MODER3_0;
  /*------------------------------ADC_Init-----------------------------*/
  /* ADC clock enable */
  SET_BIT(RCC->APB2ENR, RCC_APB2ENR_ADC1EN);
  /* Delay after an RCC peripheral clock enabling */
  tmpreg = READ_BIT(RCC->APB2ENR, RCC_APB2ENR_ADC1EN);

  ADC_AIL_GPIO_PORT->MODER |= GPIO_MODER_MODER0;
  ADC_THR_GPIO_PORT->MODER |= GPIO_MODER_MODER2;
  ADC_ELE_GPIO_PORT->MODER |= GPIO_MODER_MODER1;
  ADC_RUD_GPIO_PORT->MODER |= GPIO_MODER_MODER3;
  ADC_VRA_GPIO_PORT->MODER |= GPIO_MODER_MODER6;
  ADC_VRB_GPIO_PORT->MODER |= GPIO_MODER_MODER7;
  ADC_SwC_GPIO_PORT->MODER |= GPIO_MODER_MODER0;
  ADC_BATT_GPIO_PORT->MODER |= GPIO_MODER_MODER0;

  ADC1->CHSELR = ADC_CHSELR_CHSEL0 | ADC_CHSELR_CHSEL1 | ADC_CHSELR_CHSEL2 | ADC_CHSELR_CHSEL3 |
                 ADC_CHSELR_CHSEL6 | ADC_CHSELR_CHSEL7 | ADC_CHSELR_CHSEL8 | ADC_CHSELR_CHSEL10; 

  MODIFY_REG(ADC1->CFGR1,
      ADC_CFGR1_RES | ADC_CFGR1_ALIGN | ADC_CFGR1_WAIT | ADC_CFGR1_AUTOFF,
      0x00000000U | 0x00000000U | 0x00000000U);

  MODIFY_REG(ADC1->CFGR2, ADC_CFGR2_CKMODE, 0x00000000U);

  MODIFY_REG(ADC1->CFGR1,
      ADC_CFGR1_EXTSEL | ADC_CFGR1_EXTEN | ADC_CFGR1_DISCEN | ADC_CFGR1_CONT | ADC_CFGR1_DMAEN | ADC_CFGR1_DMACFG |
          ADC_CFGR1_OVRMOD,
      0x00000000U | 0x00000000U | 0x00000000U | 0x00000000U | 0x00000000U);

  MODIFY_REG(ADC1->CFGR1, ADC_CFGR1_SCANDIR, 0x00000000U);
  MODIFY_REG(ADC1->SMPR, ADC_SMPR_SMP, ADC_SMPR_SMP_2);

  // Calibration ADC
  if ((ADC1->CR & ADC_CR_ADEN) != 0) {
    ADC1->CR &= (uint32_t)(~ADC_CR_ADEN);
  }
  ADC1->CR |= ADC_CR_ADCAL;
  while ((ADC1->CR & ADC_CR_ADCAL) != 0) {
  }
  // Enable ADC
  ADC1->CR |= ADC_CR_ADEN;
  while ((ADC1->ISR & ADC_ISR_ADRDY) == 0) {
  }

/*---------------------BUZZER output------------------------------------------*/
  BUZZER_GPIO_PORT->MODER |= GPIO_MODER_MODER8_0;
/*-----------------------BTN_L_input------------------------------------------*/

/*-----------------------BTN_R_Output-----------------------------------------*/
  BTN_R_GPIO_PORT->MODER |= GPIO_MODER_MODER6_0 |
                            GPIO_MODER_MODER7_0 |
                            GPIO_MODER_MODER8_0 ;
/*---------------------BIND input---------------------------------------------*/
/*---------------------SwA input----------------------------------------------*/
/*---------------------SwB input----------------------------------------------*/
/*---------------------SwD input----------------------------------------------*/
/*-----------------------TIMER 16kHz Init (TIM6)------------------------------*/
  /* TIM6 clock enable */
  SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM6EN);
  /* Delay after an RCC peripheral clock enabling */
  tmpreg = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM6EN);
  CLEAR_BIT(TIM6->CR1, TIM_CR1_ARPE);  // Disable ARR Preload
  CLEAR_BIT(TIM6->SMCR, TIM_SMCR_MSM); // Disable Master Slave Mode
  WRITE_REG(TIM6->PSC, 0);             // Prescaler   
  WRITE_REG(TIM6->ARR, 2999);          // Preload
  /* TIM6 interrupt Init */
  SET_BIT(TIM6->DIER, TIM_DIER_UIE);   // Enable update interrupt (UIE)
  NVIC_SetPriority(TIM6_DAC_IRQn, 2);
  NVIC_EnableIRQ(TIM6_DAC_IRQn);
  SET_BIT(TIM6->CR1, TIM_CR1_CEN);     // Enable timer counter
/*-----------------------TIMER 10ms Init (TIM7)------------------------------*/
  /* TIM7 clock enable */
  SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM7EN);
  /* Delay after an RCC peripheral clock enabling */
  tmpreg = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM7EN);
  CLEAR_BIT(TIM7->CR1, TIM_CR1_ARPE);  // Disable ARR Preload
  CLEAR_BIT(TIM7->SMCR, TIM_SMCR_MSM); // Disable Master Slave Mode
  WRITE_REG(TIM7->PSC, 7);             // Prescaler   
  WRITE_REG(TIM7->ARR, 59999);         // Preload
  /* TIM6 interrupt Init */
  SET_BIT(TIM7->DIER, TIM_DIER_UIE);   // Enable update interrupt (UIE)
  NVIC_SetPriority(TIM7_IRQn, 2);
  NVIC_EnableIRQ(TIM7_IRQn);
  SET_BIT(TIM7->CR1, TIM_CR1_CEN);     // Enable timer counter
 /*------------Audio_TIMER_Init(TIM14 Interrupt period 128 uS)-----------------*/
  /* TIM14 clock enable */
  SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM14EN);
  /* Delay after an RCC peripheral clock enabling */
  tmpreg = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM14EN);
  CLEAR_BIT(TIM14->CR1, TIM_CR1_ARPE);  // Disable ARR Preload
  CLEAR_BIT(TIM14->SMCR, TIM_SMCR_MSM); // Disable Master Slave Mode
  WRITE_REG(TIM14->PSC, 0);             // Prescaler   
  WRITE_REG(TIM14->ARR, 6143/*12284*/);          // Preload
  /* TIM6 interrupt Init */
  SET_BIT(TIM14->DIER, TIM_DIER_UIE);   // Enable update interrupt (UIE)
  NVIC_SetPriority(TIM14_IRQn, 0);
  NVIC_EnableIRQ(TIM14_IRQn);
  SET_BIT(TIM14->CR1, TIM_CR1_CEN);     // Enable timer counter
 /*------------PPM_TIMER_Init(TIM15 clock 3mHz)------------------------------*/
   /**TIM15 GPIO Configuration
  PF9   ------> TIM15_CH1
  PF10   ------> TIM15_CH2
  */
  //PF9
  PPM_IN_GPIO_PORT->MODER  |= GPIO_MODER_MODER9_1;      // Select alternate function mode
  PPM_IN_GPIO_PORT->AFR[1] |= (0x0000000U << (1 * 4));  // Select alternate function 0
  PPM_IN_GPIO_PORT->PUPDR  |= GPIO_PUPDR_PUPDR9_0;      // PullUp
  //PF10
  PPM_OUT_GPIO_PORT->MODER  |= GPIO_MODER_MODER10_1;    // Select alternate function mode
  PPM_OUT_GPIO_PORT->AFR[1] |= (0x0000000U << (2 * 4)); // Select alternate function 0

  /* TIM15 clock enable */
  SET_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM15EN);  
  /* Delay after an RCC peripheral clock enabling */
  tmpreg = READ_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM15EN);
  TIM15->PSC   = 15;                                    // Prescaler
  TIM15->CCMR1 = (TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_0); /*OCyREF toggles on compare match*/
  TIM15->BDTR |= TIM_BDTR_MOE;

  TIM15->CCMR1 |= TIM_CCMR1_CC1S_0 | TIM_CCMR1_IC1F_0 | TIM_CCMR1_IC1F_1;
  TIM15->CCER |= TIM_CCER_CC1E | TIM_CCER_CC1P;

  WRITE_REG(TIM15->SR, ~(TIM_SR_CC1IF));    // Clear capture interrupt flag
  TIM15->DIER |= TIM_DIER_CC1IE;            // Enable capture interrupt
  WRITE_REG(TIM15->SR, ~(TIM_SR_CC2IF));    // Clear compare interrupt flag
  TIM15->DIER |= TIM_DIER_CC2IE;            // Enable compare interrupt

  NVIC_SetPriority(TIM15_IRQn, 2);
  NVIC_EnableIRQ(TIM15_IRQn);

  /*---------------------I2C EEPROM Init(I2C2)---------------------------------*/
  /**I2C2 GPIO Configuration
 PB10   ------> I2C2_SCL
 PB11   ------> I2C2_SDA
 */
  //PB10
  GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEEDR10;    // high output speed
  GPIOB->OTYPER |= GPIO_OTYPER_OT_10;          // open-drain output type
  GPIOB->PUPDR |= GPIO_PUPDR_PUPDR10_0;        // pull up
  GPIOB->MODER |= GPIO_MODER_MODER10_1;        // Select alternate function mode
  GPIOB->AFR[1] |= (0x0000001U << (2 * 4));    // Select alternate function 1
  // PB11
  GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEEDR11;    // high output speed
  GPIOB->OTYPER |= GPIO_OTYPER_OT_11;          // open-drain output type
  GPIOB->PUPDR |= GPIO_PUPDR_PUPDR11_0;        // pull up
  GPIOB->MODER |= GPIO_MODER_MODER11_1;        // Select alternate function mode
  GPIOB->AFR[1] |= (0x0000001U << (3 * 4));    // Select alternate function 1

  /* I2C2 clock enable */
  SET_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C2EN);
  /* Delay after an RCC peripheral clock enabling */
  tmpreg = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C2EN);

  CLEAR_BIT(I2C2->OAR2, I2C_OAR2_OA2EN);   // Disable  acknowledge on Own Address2 match address
  CLEAR_BIT(I2C2->CR1, I2C_CR1_GCEN);      // Disable General Call
  CLEAR_BIT(I2C2->CR1, I2C_CR1_NOSTRETCH); // Enable Clock stretching
  CLEAR_BIT(I2C2->CR1, I2C_CR1_PE);        // Disable I2C
  MODIFY_REG(I2C2->CR1, I2C_CR1_ANFOFF | I2C_CR1_DNF, 0x00000000U);  // Configure Noise Filters (Analog and Digital)
  WRITE_REG(I2C2->TIMINGR, 0x00401B5A);    // SetTiming 375kHz
  CLEAR_BIT(I2C2->OAR1, I2C_OAR1_OA1EN);   // Disable acknowledge on Own Address1 match address
  MODIFY_REG(I2C2->OAR1, I2C_OAR1_OA1 | I2C_OAR1_OA1MODE, 0x00000000U); // Set the Own Address1
  MODIFY_REG(I2C2->CR1, I2C_CR1_SMBHEN | I2C_CR1_SMBDEN, 0x00000000U);  // Set peripheral mode I2C
  MODIFY_REG(I2C2->CR2, I2C_CR2_NACK, 0x00000000U); // Prepare the generation of a ACKnowledge
  SET_BIT(I2C2->CR2, I2C_CR2_AUTOEND);     // Enable automatic STOP condition generation
  MODIFY_REG(I2C2->OAR2, I2C_OAR2_OA2 | I2C_OAR2_OA2MSK, 0 | I2C_OAR2_OA2NOMASK); // Set the 7bits Own Address2
  /*---------------------Radio Init (SPI1)------------------------------------*/
  /* SPI1 clock enable */
  SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SPI1EN);
  /* Delay after an RCC peripheral clock enabling */
  tmpreg = READ_BIT(RCC->APB2ENR, RCC_APB2ENR_SPI1EN);
  /**SPI1 GPIO Configuration
  PE13   ------> SPI1_SCK
  PE14   ------> SPI1_MISO
  PE15   ------> SPI1_MOSI
  */
  //PE13
  GPIOE->OSPEEDR |= GPIO_OSPEEDR_OSPEEDR13;    // high output speed
  GPIOE->OTYPER |= 0x00000000U;                // Output PUSHPULL
  GPIOE->PUPDR |= 0x00000000U;                 // PULL_NO
  GPIOE->MODER |= GPIO_MODER_MODER13_1;        // Select alternate function mode
  GPIOE->AFR[1] |= (0x0000001U << (5 * 4));    // Select alternate function 1
  //PE14
  GPIOE->OSPEEDR |= GPIO_OSPEEDR_OSPEEDR14;    // high output speed
  GPIOE->OTYPER |= 0x00000000U;                // Output PUSHPULL
  GPIOE->PUPDR |= 0x00000000U;                 // PULL_NO
  GPIOE->MODER |= GPIO_MODER_MODER14_1;        // Select alternate function mode
  GPIOE->AFR[1] |= (0x0000001U << (6 * 4));    // Select alternate function 1
  //PE15
  GPIOE->OSPEEDR |= GPIO_OSPEEDR_OSPEEDR15;    // high output speed
  GPIOE->OTYPER |= 0x00000000U;                // Output PUSHPULL
  GPIOE->PUPDR |= 0x00000000U;//GPIO_PUPDR_PUPDR15_0;//                  // PULL_NO
  GPIOE->MODER |= GPIO_MODER_MODER15_1;        // Select alternate function mode
  GPIOE->AFR[1] |= (0x0000001U << (7 * 4));    // Select alternate function 1

//  SPI1->CR1 = (SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE);          /*!< Half-Duplex Tx mode. Tx transfer on 1 line */
  SPI1->CR1 |= (SPI_CR1_MSTR | SPI_CR1_SSI);                /*!< Master configuration  */
  SPI1->CR1 |= 0x00000000U;                                 /*!< SPI_POLARITY_LOW */
  SPI1->CR1 |= 0x00000000U;                                 /*!< First clock transition is the first data capture edge  */
  SPI1->CR1 |= SPI_CR1_SSM;                                 /*!< NSS managed internally. NSS pin not used and free */
  SPI1->CR1 |= SPI_CR1_BR_1;                                /*!< BaudRate control equal to fPCLK/8   */
  SPI1->CR1 |= 0x00000000U;                                 /*!< Data is transmitted/received with the MSB first */   
  SPI1->CR1 |= 0x00000000U;                                 /*!< CRC calculation disabled */


  SPI1->CR2 |= (SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0); /*!< Data length for SPI transfer:  8 bits */
  SPI1->CR2 |= 0x00000000U;                                  /*!< Motorola mode. Used as default value */
  SPI1->CR2 |= SPI_CR2_FRXTH;
  
  CLEAR_BIT(SPI1->CR2, SPI_CR2_NSSP);                       /* Disable NSS pulse management */ 
  SET_BIT(SPI1->CR1, SPI_CR1_SPE);    // SPI_ENABLE

  //RF_SCN output
  RF_SCN_GPIO_PORT->MODER |= GPIO_MODER_MODER12_0;
  //RF_RxTx output
  RF_RxTx_GPIO_PORT->MODER |= GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0;
  //RF_RF0 output
  RF_RF0_GPIO_PORT->MODER |= GPIO_MODER_MODER10_0;
  //RF_RF1 output
  RF_RF1_GPIO_PORT->MODER |= GPIO_MODER_MODER11_0;

  /* SYSCFG clock enable */
  SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SYSCFGCOMPEN);
  /* Delay after an RCC peripheral clock enabling */
  tmpreg = READ_BIT(RCC->APB2ENR, RCC_APB2ENR_SYSCFGCOMPEN);
  // RF_GIO1
  SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI2_PB;          // Set EXTI Source 
  EXTI->FTSR |= EXTI_FTSR_TR2;                           // Falling edge

 
  NVIC_SetPriority(EXTI2_3_IRQn, 2);
  NVIC_EnableIRQ(EXTI2_3_IRQn);
 /*------------------Radio_Protocol_Timer_Init(TIM16)------------------*/
  /* TIM16 clock enable */
  SET_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM16EN);
  /* Delay after an RCC peripheral clock enabling */
  tmpreg = READ_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM16EN);
  CLEAR_BIT(TIM16->CR1, TIM_CR1_ARPE);  // Disable ARR Preload
  CLEAR_BIT(TIM16->SMCR, TIM_SMCR_MSM); // Disable Master Slave Mode
  WRITE_REG(TIM16->PSC, 2);             // Prescaler   
  WRITE_REG(TIM16->ARR, 61759);         // Preload
  /* TIM6 interrupt Init */
  SET_BIT(TIM16->DIER, TIM_DIER_UIE);   // Enable update interrupt (UIE)
  NVIC_SetPriority(TIM16_IRQn, 2);
  NVIC_EnableIRQ(TIM16_IRQn);
 
  (void)tmpreg;
}

/******************************************************************************/
/*                                                                            */
/******************************************************************************/

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
void getADC_osmp() {
#define N_samples 8
  uint32_t adc_in[8] = {0};
  /* Performs the AD conversion */
  for (uint8_t i = 0; i < N_samples; i++) {
    cli();
    ADC1->CR |= ADC_CR_ADSTART; /* Start the ADC conversion */
    for (uint8_t j = 0; j < 8; j++) {
      while ((ADC1->ISR & ADC_ISR_EOC) == 0) {
      }
      adc_in[j] += ADC1->DR; /* Store the ADC conversion result */
    }
    sei();
  }
  s_anaFilt[0] = (uint16_t)(adc_in[0] / N_samples) >> 1;
  s_anaFilt[1] = (uint16_t)(adc_in[2] / N_samples) >> 1;
  s_anaFilt[2] = (uint16_t)(adc_in[1] / N_samples) >> 1;
  s_anaFilt[3] = (uint16_t)(adc_in[3] / N_samples) >> 1;
  s_anaFilt[4] = (uint16_t)(adc_in[4] / N_samples) >> 1;
  s_anaFilt[5] = (uint16_t)(adc_in[5] / N_samples) >> 1;
  s_anaFilt[6] = (uint16_t)(adc_in[6] / N_samples) >> 1;
  s_anaFilt[7] = (uint16_t)(adc_in[7] / N_samples) >> 1;
}
/*------------------------------------------------------------------------------*/
void backlight_on(void) {BACK_LIGHT_GPIO_PORT->BSRR = BACK_LIGHT_SET_Pin;}
void backlight_off(void) {BACK_LIGHT_GPIO_PORT->BSRR = BACK_LIGHT_RESET_Pin;}

void rd_1(void) {LCD_RD_GPIO_PORT->BSRR = LCD_RD_SET_Pin;}
void rd_0(void) {LCD_RD_GPIO_PORT->BSRR = LCD_RD_RESET_Pin;}
void cs_1(void) {LCD_CS_GPIO_PORT->BSRR = LCD_CS_SET_Pin;}
void cs_0(void) {LCD_CS_GPIO_PORT->BSRR = LCD_CS_RESET_Pin;}
void rw_1(void) {LCD_RW_GPIO_PORT->BSRR = LCD_RW_SET_Pin;}
void rw_0(void) {LCD_RS_GPIO_PORT->BSRR = LCD_RW_RESET_Pin;}
void rs_1(void) {LCD_RS_GPIO_PORT->BSRR = LCD_RS_SET_Pin;}
void rs_0(void) {LCD_RS_GPIO_PORT->BSRR = LCD_RS_RESET_Pin;}
void rst_1(void) {LCD_RST_GPIO_PORT->BSRR = LCD_RST_SET_Pin;}
void rst_0(void) {LCD_RST_GPIO_PORT->BSRR = LCD_RST_RESET_Pin;}

void LCD_DATA(uint8_t Data) {
  LCD_DATA_GPIO_PORT->BSRR = (uint32_t)((LCD_DATA_PORT_MASK << 16) | Data);
}
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
/*---------------------------------------------------------------------------*/
void Buzzer_SetVal(void) {BUZZER_GPIO_PORT->BSRR = BUZZER_SET_Pin;}
void Buzzer_ClrVal(void) {BUZZER_GPIO_PORT->BSRR = BUZZER_RESET_Pin;}
void Buzzer_ToggleVal(void){BUZZER_GPIO_PORT->ODR ^= BUZZER_SET_Pin;}
/*---------------------------------------------------------------------------*/
/******************************************************************************/
/*                           Radio control                                    */
/******************************************************************************/
const uint8_t AFHDS2A_A7105_regs[] = {
      0xFF, 0xC2 | (1<<5), 0x00, 0x25, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00,0x19/*GIO1 SDO*/, 0x01/*GIO2 WTR*/, 0x05, 0x00, 0x50,// 00 - 0f
      0x9e, 0x4b, 0x00, 0x02, 0x16, 0x2b, 0x12, 0x4f, 0x62, 0x80, 0xFF, 0xFF, 0x2a, 0x32, 0xc3, 0x1E/*0x1f*/,  // 10 - 1f
      0x1e, 0xFF, 0x00, 0xFF, 0x00, 0x00, 0x3b, 0x00, 0x17, 0x47, 0x80, 0x03, 0x01, 0x45, 0x18, 0x00,	       // 20 - 2f
      0x01, 0x0f // 30 - 31
};

const uint8_t AFHDS_A7105_regs[] = {
       /* 00    01    02    03    04    05    06    07    08    09    0A    0B    0C    0D    0E    0F  */
	0xff, 0x42, 0x00, 0x14, 0x00, 0xff, 0xff ,0x00, 0x00, 0x00, 0x00, 0x19/*GIO1 SDO*/, 0x01/*GIO2 WTR*/, 0x05, 0x00, 0x50,	// 00 - 0f
	0x9e, 0x4b, 0x00, 0x02, 0x16, 0x2b, 0x12, 0x00, 0x62, 0x80, 0x80, 0x00, 0x0a, 0x32, 0xc3, 0x0f,	// 10 - 1f
/*0x13*/0x1c, 0xc3, 0x00, 0xff, 0x00, 0x50/*0x00*/, 0x3b, 0x00, 0x17, 0x47, 0x80, 0x03, 0x01, 0x45, 0x18, 0x00,	// 20 - 2f
	0x01, 0x0f // 30 - 31
};

void SPI_RADIO_SendBlock(uint8_t *BufferPtr, uint16_t Size) {
  __IO uint8_t *spidr = ((__IO uint8_t *)&SPI1->DR);
  while (Size) {
    while ((SPI1->SR & SPI_SR_TXE) == 0) {
    }
    *spidr = *BufferPtr++;
    Size--;
  }
  while ((SPI1->SR & SPI_SR_BSY) != 0) {
  }
}
/*---------------------------------------------------------------------------*/
void SPI_RADIO_ReceiveBlock(uint8_t *BufferPtr, uint16_t Size) {
  __IO uint8_t *spidr = ((__IO uint8_t *)&SPI1->DR);
  uint8_t dummy;
  while ((SPI1->SR & SPI_SR_FRLVL) != 0) {
    dummy = (uint8_t)(READ_REG(SPI1->DR));
  }
  (void)dummy;
  while (Size) {
    while ((SPI1->SR & SPI_SR_TXE) == 0) {
    }
    *spidr = 0x00;
    while ((SPI1->SR & SPI_SR_RXNE) == 0) {
    }
    *BufferPtr++ = (uint8_t)(READ_REG(SPI1->DR));
    Size--;
  }
  while ((SPI1->SR & SPI_SR_BSY) != 0) {
  }
}
/*---------------------------------------------------------------------------*/
void a7105_csn_on(void) {RF_SCN_GPIO_PORT->BSRR = RF_SCN_SET_PIN;}
void a7105_csn_off(void) {RF_SCN_GPIO_PORT->BSRR = RF_SCN_RESET_PIN;}
void RF0_SetVal(void) {RF_RF0_GPIO_PORT->BSRR = RF_RF0_SET_PIN;}
void RF0_ClrVal(void) {RF_RF0_GPIO_PORT->BSRR = RF_RF0_RESET_PIN;}
void RF1_SetVal(void) {RF_RF1_GPIO_PORT->BSRR = RF_RF1_SET_PIN;}
void RF1_ClrVal(void) {RF_RF1_GPIO_PORT->BSRR = RF_RF1_RESET_PIN;}
void TX_RX_PutVal(uint32_t Val) {
  uint8_t tmp = 0;
  tmp  = (Val << 1) & 0x02;
  tmp |= (Val >> 1) & 0x01;
  RF_RxTx_GPIO_PORT->BSRR = (uint32_t)((RF_RxTx_PIN_MASK << 16) | (tmp << 8));
}

void EnableGIO(void) {
  EXTI->PR |= RF_GIO2_PIN;
  SET_BIT(EXTI->IMR, RF_GIO2_PIN);
}
void DisableGIO(void) {
  CLEAR_BIT(EXTI->IMR, RF_GIO2_PIN);
}
/******************************************************************************/
/*                         EEPROM I2C                                         */
/******************************************************************************/
#define DEFAULT_SYSTEM_CLOCK 48000000u
uint8_t i2c_master(uint8_t mode, uint16_t slave_address) {
  uint32_t busy_counter = DEFAULT_SYSTEM_CLOCK / 1000;

  // Enable I2C
  SET_BIT(I2C2->CR1, I2C_CR1_PE);
  /* Reset index for TX and RX buffers */
  i2c_buffer.tx_index = 0;
  i2c_buffer.rx_index = 0;
  if (mode == I2C_TX) {
    /* Make sure bus is idle */
    while (I2C2->ISR & I2C_ISR_BUSY) {
      if (!(busy_counter--)) {
        CLEAR_BIT(I2C2->CR1, I2C_CR1_PE);
        return I2C_BUSY;
      }
    }
    I2C2->CR2 = I2C_CR2_AUTOEND | (i2c_buffer.length << 0x10) | ((slave_address << 1) | I2C_TX);
    I2C2->CR2 |= I2C_CR2_START; /* Go */
    while ((I2C2->ISR & I2C_ISR_TXIS) != (I2C_ISR_TXIS)) {
      if (!(busy_counter--)) {
        CLEAR_BIT(I2C2->CR1, I2C_CR1_PE);
        return I2C_BUSY;
      }
    }
    if ((I2C2->ISR & I2C_ISR_NACKF) == (I2C_ISR_NACKF)) {
      CLEAR_BIT(I2C2->CR1, I2C_CR1_PE);
      return I2C_NOACK;
    }
    /* Send the contents of the TX buffer */
    while (i2c_buffer.length > 0) {
      I2C2->TXDR = *i2c_buffer.buf++;
      i2c_buffer.length--;
      /* Wait for transmit data register empty */
      while ((I2C2->ISR & I2C_ISR_TXE) != (I2C_ISR_TXE)) {
        if (!(busy_counter--)) {
          CLEAR_BIT(I2C2->CR1, I2C_CR1_PE);
          return I2C_BUSY;
        }
      }
    }
    /* Wait for transfer to complete */
    while ((I2C2->ISR & I2C_ISR_STOPF) != (I2C_ISR_STOPF)) {
      if (!(busy_counter--)) {
        CLEAR_BIT(I2C2->CR1, I2C_CR1_PE);
        return I2C_BUSY;
      }
    }
    // Disable I2C
    CLEAR_BIT(I2C2->CR1, I2C_CR1_PE);
    return I2C_OK;
  } else if (mode == I2C_RX) {
    //  /* Make sure bus is idle */
    while (I2C2->ISR & I2C_ISR_BUSY) {
      if (!(busy_counter--)) {
        CLEAR_BIT(I2C2->CR1, I2C_CR1_PE);
        return I2C_BUSY;
      }
    }
    I2C2->CR2 = I2C_CR2_AUTOEND | I2C_CR2_RD_WRN | (i2c_buffer.length << 0x10) | ((slave_address << 1) | I2C_RX);
    I2C2->CR2 |= I2C_CR2_START; /* Go */
    while (i2c_buffer.length > 0) {
      while ((I2C2->ISR & I2C_ISR_RXNE) != (I2C_ISR_RXNE)) {
      }
      *i2c_buffer.buf++ = I2C2->RXDR;
      i2c_buffer.length--;
    }
    while ((I2C2->ISR & I2C_ISR_STOPF) != (I2C_ISR_STOPF)) {
    }
  }
  // Disable I2C
  CLEAR_BIT(I2C2->CR1, I2C_CR1_PE);
  return I2C_OK;
}
/*-----------------------PPM Timer---------------------------------------------*/
uint16_t GetPPMTimCapture(void) {
  return TIM15->CCR1;
}
uint32_t GetPPMOutState(void) {
  return PPM_OUT_GPIO_PORT->IDR & PPM_OUT_PIN_MASK;
}
void SetPPMTimCompare(uint16_t val) {
  TIM15->CCR2 = val;
}
uint16_t GetPPMTimCompare(void) {
  return TIM15->CCR2;
}
uint32_t GetPPMTimCompareInterruptFlag(void) {
  return TIM15->SR & TIM_SR_CC2IF;
}
void ClearPPMTimCompareInterruptFlag(void) {
  WRITE_REG(TIM15->SR, ~(TIM_SR_CC2IF));
}
void EnablePPMTim(void) {
  SET_BIT(TIM15->CR1, TIM_CR1_CEN);    
}
void DisablePPMTim(void) {
  CLEAR_BIT(TIM15->CR1, TIM_CR1_CEN);    
}
void EnablePPMOut(void){
  SET_BIT(TIM15->CCER, TIM_CCER_CC2E);    
}
void DisablePPMOut(void){
  CLEAR_BIT(TIM15->CCER, TIM_CCER_CC2E);    
}
/*----------------------PRT Timer----------------------------------------------*/
uint16_t GetPRTTimVal(void) {
  return 0;
}
void EnablePRTTim(void) {
  SET_BIT(TIM16->CR1, TIM_CR1_CEN);    
}
void DisablePRTTim(void) {
  CLEAR_BIT(TIM16->CR1, TIM_CR1_CEN);  
  TIM16->CNT = 0;
}
void SetPRTTimPeriod(uint8_t prot){
  TIM16->CNT = 0;
  switch (prot) {
  case PROTO_AFHDS2A:
  //WRITE_REG(TIM16->PSC, 2);         
  WRITE_REG(TIM16->ARR, 61759);         
    break;
  case PROTO_AFHDS:
  //WRITE_REG(TIM16->PSC, 1);         
  WRITE_REG(TIM16->ARR, 23999/*35999*/);         
    break;
  default:
    break;
  }
};
/*----------------------------------------------------------------------------*/
uint32_t GetChipID(void) {
  return (uint32_t)(READ_REG(*((uint32_t *)UID_BASE))) ^ 
  (uint32_t)(READ_REG(*((uint32_t *)(UID_BASE + 4U)))) ^
  (uint32_t)(READ_REG(*((uint32_t *)(UID_BASE + 8U))));
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

/*--------------handler for Timer16kHz ---------------------------------------*/
void TIM6_DAC_IRQHandler(void) {
  WRITE_REG(TIM6->SR, ~(TIM_SR_UIF));    // Clear the update interrupt flag (UIF)
    g_tmr16KHz++;
    if (tmrEEPROM != 0xFFFF)
      tmrEEPROM++;
}
/*--------------handler for Timer10ms-----------------------------------------*/
void TIM7_IRQHandler(void) {
  WRITE_REG(TIM7->SR, ~(TIM_SR_UIF));    // Clear the update interrupt flag (UIF)
  ISR_TIMER0_COMP_vect();
}
/*--------------handler for AudioTimer 128uS----------------------------------*/
void TIM14_IRQHandler(void) {
  WRITE_REG(TIM14->SR, ~(TIM_SR_UIF));    // Clear the update interrupt flag (UIF)
  ISR_TIMER2_OVF_vect();
}
/*--------------handler for PPM Timer ----------------------------------------*/
void TIM15_IRQHandler(void) {
  if (TIM15->SR & TIM_SR_CC2IF) {    // Compare PPM-OUT
    WRITE_REG(TIM15->SR, ~(TIM_SR_CC2IF));
    ISR_TIMER1_COMPA_vect();
  }
  if (TIM15->SR & TIM_SR_CC1IF) {    // Capture PPM-IN
    WRITE_REG(TIM15->SR, ~(TIM_SR_CC1IF));
    ISR_TIMER3_CAPT_vect();
  }
}
/*-------------handler for RADIO GIO2 (FALLING AGE)---------------------------*/
void EXTI2_3_IRQHandler(void) {
  if (EXTI->PR & RF_GIO2_PIN) {
    WRITE_REG(EXTI->PR, RF_GIO2_PIN);
    DisableGIO();
    SETBIT(RadioState, CALLER, GPIO_CALL);
    ActionAFHDS2A();
  }
}
/*------------handler for Radio_Protocol_Timer 3860 uS------------------------*/
void TIM16_IRQHandler(void) {
  WRITE_REG(TIM16->SR, ~(TIM_SR_UIF));    // Clear the update interrupt flag (UIF)
      switch (g_model.protocol) {
      case PROTO_AFHDS2A:
        SETBIT(RadioState, CALLER, TIM_CALL);
        ActionAFHDS2A();
        break;
      case PROTO_AFHDS:
        ActionAFHDS();
        break;
      default:
        break;
      }
}