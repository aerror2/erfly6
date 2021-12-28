


#ifndef __BOARDI6X_H
#define __BOARDI6X_H

#ifdef __cplusplus
extern "C" { 
#endif

#include <stm32f0xx.h>
/*---------------------LCD---------------------------------------------------*/
#define LCD_DATA_GPIO_PORT GPIOE
#define LCD_DATA_PORT_MASK 0x000000FFU

#define LCD_RS_GPIO_PORT GPIOB
#define LCD_RS_SET_Pin GPIO_BSRR_BS_3
#define LCD_RS_RESET_Pin GPIO_BSRR_BR_3

#define LCD_RD_GPIO_PORT GPIOD
#define LCD_RD_SET_Pin GPIO_BSRR_BS_7
#define LCD_RD_RESET_Pin GPIO_BSRR_BR_7

#define LCD_CS_GPIO_PORT GPIOD
#define LCD_CS_SET_Pin GPIO_BSRR_BS_2
#define LCD_CS_RESET_Pin GPIO_BSRR_BR_2

#define LCD_RST_GPIO_PORT GPIOB
#define LCD_RST_SET_Pin GPIO_BSRR_BS_4
#define LCD_RST_RESET_Pin GPIO_BSRR_BR_4

#define LCD_RW_GPIO_PORT GPIOB
#define LCD_RW_SET_Pin GPIO_BSRR_BS_5
#define LCD_RW_RESET_Pin GPIO_BSRR_BR_5

#define BACK_LIGHT_GPIO_PORT GPIOF
#define BACK_LIGHT_SET_Pin GPIO_BSRR_BS_3
#define BACK_LIGHT_RESET_Pin GPIO_BSRR_BR_3

/*---------------------ADC---------------------------------------------------*/
#define ADC_AIL_GPIO_PORT GPIOA
#define ADC_AIL_PIN  GPIO_BSRR_BS_0

#define ADC_THR_GPIO_PORT GPIOA
#define ADC_THR_PIN GPIO_BSRR_BS_2

#define ADC_ELE_GPIO_PORT GPIOA
#define ADC_ELE_PIN GPIO_BSRR_BS_1

#define ADC_RUD_GPIO_PORT GPIOA
#define ADC_RUD_PIN GPIO_BSRR_BS_3

#define ADC_VRA_GPIO_PORT GPIOA
#define ADC_VRA_PIN GPIO_BSRR_BS_6

#define ADC_VRB_GPIO_PORT GPIOA
#define ADC_VRB_PIN GPIO_BSRR_BS_7

#define ADC_SwC_GPIO_PORT GPIOB
#define ADC_SwC_PIN GPIO_BSRR_BS_0

#define ADC_BATT_GPIO_PORT GPIOC
#define ADC_BATT_PIN GPIO_BSRR_BS_0
/*---------------------BUZZER------------------------------------------------*/
#define BUZZER_GPIO_PORT GPIOA
#define BUZZER_SET_Pin GPIO_BSRR_BS_8
#define BUZZER_RESET_Pin GPIO_BSRR_BR_8
/*---------------------BTN_L-------------------------------------------------*/
#define BTN_L_GPIO_PORT GPIOD
#define BTN_L_PIN_MASK  (GPIO_IDR_12 | GPIO_IDR_13 | GPIO_IDR_14 | GPIO_IDR_15)
#define BTN_L_GetVal()  ((BTN_L_GPIO_PORT->IDR & BTN_L_PIN_MASK) >> 12) 
/*---------------------BTN_R-------------------------------------------------*/
#define BTN_R_GPIO_PORT GPIOC
#define BTN_R_PIN_MASK  (GPIO_ODR_6 | GPIO_ODR_7 | GPIO_ODR_8)
#define BTN_R_ClrBit(Bit) BTN_R_GPIO_PORT->BSRR = (uint32_t)(1 << (16 + 6 + Bit))
#define BTN_R_PutVal(Val) BTN_R_GPIO_PORT->BSRR = (uint32_t)((BTN_R_PIN_MASK << 16) | (Val << 6))
/*---------------------BIND--------------------------------------------------*/
#define BIND_GPIO_PORT GPIOF
#define BIND_PIN_MASK  GPIO_IDR_2 
#define BIND_GetVal()  (BIND_GPIO_PORT->IDR & BIND_PIN_MASK) 
/*---------------------SwA---------------------------------------------------*/
#define SwA_GPIO_PORT GPIOA
#define SwA_PIN_MASK GPIO_IDR_4
#define SWA_GetVal() ((SwA_GPIO_PORT->IDR & SwA_PIN_MASK)? 0 : 1 )
/*---------------------SwB---------------------------------------------------*/
#define SwB_GPIO_PORT GPIOA
#define SwB_PIN_MASK GPIO_IDR_5
#define SWB_GetVal() ((SwB_GPIO_PORT->IDR & SwB_PIN_MASK)? 1 : 0 )
/*---------------------SwD---------------------------------------------------*/
#define SwD_GPIO_PORT GPIOB
#define SwD_PIN_MASK GPIO_IDR_1
#define SWD_GetVal() ((SwD_GPIO_PORT->IDR & SwD_PIN_MASK)? 0 : 1 )
/*---------------------PPM_IN------------------------------------------------*/
#define PPM_IN_GPIO_PORT GPIOF
#define PPM_IN_PIN_PIN_MASK GPIO_IDR_9
/*---------------------PPM_OUT------------------------------------------------*/
#define PPM_OUT_GPIO_PORT GPIOF
#define PPM_OUT_PIN_MASK GPIO_IDR_10
/*---------------------Radio A7105--------------------------------------------*/
#define RF_SCK_GPIO_PORT GPIOE
#define RF_SCK_PIN_MASK GPIO_IDR_13

#define RF_SDIO_GPIO_PORT GPIOE
#define RF_SDIO_PIN_MASK GPIO_IDR_15

#define RF_SCN_GPIO_PORT GPIOE
#define RF_SCN_SET_PIN GPIO_BSRR_BS_12
#define RF_SCN_RESET_PIN GPIO_BSRR_BR_12

#define RF_GIO2_GPIO_PORT GPIOB
#define RF_GIO2_PIN EXTI_IMR_IM2

#define RF_RxTx_GPIO_PORT GPIOE
#define RF_RxTx_PIN_MASK 0x00000300U

#define RF_Rx_SET_PIN GPIO_BSRR_BS_8
#define RF_Rx_RESET_PIN GPIO_BSRR_BR_8

#define RF_Tx_SET_PIN GPIO_PIN_BSRR_BS_9
#define RF_Tx_RESET_PIN GPIO_PIN_BSRR_BR_9

#define RF_RF0_GPIO_PORT GPIOE
#define RF_RF0_SET_PIN GPIO_BSRR_BS_10
#define RF_RF0_RESET_PIN GPIO_BSRR_BR_10

#define RF_RF1_GPIO_PORT GPIOE
#define RF_RF1_SET_PIN GPIO_BSRR_BS_11
#define RF_RF1_RESET_PIN GPIO_BSRR_BR_11

/*--------------interrupt handlers-------------------------------------------*/ 
void TIM6_DAC_IRQHandler(void);
void TIM7_IRQHandler(void);
void TIM14_IRQHandler(void);
void TIM15_IRQHandler(void);
void EXTI2_3_IRQHandler(void);
void TIM16_IRQHandler(void);

#ifdef __cplusplus
}  /* extern "C" */
#endif 

#endif