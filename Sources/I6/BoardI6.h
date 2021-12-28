


#ifndef __BOARDI6_H
#define __BOARDI6_H

#ifdef __cplusplus
extern "C" { 
#endif

#include <MKL16Z4.h>

#define ADC_AIL_VrB 0x07
#define ADC_ELE     0x05
#define ADC_THR     0x06
#define ADC_RUD     0x0B
#define ADC_VrA     0x03
#define ADC_BATT    0x04
#define ADC_SwC     0x0E
#define ADC_ONE_CONVERSION         0U        /**< One conversion mode */
#define ADC_CONTINUOUS_CONVERSIONS 0x8U      /**< Continuous conversion mode */

#define BTN_R_PORT_MASK 0x0EU                /*!< Mask of the allocated pin from the port */
#define BTN_R_PIN_ALLOC_0_INDEX 1U           /*!< The index of the first allocated pin from the port */
#define BTN_R_ClrBit(Bit) FPTB->PCOR = (1 << (Bit + BTN_R_PIN_ALLOC_0_INDEX))
#define BTN_R_PutVal(Val) FPTB->PSOR = (Val << BTN_R_PIN_ALLOC_0_INDEX)

#define BTN_L_PORT_MASK 0x000F0000U    /*!< Mask of the allocated pin from the port */
#define BTN_L_PIN_ALLOC_0_MASK 0x00010000 /*!< Mask of the first allocated pin from the port */
#define BTN_L_PIN_ALLOC_0_INDEX 16U    /*!< The index of the first allocated pin from the port */
#define BTN_L_GetVal()    ((FPTB->PDIR & BTN_L_PORT_MASK) >> BTN_L_PIN_ALLOC_0_INDEX) 

#define BIND_PORT_MASK 0x20U           /*!< Mask of the allocated pin from the port */
#define BIND_GetVal() ((FPTA->PDIR & BIND_PORT_MASK)? 1 : 0 )

#define LED_PORT_MASK 0x01U            /*!< Mask of the allocated pin from the port */
#define LCD_RD_PORT_MASK 0x40000000U   /*!< Mask of the allocated pin from the port */
#define LCD_CS_PORT_MASK 0x10U         /*!< Mask of the allocated pin from the port */
#define LCD_RW_PORT_MASK 0x80000000U   /*!< Mask of the allocated pin from the port */
#define LCD_RS_PORT_MASK 0x2000U       /*!< Mask of the allocated pin from the port */
#define LCD_RST_PORT_MASK 0x80U        /*!< Mask of the allocated pin from the port */

#define LCD_DATA_LO_PORT_MASK 0x003C0000U /*!< Mask of the allocated pin from the port */
#define LCD_DATA_LO_PIN_ALLOC_0_INDEX 18U /*!< The index of the first allocated pin from the port */

#define LCD_DATA_HI_PORT_MASK 0x0F00U  /*!< Mask of the allocated pin from the port */
#define LCD_DATA_HI_PIN_ALLOC_0_INDEX 8U /*!< The index of the first allocated pin from the port */

#define SWA_PORT_MASK 0x01U            /*!< Mask of the allocated pin from the port */
#define SWA_GetVal() ((FPTD->PDIR & SWA_PORT_MASK)? 1 : 0 )

#define SWB_PORT_MASK 0x02U            /*!< Mask of the allocated pin from the port */
#define SWB_GetVal() ((FPTC->PDIR & SWB_PORT_MASK)? 1 : 0 )

#define SWD_PORT_MASK 0x10U            /*!< Mask of the allocated pin from the port */
#define SWD_GetVal() ((FPTD->PDIR & SWD_PORT_MASK)? 1 : 0 )

#define Buzzer_PORT_MASK 0x1000U       /*!< Mask of the allocated pin from the port */

#define RF0_PORT_MASK 0x80U            /*!< Mask of the allocated pin from the port */
#define RF1_PORT_MASK 0x08U            /*!< Mask of the allocated pin from the port */

#define TX_RX_PORT_MASK 0x03U          /*!< Mask of the allocated pin from the port */
#define TX_RX_PIN_ALLOC_0_MASK 0x01    /*!< Mask of the first allocated pin from the port */
#define TX_RX_PIN_ALLOC_0_INDEX 0U     /*!< The index of the first allocated pin from the port */

#define SCN_PORT_MASK 0x10U            /*!< Mask of the allocated pin from the port */
#define GIO1_PIN_MASK 0x08U            /*!< Mask of the used pin from the port */
#define PORT_INTERRUPT_DMA_DISABLED      0U  /**< Interrupt and DMA disabled */
#define PORT_INTERRUPT_ON_ZERO           0x80000U /**< Interrupt enabled on low level */
#define PORT_INTERRUPT_ON_RISING         0x90000U /**< Interrupt enabled on rising edge */
#define PORT_INTERRUPT_ON_FALLING        0xA0000U /**< Interrupt enabled on falling edge */
#define PORT_INTERRUPT_ON_RISING_FALLING 0xB0000U /**< Interrupt enabled on rising and falling edges */
#define PORT_INTERRUPT_ON_ONE            0xC0000U /**< Interrupt enabled on high level */

extern uint16_t tim;
void PIT_IRQHandler(void);
void SPI0_IRQHandler(void);
void PORTC_PORTD_IRQHandler(void);
void TPM1_IRQHandler(void);
void TPM0_IRQHandler(void);
void TPM2_IRQHandler(void);
void UART2_IRQHandler(void);
#ifdef __cplusplus
}  /* extern "C" */
#endif 

#endif