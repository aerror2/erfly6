# 1 "/Volumes/p1t2/receiver/repo/ERFly6/Kinetis_KL/Source/MKL16Z4_Vectors.s"
# 1 "<built-in>"
# 1 "<command-line>"
# 1 "/Volumes/p1t2/receiver/repo/ERFly6/Kinetis_KL/Source/MKL16Z4_Vectors.s"
# 22 "/Volumes/p1t2/receiver/repo/ERFly6/Kinetis_KL/Source/MKL16Z4_Vectors.s"
  .syntax unified
  .code 16

  .section .init, "ax"
  .align 0





  .thumb_func
  .weak NMI_Handler
NMI_Handler:
  b .

  .thumb_func
  .weak HardFault_Handler
HardFault_Handler:
  b .

  .thumb_func
  .weak SVC_Handler
SVC_Handler:
  b .

  .thumb_func
  .weak PendSV_Handler
PendSV_Handler:
  b .

  .thumb_func
  .weak SysTick_Handler
SysTick_Handler:
  b .

  .thumb_func
Dummy_Handler:
  b .
# 149 "/Volumes/p1t2/receiver/repo/ERFly6/Kinetis_KL/Source/MKL16Z4_Vectors.s"
  .thumb_func
  .weak DMA0_IRQHandler
DMA0_IRQHandler:
  b .

  .thumb_func
  .weak DMA1_IRQHandler
DMA1_IRQHandler:
  b .

  .thumb_func
  .weak DMA2_IRQHandler
DMA2_IRQHandler:
  b .

  .thumb_func
  .weak DMA3_IRQHandler
DMA3_IRQHandler:
  b .

  .thumb_func
  .weak FTFA_IRQHandler
FTFA_IRQHandler:
  b .

  .thumb_func
  .weak LVD_LVW_IRQHandler
LVD_LVW_IRQHandler:
  b .

  .thumb_func
  .weak LLW_IRQHandler
LLW_IRQHandler:
  b .

  .thumb_func
  .weak I2C0_IRQHandler
I2C0_IRQHandler:
  b .

  .thumb_func
  .weak I2C1_IRQHandler
I2C1_IRQHandler:
  b .

  .thumb_func
  .weak SPI0_IRQHandler
SPI0_IRQHandler:
  b .

  .thumb_func
  .weak SPI1_IRQHandler
SPI1_IRQHandler:
  b .

  .thumb_func
  .weak UART0_IRQHandler
UART0_IRQHandler:
  b .

  .thumb_func
  .weak UART1_IRQHandler
UART1_IRQHandler:
  b .

  .thumb_func
  .weak UART2_IRQHandler
UART2_IRQHandler:
  b .

  .thumb_func
  .weak ADC0_IRQHandler
ADC0_IRQHandler:
  b .

  .thumb_func
  .weak CMP0_IRQHandler
CMP0_IRQHandler:
  b .

  .thumb_func
  .weak TPM0_IRQHandler
TPM0_IRQHandler:
  b .

  .thumb_func
  .weak TPM1_IRQHandler
TPM1_IRQHandler:
  b .

  .thumb_func
  .weak TPM2_IRQHandler
TPM2_IRQHandler:
  b .

  .thumb_func
  .weak RTC_IRQHandler
RTC_IRQHandler:
  b .

  .thumb_func
  .weak RTC_Seconds_IRQHandler
RTC_Seconds_IRQHandler:
  b .

  .thumb_func
  .weak PIT_IRQHandler
PIT_IRQHandler:
  b .

  .thumb_func
  .weak I2S0_IRQHandler
I2S0_IRQHandler:
  b .

  .thumb_func
  .weak DAC0_IRQHandler
DAC0_IRQHandler:
  b .

  .thumb_func
  .weak TSI0_IRQHandler
TSI0_IRQHandler:
  b .

  .thumb_func
  .weak LPTimer_IRQHandler
LPTimer_IRQHandler:
  b .

  .thumb_func
  .weak PORTA_IRQHandler
PORTA_IRQHandler:
  b .

  .thumb_func
  .weak PORTC_PORTD_IRQHandler
PORTC_PORTD_IRQHandler:
  b .







  .section .vectors, "ax"
  .align 0
  .global _vectors
  .extern __stack_end__
  .extern Reset_Handler

_vectors:
  .word __stack_end__
  .word Reset_Handler
  .word NMI_Handler
  .word HardFault_Handler
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word SVC_Handler
  .word 0
  .word 0
  .word PendSV_Handler
  .word SysTick_Handler
  .word DMA0_IRQHandler
  .word DMA1_IRQHandler
  .word DMA2_IRQHandler
  .word DMA3_IRQHandler
  .word Dummy_Handler
  .word FTFA_IRQHandler
  .word LVD_LVW_IRQHandler
  .word LLW_IRQHandler
  .word I2C0_IRQHandler
  .word I2C1_IRQHandler
  .word SPI0_IRQHandler
  .word SPI1_IRQHandler
  .word UART0_IRQHandler
  .word UART1_IRQHandler
  .word UART2_IRQHandler
  .word ADC0_IRQHandler
  .word CMP0_IRQHandler
  .word TPM0_IRQHandler
  .word TPM1_IRQHandler
  .word TPM2_IRQHandler
  .word RTC_IRQHandler
  .word RTC_Seconds_IRQHandler
  .word PIT_IRQHandler
  .word I2S0_IRQHandler
  .word Dummy_Handler
  .word DAC0_IRQHandler
  .word TSI0_IRQHandler
  .word Dummy_Handler
  .word LPTimer_IRQHandler
  .word Dummy_Handler
  .word PORTA_IRQHandler
  .word PORTC_PORTD_IRQHandler
_vectors_end:



  .section .cfm, "ax"




BackDoorKey:
  .byte 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
# 373 "/Volumes/p1t2/receiver/repo/ERFly6/Kinetis_KL/Source/MKL16Z4_Vectors.s"
FPROT:
  .byte 0xff, 0xff, 0xff, 0xff
FSEC:
  .byte 0xfe
FOPT:



  .byte 0xff

FEPROT:
  .byte 0xff
FDPROT:
  .byte 0xff
