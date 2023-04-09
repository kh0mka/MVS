//*****************************************************************************
//
// startup.c - Boot code for Stellaris.
//
// Copyright (c) 2005-2007 Luminary Micro, Inc.  All rights reserved.
//
// Software License Agreement
//
// Luminary Micro, Inc. (LMI) is supplying this software for use solely and
// exclusively on LMI's microcontroller products.
//
// The software is owned by LMI and/or its suppliers, and is protected under
// applicable copyright laws.  All rights are reserved.  Any use in violation
// of the foregoing restrictions may subject the user to criminal sanctions
// under applicable laws, as well as to civil liability for the breach of the
// terms and conditions of this license.
//
// THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
// OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
// LMI SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
// CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
//
// This is part of revision 1392 of the Stellaris Peripheral Driver Library.
//
//*****************************************************************************
#include <stdint.h>
#include <stddef.h>
#include "stm32f4xx_conf.h"

//*****************************************************************************
//
// Forward declaration of the default fault handlers.
//
//*****************************************************************************
void ResetISR(void);
static void NmiSR(void);
static void IntDefaultHandler(void);
static void TimingDelay_Decrement ( void );
void ACCEL_TIM_IRQHandler(void);

//*****************************************************************************
//
// The entry point for the application.
//
//*****************************************************************************
extern int main(void);

//*****************************************************************************
//
// Reserve space for the system stack.
//
//*****************************************************************************
#ifndef STACK_SIZE
#define STACK_SIZE                              2048
#endif
unsigned long pulStack[STACK_SIZE];

//*****************************************************************************
//
// The minimal vector table for a Cortex-M3.  Note that the proper constructs
// must be placed on this to ensure that it ends up at physical address
// 0x0000.0000.
//
//*****************************************************************************
__attribute__ ((section(".isr_vector")))
void (* const g_pfnVectors[])(void) =
{
	(void (*)(void)) (pulStack + STACK_SIZE),
                                            // The initial stack pointer
    ResetISR,								// The reset handler
    NmiSR,									// The NMI handler
    ResetISR,								// The hard fault handler
    IntDefaultHandler,						// The MPU fault handler
    IntDefaultHandler,						// The bus fault handler
    IntDefaultHandler,						// The usage fault handler
    0,										// Reserved
    0,										// Reserved
    0,										// Reserved
    0,										// Reserved
    IntDefaultHandler,						// SVCall handler
    IntDefaultHandler,						// Debug monitor handler
    IntDefaultHandler,						// Reserved
    IntDefaultHandler,						// The PendSV handler
	TimingDelay_Decrement,						// The SysTick handler
    IntDefaultHandler,						// Window Watchdog interrupt
    IntDefaultHandler,						// PVD through EXTI line detection interrupt
    IntDefaultHandler,						// Tamper and TimeStamp interrupts through the EXTI line
    IntDefaultHandler,						// RTC Wakeup interrupt through the EXTI line
    IntDefaultHandler,						// Flash global interrupt
    IntDefaultHandler,						// RCC global interrupt
    IntDefaultHandler,						// EXTI Line0 interrupt
    IntDefaultHandler,						// EXTI Line1 interrupt
    IntDefaultHandler,						// EXTI Line2 interrupt
    IntDefaultHandler,						// EXTI Line3 interrupt
    IntDefaultHandler,						// EXTI Line4 interrupt
    IntDefaultHandler,						// DMA1 Stream0 global interrupt
    IntDefaultHandler,						// DMA1 Stream1 global interrupt
    IntDefaultHandler,						// DMA1 Stream2 global interrupt
	IntDefaultHandler,						// DMA1 Stream3 global interrupt
    IntDefaultHandler,						// DMA1 Stream4 global interrupt
    IntDefaultHandler,						// DMA1 Stream5 global interrupt
    IntDefaultHandler,						// DMA1 Stream6 global interrupt
    IntDefaultHandler,						// ADC1, ADC2 and ADC3 global interrupts
    IntDefaultHandler,						// CAN1 TX interrupts
    IntDefaultHandler,						// CAN1 RX0 interrupts
    IntDefaultHandler,						// CAN1 RX1 interrupt
    IntDefaultHandler,						// CAN1 SCE interrupt
    IntDefaultHandler,						// EXTI Line[9:5] interrupts
    IntDefaultHandler,						// TIM1 Break interrupt and TIM9 global interrupt
	ACCEL_TIM_IRQHandler,						// TIM1 Update interrupt and TIM10 global interrupt
    IntDefaultHandler,						// TIM1 Trigger and Commutation interrupts and TIM11 global interrupt
    IntDefaultHandler,						// TIM1 Capture Compare interrupt
    IntDefaultHandler,						// TIM2 global interrupt
    IntDefaultHandler,						// TIM3 global interrupt
    IntDefaultHandler,						// TIM4 global interrupt
    IntDefaultHandler,						// I2C1 event interrupt
    IntDefaultHandler,						// I2C1 error interrupt
    IntDefaultHandler,						// I2C2 event interrupt
    IntDefaultHandler,						// I2C2 error interrupt
    IntDefaultHandler,						// SPI1 global interrupt
    IntDefaultHandler,						// SPI2 global interrupt
    IntDefaultHandler,						// USART1 global interrupt
    IntDefaultHandler,						// USART2 global interrupt
    IntDefaultHandler,						// USART3 global interrupt
    IntDefaultHandler,						// EXTI Line[15:10] interrupts
    IntDefaultHandler,						// RTC Alarms (A and B) through EXTI line interrupt
    IntDefaultHandler,						// USB On-The-Go FS Wakeup through EXTI line interrupt
    IntDefaultHandler,						// TIM8 Break interrupt and TIM12 global interrupt
    IntDefaultHandler,						// TIM8 Update interrupt and TIM13 global interrupt
    IntDefaultHandler,						// TIM8 Trigger and Commutation interrupts and TIM14 global interrupt
    IntDefaultHandler,						// TIM8 Capture Compare interrupt
	IntDefaultHandler,						// DMA1 Stream7 global interrupt
    IntDefaultHandler,						// FSMC global interrupt
    IntDefaultHandler,						// SDIO global interrupt
    IntDefaultHandler,						// TIM5 global interrupt
    IntDefaultHandler,						// SPI3 global interrupt
    IntDefaultHandler,						// UART4 global interrupt
    IntDefaultHandler,						// UART5 global interrupt
    IntDefaultHandler,						// TIM6 global interrupt, DAC1 and DAC2 underrun error interrupts
    IntDefaultHandler,						// TIM7 global interrupt
    IntDefaultHandler,						// DMA2 Stream0 global interrupt
	IntDefaultHandler,						// DMA2 Stream1 global interrupt
    IntDefaultHandler,						// DMA2 Stream2 global interrupt
    IntDefaultHandler,						// DMA2 Stream3 global interrupt
    IntDefaultHandler,						// DMA2 Stream4 global interrupt
    IntDefaultHandler,						// Ethernet global interrupt
    IntDefaultHandler,						// Ethernet Wakeup through EXTI line interrupt
    IntDefaultHandler,						// CAN2 TX interrupts
    IntDefaultHandler,						// CAN2 RX0 interrupts
    IntDefaultHandler,						// CAN2 RX1 interrupt
    IntDefaultHandler,						// CAN2 SCE interrupt
    IntDefaultHandler,						// USB On The Go FS global interrupt
    IntDefaultHandler,						// DMA2 Stream5 global interrupt
    IntDefaultHandler,						// DMA2 Stream6 global interrupt
    IntDefaultHandler,						// DMA2 Stream7 global interrupt
    IntDefaultHandler,						// USART6 global interrupt
    IntDefaultHandler,						// I2C3 event interrupt
    IntDefaultHandler,						// I2C3 error interrupt
    IntDefaultHandler,						// USB On The Go HS End Point 1 Out global interrupt
    IntDefaultHandler,						// USB On The Go HS End Point 1 In global interrupt
    IntDefaultHandler,						// USB On The Go HS Wakeup through EXTI interrupt
    IntDefaultHandler,						// USB On The Go HS global interrupt
    IntDefaultHandler,						// DCMI global interrupt
    IntDefaultHandler,						// CRYP crypto global interrupt
    IntDefaultHandler,						// Hash and Rng global interrupt
    IntDefaultHandler						// FPU global interrupt
};

//*****************************************************************************
//
// The following are constructs created by the linker, indicating where the
// the "data" and "bss" segments reside in memory.  The initializers for the
// for the "data" segment resides immediately following the "text" segment.
//
//*****************************************************************************
extern unsigned long _etext;
extern unsigned long _sdata;
extern unsigned long _edata;
extern unsigned long _sbss;
extern unsigned long _ebss;

//*****************************************************************************
//
// This is the code that gets called when the processor first starts execution
// following a reset event.  Only the absolutely necessary set is performed,
// after which the application supplied main() routine is called.  Any fancy
// actions (such as making decisions based on the reset cause register, and
// resetting the bits in that register) are left solely in the hands of the
// application.
//
//*****************************************************************************
void ResetISR(void) {
	register unsigned long *pulSrc, *pulDest;
	//
	// Copy the data segment initializers from flash to SRAM.
	//
	pulSrc = &_etext;
	for (pulDest = &_sdata; pulDest < &_edata;) {
		*pulDest++ = *pulSrc++;
	}
	//
	// Zero fill the bss segment.
	//
	for (pulDest = &_sbss; pulDest < &_ebss;) {
		*pulDest++ = 0;
	}
	SystemInit();
	//
	// Call the application's entry point.
	//
	main();
}

//*****************************************************************************
//
// This is the code that gets called when the processor receives a NMI.  This
// simply enters an infinite loop, preserving the system state for examination
// by a debugger.
//
//*****************************************************************************
static void NmiSR(void) {
	//
	// Enter an infinite loop.
	//
	while (1) {
	}
}

//*****************************************************************************
//
// This is the code that gets called when the processor receives an unexpected
// interrupt.  This simply enters an infinite loop, preserving the system state
// for examination by a debugger.
//
//*****************************************************************************
static void IntDefaultHandler(void) {
	//
	// Enter an infinite loop.
	//
	while (1) {
	}
}

__IO uint32_t TimingDelay;
__IO uint32_t tick = 0;


/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
static void TimingDelay_Decrement ( void ) {
	if (TimingDelay != 0x00) {
		TimingDelay--;
	}
	tick++;	}


/**
 * @brief  Inserts a delay time.
 * @param  nTime: specifies the delay time length, in milliseconds
 * @retval None
 */
void Delay ( uint32_t nTime ) {
	TimingDelay = nTime;
	while (TimingDelay != 0);
}
void ResetTickCount() {
	tick = 0;
}
uint32_t GetTickCount() {
	return tick;}
