/******************************************************************************
 * @file     startup_ARMCM0plus.c
 * @brief    CMSIS-Core(M) Device Startup File for a Cortex-M0+ Device
 * @version  V2.0.3
 * @date     31. March 2020
 ******************************************************************************/
/*
 * Copyright (c) 2009-2020 Arm Limited. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#define __PROGRAM_START Reset_Handler

#include "cmsis_compiler.h"

#include "system_RP2040.h"

/*----------------------------------------------------------------------------
  External References
 *----------------------------------------------------------------------------*/
extern uint32_t __INITIAL_SP;

extern __NO_RETURN void __PROGRAM_START(void);

/*----------------------------------------------------------------------------
  Internal References
 *----------------------------------------------------------------------------*/
__NO_RETURN void Reset_Handler(void);
void             Default_Handler(void);

/*----------------------------------------------------------------------------
  Exception / Interrupt Handler
 *----------------------------------------------------------------------------*/
/* Exceptions */
void NMI_Handler(void) __attribute__((weak, alias("Default_Handler")));
void HardFault_Handler(void) __attribute__((weak));
void SVC_Handler(void) __attribute__((weak, alias("Default_Handler")));
void PendSV_Handler(void) __attribute__((weak, alias("Default_Handler")));
void SysTick_Handler(void) __attribute__((weak, alias("Default_Handler")));

void TIMER_IRQ_0_Handler(void) __attribute__((weak, alias("Default_Handler")));
void TIMER_IRQ_1_Handler(void) __attribute__((weak, alias("Default_Handler")));
void TIMER_IRQ_2_Handler(void) __attribute__((weak, alias("Default_Handler")));
void TIMER_IRQ_3_Handler(void) __attribute__((weak, alias("Default_Handler")));
void PWM_IRQ_WRAP_Handler(void) __attribute__((weak, alias("Default_Handler")));
void USBCTRL_IRQ_Handler(void) __attribute__((weak, alias("Default_Handler")));
void XIP_IRQ_Handler(void) __attribute__((weak, alias("Default_Handler")));
void PIO0_IRQ_0_Handler(void) __attribute__((weak, alias("Default_Handler")));
void PIO0_IRQ_1_Handler(void) __attribute__((weak, alias("Default_Handler")));
void PIO1_IRQ_0_Handler(void) __attribute__((weak, alias("Default_Handler")));
void PIO1_IRQ_1_Handler(void) __attribute__((weak, alias("Default_Handler")));
void DMA_IRQ_0_Handler(void) __attribute__((weak, alias("Default_Handler")));
void DMA_IRQ_1_Handler(void) __attribute__((weak, alias("Default_Handler")));
void IO_IRQ_BANK0_Handler(void) __attribute__((weak, alias("Default_Handler")));
void IO_IRQ_QSPI_Handler(void) __attribute__((weak, alias("Default_Handler")));
void SIO_IRQ_PROC0_Handler(void) __attribute__((weak, alias("Default_Handler")));
void SIO_IRQ_PROC1_Handler(void) __attribute__((weak, alias("Default_Handler")));
void CLOCKS_IRQ_Handler(void) __attribute__((weak, alias("Default_Handler")));
void SPI0_IRQ_Handler(void) __attribute__((weak, alias("Default_Handler")));
void SPI1_IRQ_Handler(void) __attribute__((weak, alias("Default_Handler")));
void UART0_IRQ_Handler(void) __attribute__((weak, alias("Default_Handler")));
void UART1_IRQ_Handler(void) __attribute__((weak, alias("Default_Handler")));
void ADC_IRQ_FIFO_Handler(void) __attribute__((weak, alias("Default_Handler")));
void I2C0_IRQ_Handler(void) __attribute__((weak, alias("Default_Handler")));
void I2C1_IRQ_Handler(void) __attribute__((weak, alias("Default_Handler")));
void RTC_IRQ_Handler(void) __attribute__((weak, alias("Default_Handler")));

/*----------------------------------------------------------------------------
  Exception / Interrupt Vector table
 *----------------------------------------------------------------------------*/

#if defined(__GNUC__)
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wpedantic"
#endif

extern const VECTOR_TABLE_Type __VECTOR_TABLE[48];
const VECTOR_TABLE_Type        __VECTOR_TABLE[48] __VECTOR_TABLE_ATTRIBUTE = {
    (VECTOR_TABLE_Type) (&__INITIAL_SP), /*     Initial Stack Pointer */
    Reset_Handler,                       /*     Reset Handler */
    NMI_Handler,                         /* -14 NMI Handler */
    HardFault_Handler,                   /* -13 Hard Fault Handler */
    0,                                   /*     Reserved */
    0,                                   /*     Reserved */
    0,                                   /*     Reserved */
    0,                                   /*     Reserved */
    0,                                   /*     Reserved */
    0,                                   /*     Reserved */
    0,                                   /*     Reserved */
    SVC_Handler,                         /*  -5 SVCall Handler */
    0,                                   /*     Reserved */
    0,                                   /*     Reserved */
    PendSV_Handler,                      /*  -2 PendSV Handler */
    SysTick_Handler,                     /*  -1 SysTick Handler */

    /* Interrupts */
    TIMER_IRQ_0_Handler,   /*   0  TIMER_IRQ_0   */
    TIMER_IRQ_1_Handler,   /*   1  TIMER_IRQ_1   */
    TIMER_IRQ_2_Handler,   /*   2  TIMER_IRQ_2   */
    TIMER_IRQ_3_Handler,   /*   3  TIMER_IRQ_3   */
    PWM_IRQ_WRAP_Handler,  /*   4  PWM_IRQ_WRAP  */
    USBCTRL_IRQ_Handler,   /*   5  USBCTRL_IRQ   */
    XIP_IRQ_Handler,       /*   6  XIP_IRQ       */
    PIO0_IRQ_0_Handler,    /*   7  PIO0_IRQ_0    */
    PIO0_IRQ_1_Handler,    /*   8  PIO0_IRQ_1    */
    PIO1_IRQ_0_Handler,    /*   9  PIO1_IRQ_0    */
    PIO1_IRQ_1_Handler,    /*   10 PIO1_IRQ_1    */
    DMA_IRQ_0_Handler,     /*   11 DMA_IRQ_0     */
    DMA_IRQ_1_Handler,     /*   12 DMA_IRQ_1     */
    IO_IRQ_BANK0_Handler,  /*   13 IO_IRQ_BANK0  */
    IO_IRQ_QSPI_Handler,   /*   14 IO_IRQ_QSPI   */
    SIO_IRQ_PROC0_Handler, /*   15 SIO_IRQ_PROC0 */
    SIO_IRQ_PROC1_Handler, /*   16 SIO_IRQ_PROC1 */
    CLOCKS_IRQ_Handler,    /*   17 CLOCKS_IRQ    */
    SPI0_IRQ_Handler,      /*   18 SPI0_IRQ      */
    SPI1_IRQ_Handler,      /*   19 SPI1_IRQ      */
    UART0_IRQ_Handler,     /*   20 UART0_IRQ     */
    UART1_IRQ_Handler,     /*   21 UART1_IRQ     */
    ADC_IRQ_FIFO_Handler,  /*   22 ADC_IRQ_FIFO  */
    I2C0_IRQ_Handler,      /*   23 I2C0_IRQ      */
    I2C1_IRQ_Handler,      /*   24 I2C1_IRQ      */
    RTC_IRQ_Handler,       /*   25 RTC_IRQ       */
};

#if defined(__GNUC__)
    #pragma GCC diagnostic pop
#endif

/*----------------------------------------------------------------------------
  Reset Handler called on controller reset
 *----------------------------------------------------------------------------*/
__NO_RETURN void Reset_Handler(void)
{
    extern void _start(void) __NO_RETURN;

    typedef struct {
        uint32_t const *src;
        uint32_t       *dest;
        uint32_t        wlen;
    } __copy_table_t;

    typedef struct {
        uint32_t *dest;
        uint32_t  wlen;
    } __zero_table_t;

    extern const __copy_table_t __copy_table_start__;
    extern const __copy_table_t __copy_table_end__;
    extern const __zero_table_t __zero_table_start__;
    extern const __zero_table_t __zero_table_end__;

#ifdef __RAMFUNC
    extern const uint32_t __ramfunc_at_flash, __ramfunc_start__, __ramfunc_end__;
#endif

    for (__copy_table_t const *pTable = &__copy_table_start__; pTable < &__copy_table_end__; ++pTable) {
        for (uint32_t i = 0u; i < pTable->wlen; ++i) {
            pTable->dest[i] = pTable->src[i];
        }
    }

    for (__zero_table_t const *pTable = &__zero_table_start__; pTable < &__zero_table_end__; ++pTable) {
        for (uint32_t i = 0u; i < pTable->wlen; ++i) {
            pTable->dest[i] = 0u;
        }
    }

#ifdef __RAMFUNC
    uint32_t *rom = (uint32_t*) &__ramfunc_at_flash;
    for (uint32_t *p = (uint32_t*)&__ramfunc_start__; p < (uint32_t*)&__ramfunc_end__; ++p) {
        *p = *rom++;
    }
#endif

    SystemInit(); /* CMSIS System Initialization */

    _start(); /* Enter PreMain (C library entry point) */
}

#if defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
    #pragma clang diagnostic push
    #pragma clang diagnostic ignored "-Wmissing-noreturn"
#endif

/*----------------------------------------------------------------------------
  Hard Fault Handler
 *----------------------------------------------------------------------------*/
__attribute__((__weak)) void HardFault_Handler(void)
{
    while (1)
        ;
}

/*----------------------------------------------------------------------------
  Default Handler for Exceptions / Interrupts
 *----------------------------------------------------------------------------*/
__attribute__((__weak))  void Default_Handler(void)
{
    while (1)
        ;
}

#if defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
    #pragma clang diagnostic pop
#endif
