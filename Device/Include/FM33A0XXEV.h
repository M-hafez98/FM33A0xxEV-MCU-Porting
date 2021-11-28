/**************************************************************************************************** 
 * @file     FM33A0XXEV.h
 *
 * @brief    CMSIS CORTEX-M0 Peripheral Access Layer Header File for
 *           FM33A0XXEV from Keil.
 *
 * @version  V0.0.1
 * @date     7 May 2020
 *
 * @note     Generated with SVDConv V2.87e 
 *           from CMSIS SVD File 'FM33A0XXEV.SVD' Version 1.0,
 *
 * @par      ARM Limited (ARM) is supplying this software for use with Cortex-M
 *           processor based microcontroller, but can be equally used for other
 *           suitable processor architectures. This file can be freely distributed.
 *           Modifications to this file shall be clearly marked.
 *
 *           THIS SOFTWARE IS PROVIDED “AS IS”. NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 *           OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 *           MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 *           ARM SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
 *           CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 *******************************************************************************************************/


/** @addtogroup Keil
  * @{
  */

/** @addtogroup FM33A0XXEV
  * @{
  */

#ifndef FM33A0XXEV_H
#define FM33A0XXEV_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus;
typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunState;
typedef enum {FAIL = 0, PASS = !FAIL} ErrorStatus;

#define __RCHF_INITIAL_CLOCK        (8000000)        /* Value of the Internal RC HIGH oscillator in Hz */
#define __RCMF_CLOCK                (2000000)        /* Value of the Internal RC 4M oscillator in Hz */
#define __RCLP_CLOCK               (32000)          /* Value of the Internal RC LOW oscillator in Hz */
#define __XTLF_CLOCK                (32768)          /* Value of the EXTERNAL oscillator in Hz */


/* -------------------------  Interrupt Number Definition  ------------------------ */

typedef enum {

/* ------------------  FM33A0XXEV Processor Exceptions Numbers  ------------------*/
  Reset_IRQn                    = -15,              /*!<   1  复位向量 */
  NMI_IRQn                      = -14,              /*!<   2  WKUPx中断低功耗模式错误中断 */
  HardFault_IRQn                = -13,              /*!<   3  HardFault中断向量 */
  SVC_IRQn                      = -5,               /*!<  11  SVCall系统服务请求 */
  PendSV_IRQn                   = -2,               /*!<  14  可挂起系统服务请求 */
  SysTick_IRQn                  = -1,               /*!<  15  内部定时器中断向量 */

/* --------------------  Cortex-M0 Specific Interrupt Numbers  --------------------*/
  WWDT_IRQn                     = 0,                /*!<   0  窗口看门狗中断 */
  SVD_IRQn                      = 1,                /*!<   1  电源监测报警中断 */
  RTC_IRQn                      = 2,                /*!<   2  RTC中断 */
  FLASH_IRQn                    = 3,                /*!<   3  NVMIF中断 */
  CMU_IRQn                      = 4,                /*!<   4  XTLF停振检测中断XTHF停振检测中断系统时钟切换错误中断 */
  ADC_IRQn                      = 5,                /*!<   5  ADC或CIC中断 */
  SPI0_IRQn                     = 6,                /*!<   6  SPI中断 */
  SPI1_IRQn                     = 7,                /*!<   7  SPI中断 */
  SPI2_IRQn                     = 8,                /*!<   8  SPI中断 */
  UART0_IRQn                    = 9,                /*!<   9  UART中断 */
  UART1_IRQn                    = 10,               /*!<  10  UART中断 */
  UART2_IRQn                    = 11,               /*!<  11  UART中断 */
  UART3_IRQn                    = 12,               /*!<  12  UART中断 */
  UART4_IRQn                    = 13,               /*!<  13  UART中断 */
  UART5_IRQn                    = 14,               /*!<  14  UART中断 */
  U7816_IRQn                    = 15,               /*!<  15  U7816中断 */
  LPUART0_IRQn                  = 16,               /*!<  16  低功耗UART中断 */
  I2Cx_IRQn                     = 17,               /*!<  17  I2C0&1中断 */
  RSV_IRQn                      = 18,               /*!<  18  NULL         */
  CRYPTO_IRQn                   = 19,               /*!<  19  加密算法或TRNG中断整合AES/PAE/HASH */
  LPTIM_IRQn                    = 20,               /*!<  20  低功耗定时器中断 */
  DMA_IRQn                      = 21,               /*!<  21  DMA中断 */
  WKUPx_IRQn                    = 22,               /*!<  22  WKUP引脚中断 */
  COMP_IRQn                     = 23,               /*!<  23  模拟比较器中断 */
  BTx_IRQn                      = 24,               /*!<  24  Basic Timer x中断 */
  QSPI_IRQn                     = 25,               /*!<  25  QuadSPI中断 */
  ETx_IRQn                      = 26,               /*!<  26  Extended Timer x中断 */
  BSTIM_IRQn                    = 27,               /*!<  27  基本定时器中断 */
  SPI3_IRQn                     = 28,               /*!<  28  SPI中断 */
  SPI4_IRQn                     = 29,               /*!<  29  SPI中断 */
  GPIO_IRQn                     = 30,               /*!<  30  外部引脚中断 */
  LPUART1_IRQn                  = 31,               /*!<  31  低功耗UART中断 */

} IRQn_Type;


/** @addtogroup Configuration_of_CMSIS
  * @{
  */

/* ================================================================================ */
/* ================      Processor and Core Peripheral Section     ================ */
/* ================================================================================ */

/* ----------------Configuration of the CORTEX-M0 Processor and Core Peripherals---------------- */
#define __CM0_REV                 0x0100            /*!< CORTEX-M0 Core Revision                                               */
#define __MPU_PRESENT                  1            /*!< MPU present or not                                                    */
#define __VTOR_PRESENT                 1           /*!< VTOR present or not                                                    */
#define __NVIC_PRIO_BITS               2            /*!< Number of Bits used for Priority Levels                               */
#define __Vendor_SysTickConfig         0            /*!< Set to 1 if different SysTick Config is used                          */
/** @} */ /* End of group Configuration_of_CMSIS */

#include "core_cm0plus.h"                            /*!< Cortex-M0 processor and core peripherals*/
#include "system_FM33A0XXEV.h"                       /*!< FM33A0XXEV System*/

/* ================================================================================ */
/* ================       Device Specific Peripheral Section       ================ */
/* ================================================================================ */

/** @addtogroup Device_Peripheral_Registers
  * @{
  */

/* -------------------  Start of section using anonymous unions  ------------------ */
#if defined(__CC_ARM)
  #pragma push
  #pragma anon_unions
#elif defined(__ICCARM__)
  #pragma language=extended
#elif defined(__GNUC__)
/* anonymous unions are enabled by default */
#elif defined(__TMS470__)
/* anonymous unions are enabled by default */
#elif defined(__TASKING__)
  #pragma warning 586
#else
#warning Not supported compiler type
#endif


/* ================================================================================ */
/* ================                       PMU                      ================ */
/* ================================================================================ */

typedef struct
{
  __IO uint32_t CR;                     /*!<  Power Management Control Register,                    Address offset: 0x00 */
  __IO uint32_t WKTR;                   /*!<  Wakeup Time Register,                                 Address offset: 0x04 */
  __IO uint32_t WKFR;                   /*!<  Wakeup Source Flags Register,                         Address offset: 0x08 */
  __IO uint32_t IER;                    /*!<  PMU Interrupt Enable Register,                        Address offset: 0x0C */
  __IO uint32_t ISR;                    /*!<  PMU Interrupt and Status Register,                    Address offset: 0x10 */
}PMU_Type;


/* ================================================================================ */
/* ================                       DBG                      ================ */
/* ================================================================================ */

typedef struct
{
  __IO uint32_t RSV1;                   /*!<  RESERVED REGISTER,                                    Address offset: 0x00 */
  __IO uint32_t CR;                     /*!<  Debug Configuration Register,                         Address offset: 0x04 */
  __IO uint32_t HDFR;                   /*!<  HardFault Flag Register,                              Address offset: 0x08 */
}DBG_Type;


/* ================================================================================ */
/* ================                       FLS                      ================ */
/* ================================================================================ */

typedef struct
{
  __IO uint32_t RDCR;                   /*!<  Flash Read Control Register,                          Address offset: 0x00 */
  __IO uint32_t PFCR;                   /*!<  Flash Prefetch Control Register,                      Address offset: 0x04 */
  __I  uint32_t OPTBR;                  /*!<  Flash Option Bytes Register,                          Address offset: 0x08 */
  __IO uint32_t RSV1[2];                /*!<  RESERVED REGISTER,                                    Address offset: 0x0C */
  __IO uint32_t EPCR;                   /*!<  Flash Erase/Program Control Register,                 Address offset: 0x14 */
  __O  uint32_t KEY;                    /*!<  Flash Key Register,                                   Address offset: 0x18 */
  __IO uint32_t IER;                    /*!<  Flash Interrupt Enable Register,                      Address offset: 0x1C */
  __IO uint32_t ISR;                    /*!<  Flash Interrupt Status Register,                      Address offset: 0x20 */
  __IO uint32_t RSV2[9];                /*!<  RESERVED REGISTER,                                    Address offset: 0x24 */
  __IO uint32_t ACLOCK1;                /*!<  Flash Application Code Lock Register1,                Address offset: 0x48 */
  __IO uint32_t ACLOCK2;                /*!<  Flash Application Code Lock Register2,                Address offset: 0x4C */
  __IO uint32_t ACLOCK3;                /*!<  Flash Application Code Lock Register1,                Address offset: 0x50 */
  __IO uint32_t ACLOCK4;                /*!<  Flash Application Code Lock Register2,                Address offset: 0x54 */
}FLS_Type;


/* ================================================================================ */
/* ================                       RMU                      ================ */
/* ================================================================================ */

typedef struct
{
  __IO uint32_t PDRCR;                  /*!<  PDR Control Register,                                 Address offset: 0x00 */
  __IO uint32_t BORCR;                  /*!<  BOR Control Register,                                 Address offset: 0x04 */
  __IO uint32_t RSTCFGR;                /*!<  Reset Config Register,                                Address offset: 0x08 */
  __O  uint32_t SOFTRST;                /*!<  Soft Reset Register,                                  Address offset: 0x0C */
  __IO uint32_t RSR;                    /*!<  Reset Status Register,                                Address offset: 0x10 */
  __O  uint32_t PRSTEN;                 /*!<  Peripheral Reset Enable Register,                     Address offset: 0x14 */
  __IO uint32_t AHBRST;                 /*!<  AHB Peripheral Reset Register,                        Address offset: 0x18 */
  __IO uint32_t APBRST1;                /*!<  APB Peripheral Reset Register1,                       Address offset: 0x1C */
  __IO uint32_t APBRST2;                /*!<  APB Peripheral Reset Register2,                       Address offset: 0x20 */
}RMU_Type;


/* ================================================================================ */
/* ================                      IWDT                      ================ */
/* ================================================================================ */

typedef struct
{
  __O  uint32_t SERV;                   /*!<  IWDT Serve Register,                                  Address offset: 0x00 */
  __IO uint32_t CFGR;                   /*!<  IWDT Config Register,                                 Address offset: 0x04 */
  __I uint32_t CNTR;                   /*!<   Counter Register,                                    Address offset: 0x08 */
}IWDT_Type;


/* ================================================================================ */
/* ================                      WWDT                      ================ */
/* ================================================================================ */

typedef struct
{
  __O  uint32_t CR;                     /*!<  WWDT Control Register,                                Address offset: 0x00 */
  __IO uint32_t CFGR;                   /*!<  WWDT Config Register,                                 Address offset: 0x04 */
  __I  uint32_t CNTR;                   /*!<  WWDT Counter Register,                                Address offset: 0x08 */
  __IO uint32_t IER;                    /*!<  WWDT Interrupt Enable Register,                       Address offset: 0x0C */
  __IO uint32_t ISR;                    /*!<  WWDT Interrupt Status Register,                       Address offset: 0x10 */
  __I  uint32_t PSCR;                   /*!<  WWDT Prescaler Register,                              Address offset: 0x14 */
}WWDT_Type;


/* ================================================================================ */
/* ================                      CMU                          ================ */
/* ================================================================================ */

typedef struct
{
  __IO uint32_t SYSCLKCR;               /*!<  System Clock Control Register,                        Address offset: 0x00 */
  __IO uint32_t RCHFCR;                 /*!<  RCHF Control Register,                                Address offset: 0x04 */
  __IO uint32_t RCHFTR;                 /*!<  RCHF Trim Register,                                   Address offset: 0x08 */
  __IO uint32_t PLLLCR;                 /*!<  PLL_L Control Register,                               Address offset: 0x0C */
  __IO uint32_t PLLHCR;                 /*!<  PLL_H Control Register,                               Address offset: 0x10 */
  __IO uint32_t XTHFCR;                 /*!<  XTHF Control Register,                                Address offset: 0x14 */
  __IO uint32_t IER;                    /*!<  Interrupt Enable Register,                            Address offset: 0x18 */
  __IO uint32_t ISR;                    /*!<  Interrupt Status Register,                            Address offset: 0x1C */
  __IO uint32_t PCLKCR1;                /*!<  Peripheral bus Clock Control Register1,               Address offset: 0x20 */
  __IO uint32_t PCLKCR2;                /*!<  Peripheral bus Clock Control Register2,               Address offset: 0x24 */
  __IO uint32_t PCLKCR3;                /*!<  Peripheral bus Clock Control Register3,               Address offset: 0x28 */
  __IO uint32_t PCLKCR4;                /*!<  Peripheral bus Clock Control Register4,               Address offset: 0x2C */
  __IO uint32_t OPCCR1;                 /*!<  Peripheral Operation Clock Control Register1,         Address offset: 0x30 */
  __IO uint32_t OPCCR2;                 /*!<  Peripheral Operation Clock Control Register2,         Address offset: 0x34 */
}CMU_Type;

/* ================================================================================ */
/* ================                      CDIF                      ================ */
/* ================================================================================ */

typedef struct
{
  __IO uint32_t CR;                     /*!<  CDIF Control Register,                                Address offset: 0x00 */
  __IO uint32_t PSCR;                   /*!<  CDIF Prescaler Register,                              Address offset: 0x04 */
}CDIF_Type;


/* ================================================================================ */
/* ================                      VRTC                      ================ */
/* ================================================================================ */

typedef struct
{
  __IO uint32_t PDRCR;                /*!<  RESERVED REGISTER,                                      Address offset: 0x00 */
  __IO uint32_t RSV1[4];                /*!<  RESERVED REGISTER,                                    Address offset: 0x04 */
  __IO uint32_t RCMFCR;                 /*!<  RCMF Control Register,                                Address offset: 0x14 */
  __IO uint32_t RCLPCR;                 /*!<  RCLP Control Register,                                Address offset: 0x18 */
  __IO uint32_t RCLPTR;                 /*!<  RCLP Control Register,                                Address offset: 0x1C */
  __IO uint32_t XTLFCR;                 /*!<  XTLF Control Register,                                Address offset: 0x20 */
  __IO uint32_t ADCCR;                  /*!<  ADC Clock Control Register,                           Address offset: 0x24 */
  __IO uint32_t LFDIER;                 /*!<  LFDET Interrupt Enable Register,                      Address offset: 0x28 */
  __IO uint32_t LFDISR;                 /*!<  LFDET Interrupt Status Register,                      Address offset: 0x2C */
}VRTC_Type;


/* ================================================================================ */
/* ================                       SVD                      ================ */
/* ================================================================================ */

typedef struct
{
  __IO uint32_t CFGR;                   /*!<  SVD Config Register,                                  Address offset: 0x00 */
  __IO uint32_t CR;                     /*!<  SVD Control Register,                                 Address offset: 0x04 */
  __IO uint32_t IER;                    /*!<  SVD Interrupt Enable Register,                        Address offset: 0x08 */
  __IO uint32_t ISR;                    /*!<  SVD Interrupt Status Register,                        Address offset: 0x0C */
  __IO uint32_t VSR;                    /*!<  SVD reference Voltage Select Register,                Address offset: 0x10 */
}SVD_Type;


/* ================================================================================ */
/* ================                       AES                      ================ */
/* ================================================================================ */

typedef struct
{
  __IO uint32_t CR;                     /*!<  AES Control Register,                                 Address offset: 0x00 */
  __IO uint32_t IER;                    /*!<  AES Interrupt Enable Register,                        Address offset: 0x04 */
  __IO uint32_t ISR;                    /*!<   Interrupt Status Register,                           Address offset: 0x08 */
  __IO uint32_t DIR;                    /*!<  AES Data Input Register,                              Address offset: 0x0C */
  __I  uint32_t DOR;                    /*!<  AES Data Output Register,                             Address offset: 0x10 */
  __O  uint32_t KEY0;                   /*!<  AES Key Register 0,                                   Address offset: 0x14 */
  __O  uint32_t KEY1;                   /*!<  AES Key Register 1,                                   Address offset: 0x18 */
  __O  uint32_t KEY2;                   /*!<  AES Key Register 2,                                   Address offset: 0x1C */
  __O  uint32_t KEY3;                   /*!<  AES Key Register 3,                                   Address offset: 0x20 */
  __O  uint32_t KEY4;                   /*!<  AES Key Register 4,                                   Address offset: 0x24 */
  __O  uint32_t KEY5;                   /*!<  AES Key Register 5,                                   Address offset: 0x28 */
  __O  uint32_t KEY6;                   /*!<  AES Key Register 6,                                   Address offset: 0x2C */
  __O  uint32_t KEY7;                   /*!<  AES Key Register 7,                                   Address offset: 0x30 */
  __IO uint32_t IVR0;                   /*!<  AES Initial Vector Register 0,                        Address offset: 0x34 */
  __IO uint32_t IVR1;                   /*!<  AES Initial Vector Register 1,                        Address offset: 0x38 */
  __IO uint32_t IVR2;                   /*!<  AES Initial Vector Register 2,                        Address offset: 0x3C */
  __IO uint32_t IVR3;                   /*!<  AES Initial Vector Register 3,                        Address offset: 0x40 */
  __IO uint32_t H0;                     /*!<  AES MultH parameter Register 0,                       Address offset: 0x44 */
  __IO uint32_t H1;                     /*!<  AES MultH parameter Register 1,                       Address offset: 0x48 */
  __IO uint32_t H2;                     /*!<  AES MultH parameter Register 2,                       Address offset: 0x4C */
  __IO uint32_t H3;                     /*!<  AES MultH parameter Register 3,                       Address offset: 0x50 */
}AES_Type;


/* ================================================================================ */
/* ================                       PAE                      ================ */
/* ================================================================================ */

typedef struct
{
  __IO uint32_t CSR;                    /*!<  ,                                                     Address offset: 0x00 */
  __IO uint32_t MLR;                    /*!<  ,                                                     Address offset: 0x04 */
  __O  uint32_t MPR;                    /*!<  ,                                                     Address offset: 0x08 */
  __IO uint32_t M0CFG;                  /*!<  ,                                                     Address offset: 0x0C */
  __IO uint32_t M1CFG;                  /*!<  ,                                                     Address offset: 0x10 */
  __IO uint32_t M2CFG;                  /*!<  ,                                                     Address offset: 0x14 */
  __IO uint32_t M3CFG;                  /*!<  ,                                                     Address offset: 0x18 */
  __O  uint32_t WORD;                   /*!<  ,                                                     Address offset: 0x1C */
}PAE_Type;


/* ================================================================================ */
/* ================                     HASH                       ================ */
/* ================================================================================ */

typedef struct
{
  __IO uint32_t CSR;                    /*!<  Hash Control and Status Register,                     Address offset: 0x00 */
  __IO uint32_t DTR;                    /*!<  Hash Data Type Register,                              Address offset: 0x04 */
}HASH_Type;


/* ================================================================================ */
/* ================                       RNG                      ================ */
/* ================================================================================ */

typedef struct
{
  __IO uint32_t CR;                     /*!<  Random Number Generator Control Register,             Address offset: 0x00 */
  __I  uint32_t DOR;                    /*!<  Random Number Generator Data Output Register,         Address offset: 0x04 */
  __IO uint32_t RSV1[2];                /*!<  RESERVED REGISTER,                                    Address offset: 0x08 */
  __IO uint32_t SR;                     /*!<  Random Number Generator Status Register,              Address offset: 0x10 */
  __IO uint32_t CRC_CR;                 /*!<  CRC Control Register,                                 Address offset: 0x14 */
  __IO uint32_t CRC_DIR;                /*!<  CRC Data input Register,                              Address offset: 0x18 */
  __IO uint32_t CRC_SR;                 /*!<  CRC Status Register,                                  Address offset: 0x1C */
}RNG_Type;


/* ================================================================================ */
/* ================                      COMP                      ================ */
/* ================================================================================ */

typedef struct
{
  __IO uint32_t CR1;                    /*!<  ,                                                     Address offset: 0x00 */
  __IO uint32_t CR2;                    /*!<  ,                                                     Address offset: 0x04 */
  __IO uint32_t ICR;                    /*!<  ,                                                     Address offset: 0x08 */
  __IO uint32_t ISR;                    /*!<  ,                                                     Address offset: 0x0C */
  __IO uint32_t PCR;                    /*!<  ,                                                     Address offset: 0x010 */ 
}COMP_Type;


/* ================================================================================ */
/* ================                      I2C                       ================ */
/* ================================================================================ */

typedef struct
{
  __IO uint32_t CFGR;               /*!<  I2C Master Config Register,                           Address offset: 0x00 */
  __IO uint32_t CR;                 /*!<  I2C Master Control Register,                          Address offset: 0x04 */
  __IO uint32_t IER;                /*!<  I2C Master Interrupt Enable Register,                 Address offset: 0x08 */
  __IO uint32_t ISR;                /*!<  I2C Master Interrupt Status Register,                 Address offset: 0x0C */
  __IO uint32_t SR;                 /*!<  I2C Master Status Register,                           Address offset: 0x10 */
  __IO uint32_t BRG;                /*!<  I2C Master Baud Rate Generation Register,             Address offset: 0x14 */
  __IO uint32_t BUF;                /*!<  I2C Master Buffer Register,                           Address offset: 0x18 */
  __IO uint32_t TIMING;             /*!<  I2C Master Timing Register,                           Address offset: 0x1C */
  __IO uint32_t TO;                 /*!<  I2C Master Time-Out Register,                         Address offset: 0x20 */
}I2C_Type;


/* ================================================================================ */
/* ================                      UARTIR                    ================ */
/* ================================================================================ */

typedef struct
{
  __IO uint32_t CR;                   /*!<  Infrared modulation Control Register,                 Address offset: 0x00 */
}UARTIR_Type;


/* ================================================================================ */
/* ================                      UART                      ================ */
/* ================================================================================ */

typedef struct
{
  __IO uint32_t CSR;                    /*!<  UARTx Control Status Register,                        Address offset: 0x00 */
  __IO uint32_t IER;                    /*!<  UARTx Interrupt Enable Register,                      Address offset: 0x04 */
  __IO uint32_t ISR;                    /*!<  UARTx Interrupt Status Register,                      Address offset: 0x08 */
  __IO uint32_t TODR;                   /*!<  UARTx Time-Out and Delay Register,                    Address offset: 0x0C */
  __I  uint32_t RXBUF;                  /*!<  UARTx Receive Buffer,                                 Address offset: 0x10 */
  __IO uint32_t TXBUF;                  /*!<  UARTx Transmit Buffer,                                Address offset: 0x14 */
  __IO uint32_t BGR;                    /*!<  UARTx Baud rate Generator Register,                   Address offset: 0x18 */
}UART_Type;


/* ================================================================================ */
/* ================                       LPUART                   ================ */
/* ================================================================================ */

typedef struct
{
  __IO uint32_t CSR;                    /*!<   Control Status Register,                             Address offset: 0x00 */
  __IO uint32_t IER;                    /*!<   Interrupt Enable Register,                           Address offset: 0x04 */
  __IO uint32_t ISR;                    /*!<   Interrupt Status Register,                           Address offset: 0x08 */
  __IO uint32_t BMR;                    /*!<   Baud rate Modulation Register,                       Address offset: 0x0C */
  __IO uint32_t RXBUF;                  /*!<   Receive Buffer Register,                             Address offset: 0x10 */
  __IO uint32_t TXBUF;                  /*!<   Transmit Buffer Register,                            Address offset: 0x14 */
  __IO uint32_t DMR;                    /*!<   data Matching Register,                              Address offset: 0x18 */
}LPUART_Type;


/* ================================================================================ */
/* ================                      SPI                       ================ */
/* ================================================================================ */

typedef struct
{
  __IO uint32_t CR1;                    /*!<  SPIx Control Register1,                               Address offset: 0x00 */
  __IO uint32_t CR2;                    /*!<  SPIx Control Register2,                               Address offset: 0x04 */
  __O  uint32_t CR3;                    /*!<  SPIx Control Register3,                               Address offset: 0x08 */
  __IO uint32_t IER;                    /*!<  SPIx Interrupt Enable Register,                       Address offset: 0x0C */
  __IO uint32_t ISR;                    /*!<  SPIx Status Register,                                 Address offset: 0x10 */
  __O  uint32_t TXBUF;                  /*!<  SPIx Transmit Buffer,                                 Address offset: 0x14 */
  __I  uint32_t RXBUF;                  /*!<  SPIx Receive Buffer,                                  Address offset: 0x18 */
}SPI_Type;


/* ================================================================================ */
/* ================                      U7816                     ================ */
/* ================================================================================ */

typedef struct
{
  __IO uint32_t CR;                     /*!<  U7816 Control Register,                               Address offset: 0x00 */
  __IO uint32_t FFR;                    /*!<  U7816 Frame Format Register,                          Address offset: 0x04 */
  __IO uint32_t EGTR;                   /*!<  U7816 Extra Guard Time Register,                      Address offset: 0x08 */
  __IO uint32_t PSC;                    /*!<  U7816 Prescaler Register,                             Address offset: 0x0C */
  __IO uint32_t BGR;                    /*!<  U7816 Baud rate Generator Register,                   Address offset: 0x10 */
  __I  uint32_t RXBUF;                  /*!<  U7816 Receive Buffer,                                 Address offset: 0x14 */
  __O  uint32_t TXBUF;                  /*!<  U7816 Transmit Buffer,                                Address offset: 0x18 */
  __IO uint32_t IER;                    /*!<  U7816 Interrupt Enable Register,                      Address offset: 0x1C */
  __IO uint32_t ISR;                    /*!<  U7816 Interrupt Status Register,                      Address offset: 0x20 */
}U7816_Type;


/* ================================================================================ */
/* ================                      QSPI                      ================ */
/* ================================================================================ */

typedef struct
{
  __IO uint32_t CR;                     /*!<  QSPI Control Register,                                Address offset: 0x00 */
  __IO uint32_t CFG;                    /*!<   Config Register,                                     Address offset: 0x04 */
  __IO uint32_t SR;                     /*!<  QSPI Status Register,                                 Address offset: 0x08 */
  __IO uint32_t DATALEN;                /*!<  QSPI Data Length Register,                            Address offset: 0x0C */
  __IO uint32_t CCR;                    /*!<  QSPI Communication Config Register,                   Address offset: 0x10 */
  __IO uint32_t ADDR;                   /*!<  QSPI Address Register,                                Address offset: 0x14 */
  __IO uint32_t ABR;                    /*!<  QSPI Alternate Bytes Register,                        Address offset: 0x18 */
  __IO uint32_t DR;                     /*!<  QSPI Data Register,                                   Address offset: 0x1C */
  __IO uint32_t SMSK;                   /*!<  QSPI Status Mask Register,                            Address offset: 0x20 */
  __IO uint32_t SMAT;                   /*!<  QSPI Status Match Register,                           Address offset: 0x24 */
  __IO uint32_t PITV;                   /*!<  QSPI Polling Interval Register,                       Address offset: 0x28 */
  __IO uint32_t TO;                     /*!<  QSPI Time Out Register,                               Address offset: 0x2C */
}QSPI_Type;


/* ================================================================================ */
/* ================                       DMA                      ================ */
/* ================================================================================ */

typedef struct
{
  __IO uint32_t GCR;                    /*!<   Global Control Register,                             Address offset: 0x00 */
  __IO uint32_t CH0CR;                  /*!<  Channel 0 Control Register,                           Address offset: 0x04 */
  __IO uint32_t CH0MAR;                 /*!<  Channel 0 Memory Address Register,                    Address offset: 0x08 */
  __IO uint32_t CH1CR;                  /*!<  Channel 1 Control Register,                           Address offset: 0x0C */
  __IO uint32_t CH1MAR;                 /*!<  Channel 1 Memory Address Register,                    Address offset: 0x10 */
  __IO uint32_t CH2CR;                  /*!<  Channel 2 Control Register,                           Address offset: 0x14 */
  __IO uint32_t CH2MAR;                 /*!<  Channel 2 Memory Address Register,                    Address offset: 0x18 */
  __IO uint32_t CH3CR;                  /*!<  Channel 3 Control Register,                           Address offset: 0x1C */
  __IO uint32_t CH3MAR;                 /*!<  Channel 3 Memory Address Register,                    Address offset: 0x20 */
  __IO uint32_t CH4CR;                  /*!<  Channel 4 Control Register,                           Address offset: 0x24 */
  __IO uint32_t CH4MAR;                 /*!<  Channel 4 Memory Address Register,                    Address offset: 0x28 */
  __IO uint32_t CH5CR;                  /*!<  Channel 5 Control Register,                           Address offset: 0x2C */
  __IO uint32_t CH5MAR;                 /*!<  Channel 5 Memory Address Register,                    Address offset: 0x30 */
  __IO uint32_t CH6CR;                  /*!<  Channel 6 Control Register,                           Address offset: 0x34 */
  __IO uint32_t CH6MAR;                 /*!<  Channel 6 Memory Address Register,                    Address offset: 0x38 */
  __IO uint32_t CH7CR;                  /*!<  Channel 7 Control Register,                           Address offset: 0x3C */
  __IO uint32_t CH7MAR;                 /*!<  Channel 7 Memory Address Register,                    Address offset: 0x40 */
  __IO uint32_t CH8CR;                  /*!<  Channel 8 Control Register,                           Address offset: 0x44 */
  __IO uint32_t CH8MAR;                 /*!<  Channel 8 Memory Address Register,                    Address offset: 0x48 */
  __IO uint32_t CH9CR;                  /*!<  Channel 9 Control Register,                           Address offset: 0x4C */
  __IO uint32_t CH9MAR;                 /*!<  Channel 9 Memory Address Register,                    Address offset: 0x50 */
  __IO uint32_t CH10CR;                 /*!<  Channel 10 Control Register,                          Address offset: 0x54 */
  __IO uint32_t CH10MAR;                /*!<  Channel 10 Memory Address Register,                   Address offset: 0x58 */
  __IO uint32_t CH11CR;                 /*!<  Channel 11 Control Register,                          Address offset: 0x5C */
  __IO uint32_t CH11FAR;                /*!<  Channel 11 Flash Address Register,                    Address offset: 0x60 */
  __IO uint32_t CH11RAR;                /*!<  Channel 11 RAM Address Register,                      Address offset: 0x64 */
  __IO uint32_t ISR;                    /*!<  DMA Interrupt Status Register,                        Address offset: 0x68 */
  __IO uint32_t RSV1[37];               /*!<  RESERVED REGISTER,                                    Address offset: 0x6C */
  __IO uint32_t CH0CSR;                 /*!<  Channel 0 Control Shadow Register,                    Address offset: 0x100 */
  __IO uint32_t CH0MASR;                /*!<  Channel 0 Memory Address Shadow Register,             Address offset: 0x104 */
  __IO uint32_t CH1CSR;                 /*!<  Channel 0 Control Shadow Register,                    Address offset: 0x108 */
  __IO uint32_t CH1MASR;                /*!<  Channel 0 Memory Address Shadow Register,             Address offset: 0x10C */
  __IO uint32_t CH2CSR;                 /*!<  Channel 0 Control Shadow Register,                    Address offset: 0x110 */
  __IO uint32_t CH2MASR;                /*!<  Channel 0 Memory Address Shadow Register,             Address offset: 0x114 */
  __IO uint32_t CH3CSR;                 /*!<  Channel 0 Control Shadow Register,                    Address offset: 0x118 */
  __IO uint32_t CH3MASR;                /*!<  Channel 0 Memory Address Shadow Register,             Address offset: 0x11C */
  __IO uint32_t CH4CSR;                 /*!<  Channel 0 Control Shadow Register,                    Address offset: 0x120 */
  __IO uint32_t CH4MASR;                /*!<  Channel 0 Memory Address Shadow Register,             Address offset: 0x124 */
  __IO uint32_t CH5CSR;                 /*!<  Channel 0 Control Shadow Register,                    Address offset: 0x128 */
  __IO uint32_t CH5MASR;                /*!<  Channel 0 Memory Address Shadow Register,             Address offset: 0x12C */
  __IO uint32_t CH6CSR;                 /*!<  Channel 0 Control Shadow Register,                    Address offset: 0x130 */
  __IO uint32_t CH6MASR;                /*!<  Channel 0 Memory Address Shadow Register,             Address offset: 0x134 */
}DMA_Type;


/* ================================================================================ */
/* ================                       CRC                      ================ */
/* ================================================================================ */

typedef struct
{
  __IO uint32_t DR;                     /*!<  CRC Data Register,                                    Address offset: 0x00 */
  __IO uint32_t CR;                     /*!<  CRC Control Register,                                 Address offset: 0x04 */
  __IO uint32_t LFSR;                    /*!<                     Address offset: 0x08 */
  __IO uint32_t XOR;                    /*!<                                Address offset: 0x0C */
  __IO uint32_t RSV1[3];
  __IO uint32_t POLY;                   /*!<  CRC Polynominal Register,                             Address offset: 0x1C */
}CRC_Type;


/* ================================================================================ */
/* ================                       BT                       ================ */
/* ================================================================================ */

typedef struct
{
  __IO uint32_t CR1;                    /*!<  Basic Timer 1 Control Register 1,                     Address offset: 0x00 */
  __IO uint32_t CR2;                    /*!<  Basic Timer 1 Control Register 2,                     Address offset: 0x04 */
  __IO uint32_t CFGR1;                  /*!<  Basic Timer 1 Config Register 1,                      Address offset: 0x08 */
  __IO uint32_t CFGR2;                  /*!<  Basic Timer 1 Config Register 2,                      Address offset: 0x0C */
  __IO uint32_t PRES;                   /*!<  Basic Timer 1 Prescaler Register,                     Address offset: 0x10 */
  __IO uint32_t LOADCR;                 /*!<  Basic Timer 1 Load Control Register,                  Address offset: 0x14 */
  __I  uint32_t CNTL;                   /*!<  Basic Timer 1 Counter Low,                            Address offset: 0x18 */
  __I  uint32_t CNTH;                   /*!<  Basic Timer 1 Counter High,                           Address offset: 0x1C */
  __IO uint32_t PRESET;                 /*!<  Basic Timer 1 Preset Register,                        Address offset: 0x20 */
  __IO uint32_t LOADL;                  /*!<  Basic Timer 1 Load Register Low,                      Address offset: 0x24 */
  __IO uint32_t LOADH;                  /*!<  Basic Timer 1 Load Register High,                     Address offset: 0x28 */
  __IO uint32_t CMPL ;                  /*!<  Basic Timer 1 Compare Register Low,                   Address offset: 0x2C */
  __IO uint32_t CMPH ;                  /*!<  Basic Timer 1 Compare Register High,                  Address offset: 0x30 */
  __IO uint32_t OUTCNT;                 /*!<  Basic Timer 1 Output Counter,                         Address offset: 0x34 */
  __IO uint32_t OCR;                    /*!<  Basic Timer 1 Output Control Register,                Address offset: 0x38 */
  __IO uint32_t IER;                    /*!<  Basic Timer 1 Interrupt Enable Register,              Address offset: 0x3C */
  __IO uint32_t ISR;                    /*!<  Basic Timer 1 Interrupt Status Register,              Address offset: 0x40 */
}BT_Type;


/* ================================================================================ */
/* ================                       ET                       ================ */
/* ================================================================================ */

typedef struct
{
  __IO uint32_t CR;                     /*!<  ETx Control Register,                                 Address offset: 0x00 */
  __IO uint32_t INSR;                   /*!<  ETx Input Select Register,                            Address offset: 0x04 */
  __IO uint32_t PSCR1;                  /*!<  ETx Prescaler Register1,                              Address offset: 0x08 */
  __IO uint32_t PSCR2;                  /*!<  ETx Prescaler Register2,                              Address offset: 0x0C */
  __IO uint32_t IVR;                    /*!<  ETx Initial Value Register,                           Address offset: 0x10 */
  __IO uint32_t CMPR;                   /*!<  ETx Compare Register,                                 Address offset: 0x14 */
  __IO uint32_t IER;                    /*!<  ETx Interrupt Enable Register,                        Address offset: 0x18 */
  __IO uint32_t ISR;                    /*!<  ETx Interrupt Status Register,                        Address offset: 0x1C */
}ET_Type;

/* ================================================================================ */
/* ================                      ETIM EXTEND               ================ */
/* ================================================================================ */
typedef struct
{
  __IO uint32_t CNT1;					/*!< ETIMER COUNTER,                  Address offset: 0x00 */ 
  __IO uint32_t CNT2;					/*!< ETIMER COUNTER,                  Address offset: 0x04 */ 
  __IO uint32_t CNT3;					/*!< ETIMER COUNTER,                  Address offset: 0x08 */ 
  __IO uint32_t CNT4;					/*!< ETIMER COUNTER,                  Address offset: 0x0C */ 
}ETCNT_Type;

/* ================================================================================ */
/* ================                      BSTIM                     ================ */
/* ================================================================================ */

typedef struct
{
  __IO uint32_t CR1;                    /*!<  BSTIM Control Register1,                              Address offset: 0x00 */
  __IO uint32_t CR2;                    /*!<  BSTIM Control Register2,                              Address offset: 0x04 */
  __IO uint32_t RSV1;                   /*!<  RESERVED REGISTER,                                    Address offset: 0x08 */
  __IO uint32_t IER;                    /*!<  BSTIM Interrupt Enable Register,                      Address offset: 0x0C */
  __IO uint32_t ISR;                    /*!<  BSTIM Interrupt Status Register,                      Address offset: 0x10 */
  __O  uint32_t EGR;                    /*!<  BSTIM Event Generation Register,                      Address offset: 0x14 */
  __IO uint32_t RSV2[3];                /*!<  RESERVED REGISTER,                                    Address offset: 0x18 */
  __IO uint32_t CNTR;                   /*!<  BSTIM Counter Register,                               Address offset: 0x24 */
  __IO uint32_t PSCR;                   /*!<  BSTIM Prescaler Register,                             Address offset: 0x28 */
  __IO uint32_t ARR;                    /*!<  BSTIM Auto-Reload Register,                           Address offset: 0x2C */
}BSTIM_Type;


/* ================================================================================ */
/* ================                       LPTIM                    ================ */
/* ================================================================================ */

typedef struct
{
  __IO uint32_t CFGR;                   /*!<   LPTIM Config Register,                                Address offset: 0x00 */
  __I  uint32_t CNTR;                   /*!<   LPTIM Counter Register,                               Address offset: 0x04 */
  __IO uint32_t CCSR;                   /*!<   LPTIM Capture/Compare Control and Status Register,    Address offset: 0x08 */
  __IO uint32_t ARR;                    /*!<   LPTIM Auto-Reload Register,                           Address offset: 0x0C */
  __IO uint32_t IER;                    /*!<   LPTIM Interrupt Enable Register,                      Address offset: 0x10 */
  __IO uint32_t ISR;                    /*!<   LPTIM Interrupt Status Register,                      Address offset: 0x14 */
  __IO uint32_t CR;                     /*!<   LPTIM Control Register,                               Address offset: 0x18 */
  __IO uint32_t RSV1;                   /*!<   LPTIM RESERVED REGISTER,                              Address offset: 0x1C */
  __IO uint32_t CCR1;                   /*!<   LPTIM Capture/Compare Register1,                      Address offset: 0x20 */
  __IO uint32_t CCR2;                   /*!<   LPTIM Capture/Compare Register2,                      Address offset: 0x24 */
  __IO uint32_t CCR3;                   /*!<   LPTIM Capture/Compare Register3,                      Address offset: 0x28 */
  __IO uint32_t CCR4;                   /*!<   LPTIM Capture/Compare Register4,                      Address offset: 0x2C */
} LPTIM_Type;


/* ================================================================================ */
/* ================                      RTC                       ================ */
/* ================================================================================ */

typedef struct
{
  __IO uint32_t WER;                    /*!<  RTC Write Enable Register,                          Address offset: 0x00 */
  __IO uint32_t IER;                    /*!<  RTC Interrupt Enable Register,                      Address offset: 0x04 */
  __IO uint32_t ISR;                    /*!<  RTC Interrupt Status Register,                      Address offset: 0x08 */
  __IO uint32_t BCDSEC;                 /*!<  RTC BCD time register,                              Address offset: 0x0C */
  __IO uint32_t BCDMIN;                 /*!<  RTC BCD time register,                              Address offset: 0x10 */
  __IO uint32_t BCDHOUR;                /*!<  RTC BCD time register,                              Address offset: 0x14 */
  __IO uint32_t BCDDATE;                /*!<  RTC BCD time register,                              Address offset: 0x18 */
  __IO uint32_t BCDWEEK;                /*!<  RTC BCD time register,                              Address offset: 0x1C */
  __IO uint32_t BCDMONTH;               /*!<  RTC BCD time register,                              Address offset: 0x20 */
  __IO uint32_t BCDYEAR;                /*!<  RTC BCD time register,                              Address offset: 0x24 */
  __IO uint32_t ALARM;                  /*!<  RTC Alarm Register,                                 Address offset: 0x28 */
  __IO uint32_t TMSEL;                  /*!<  RTC Time Mark Select,                               Address offset: 0x2C */
  __IO uint32_t ADJUST;                 /*!<  RTC time Adjust Register,                           Address offset: 0x30 */
  __IO uint32_t ADSIGN;                 /*!<  RTC time Adjust Sign Register,                      Address offset: 0x34 */
  __IO uint32_t VCAL;                   /*!<  RTC Virtual Calibration Register,                   Address offset: 0x38 */
  __IO uint32_t MSCNT;                  /*!<  RTC Milli-Second Register,                          Address offset: 0x3C */
  __IO uint32_t CALSTEP;                /*!<  RTC Calibration Step Register,                      Address offset: 0x40 */
  __I  uint32_t ADCNT;                  /*!<  RTC Adjust Counter Register,                        Address offset: 0x44 */
  __I  uint32_t SSR;                    /*!<  RTC Sub-Second Register,                            Address offset: 0x48 */
  __IO uint32_t SSA;                    /*!<  RTC Sub-Second Alarm Register,                      Address offset: 0x4C */
  __IO uint32_t DTR;                    /*!<  RTC Time Mark Duty Cycle Register,                  Address offset: 0x50 */
  __IO uint32_t RSV1[10];               /*!<  RESERVED REGISTER,                                  Address offset: 0x54 */
  __IO uint32_t CR;                     /*!<  RTC Control Register,                               Address offset: 0x7C */
}RTC_Type;

/* ================================================================================ */
/* ================                       LCD                      ================ */
/* ================================================================================ */

typedef struct
{
  __IO uint32_t CR;                     /*!<  LCD Control Register,                                 Address offset: 0x00 */
  __IO uint32_t TEST;                   /*!<  LCD test Register,                                    Address offset: 0x04 */
  __IO uint32_t FCR;                    /*!<  LCD Frequency Control Register,                       Address offset: 0x08 */
  __IO uint32_t FLKT;                   /*!<  LCD Flick Time Register,                              Address offset: 0x0C */
  __IO uint32_t RSV1;                   /*!<  RESERVED REGISTER,                                    Address offset: 0x10 */
  __IO uint32_t IER;                    /*!<  LCD Interrupt Enable Register,                        Address offset: 0x14 */
  __IO uint32_t ISR;                    /*!<  LCD Interrupt Status Register,                        Address offset: 0x18 */
  __IO uint32_t RSV2[2];                /*!<  RESERVED REGISTER,                                    Address offset: 0x1C */
  __IO uint32_t DATA0;                  /*!<  data buffer registers 0,                              Address offset: 0x24 */
  __IO uint32_t DATA1;                  /*!<  data buffer registers 1,                              Address offset: 0x28 */
  __IO uint32_t DATA2;                  /*!<  data buffer registers 2,                              Address offset: 0x2C */
  __IO uint32_t DATA3;                  /*!<  data buffer registers 3,                              Address offset: 0x30 */
  __IO uint32_t DATA4;                  /*!<  data buffer registers 4,                              Address offset: 0x34 */
  __IO uint32_t DATA5;                  /*!<  data buffer registers 5,                              Address offset: 0x38 */
  __IO uint32_t DATA6;                  /*!<  data buffer registers 6,                              Address offset: 0x3C */
  __IO uint32_t DATA7;                  /*!<  data buffer registers 7,                              Address offset: 0x40 */
  __IO uint32_t DATA8;                  /*!<  data buffer registers 8,                              Address offset: 0x44 */
  __IO uint32_t DATA9;                  /*!<  data buffer registers 9,                              Address offset: 0x48 */
  __IO uint32_t RSV3[1];                /*!<  RESERVED REGISTER,                                    Address offset: 0x4C */
  __IO uint32_t COMEN;                  /*!<  LCD COM Enable Register,                              Address offset: 0x50 */
  __IO uint32_t SEGEN0;                 /*!<  LCD SEG Enable Register0,                             Address offset: 0x54 */
  __IO uint32_t SEGEN1;                 /*!<  LCD SEG Enable Register1,                             Address offset: 0x58 */
  __IO uint32_t BSTCR;                  /*!<  LCD Boost Control Register,                           Address offset: 0x5C */
}LCD_Type;


/* ================================================================================ */
/* ================                       ADC                      ================ */
/* ================================================================================ */

typedef struct
{
  __IO uint32_t CR;                     /*!<  ADC Control Register,                                 Address offset: 0x00 */
  __IO uint32_t TRIM;                   /*!<  ADC Trim Register,                                    Address offset: 0x04 */
  __I  uint32_t DR;                     /*!<  ADC Data Register,                                    Address offset: 0x08 */
  __IO uint32_t ISR;                    /*!<  ADC Interrupt Status Regsiter,                        Address offset: 0x0C */
  __IO uint32_t CFGR;                   /*!<  ADC Config Register,                                  Address offset: 0x10 */
}ADC_Type;


/* ================================================================================ */
/* ================                        CIC                   ================ */
/* ================================================================================ */

typedef struct
{
  __I  uint32_t DR;                 /*!<  CIC Output Data Register,                             Address offset: 0x00 */
  __IO uint32_t OS;                 /*!<  CIC Output Offset Register,                           Address offset: 0x04 */
  __I  uint32_t USDR;               /*!<  CIC Unsigned Data Register,                           Address offset: 0x08 */
  __IO uint32_t CR;                 /*!<  CIC Control Register,                                 Address offset: 0x0C */
  __IO uint32_t ISR;                /*!<  CIC Interrupt Status Register,                        Address offset: 0x10 */
}CIC_Type;


/* ================================================================================ */
/* ================                      GPIO                      ================ */
/* ================================================================================ */

typedef struct
{
  __IO uint32_t INEN;                   /*!<  GPIOx Input Enable Register,                          Address offset: 0x00 */
  __IO uint32_t PUEN;                   /*!<  GPIOx Pull-Up Enable Register,                        Address offset: 0x04 */
  __IO uint32_t ODEN;                   /*!<   Open-Drain Enable Register,                          Address offset: 0x08 */
  __IO uint32_t FCR;                    /*!<   Function Control Register,                           Address offset: 0x0C */
  __IO uint32_t DO;                     /*!<   Data Output Register,                                Address offset: 0x10 */
  __O uint32_t DSET;                    /*!<   Data Set Register,                                   Address offset: 0x14 */
  __O uint32_t DRST;                    /*!<   Data Reset Register,                                 Address offset: 0x18 */
  __I uint32_t DIN;                     /*!<   Data Input Register,                                 Address offset: 0x1C */
  __IO uint32_t DFS;                    /*!<   Digital Function Select,                             Address offset: 0x20 */
  __IO uint32_t RSV;                     /*!<  RESERVED REGISTER,                          Address offset: 0x24 */
  __IO uint32_t ANEN;                   /*!<  Analog channel Enable Register,                       Address offset: 0x28 */
}GPIO_Type;

/* ================================================================================ */
/* ================                      GPIOH                     ================ */
/* ================================================================================ */

typedef struct
{
  __IO uint32_t INEN;                   /*!<  GPIOx Input Enable Register,                          Address offset: 0x00 */
  __IO uint32_t PUEN;                   /*!<  GPIOx Pull-Up Enable Register,                        Address offset: 0x04 */
  __IO uint32_t FCR;                    /*!<   Function Control Register,                           Address offset: 0x08 */
  __IO uint32_t DO;                     /*!<   Data Output Register,                                Address offset: 0x0C */
  __IO uint32_t DIN;                    /*!<   Data Input Register,                                 Address offset: 0x10 */
}GPIOH_Type;

/* ================================================================================ */
/* ================                      GPIO                      ================ */
/* ================================================================================ */

typedef struct
{
  __IO uint32_t EXTISEL0;               /*!<  External Interrupt input Select Register1,            Address offset: 0x00 */
  __IO uint32_t EXTISEL1;               /*!<  External Interrupt input Select Register0,            Address offset: 0x04 */
  __IO uint32_t EXTIEDS0;               /*!<  External Interrupt Edge Select and Enable Register1,  Address offset: 0x08 */
  __IO uint32_t EXTIEDS1;               /*!<  External Interrupt Edge Select and Enable Register0,  Address offset: 0x0C */
  __IO uint32_t EXTIDF;                 /*!<  External Interrupt Digital Filter Register,           Address offset: 0x10 */
  __IO uint32_t EXTIISR;                /*!<  External Interrupt and Status Register,               Address offset: 0x14 */
  __I uint32_t EXTIDI;                  /*!<  External Interrupt Data Input Register,               Address offset: 0x18 */
  __IO uint32_t RSV1[9];                /*!<  RESERVED REGISTER,                                    Address offset: 0x1C */
  __IO uint32_t FOUTSEL;                /*!<  Frequency Output Select Register,                     Address offset: 0x40 */
  __IO uint32_t IOMCR;                  /*!<  IO MUX Control Register,                              Address offset: 0x44 */
  __IO uint32_t RSV2[62];               /*!<  RESERVED REGISTER,                                    Address offset: 0x48 */
  __IO uint32_t PINWKEN;                /*!<  Wakeup Enable Register,                               Address offset: 0x140 */
}GPIO_COMMON_Type;

/* --------------------  End of section using anonymous unions  ------------------- */
#if defined(__CC_ARM)
  #pragma pop
#elif defined(__ICCARM__)
  /* leave anonymous unions enabled */
#elif defined(__GNUC__)
/* anonymous unions are enabled by default */
#elif defined(__TMS470__)
  /* anonymous unions are enabled by default */
#elif defined(__TASKING__)
  #pragma warning restore
#else
  #warning Not supported compiler type
#endif


/* ================================================================================ */
/* ================              CPU memory map                    ================ */
/* ================================================================================ */


/* Peripheral and SRAM base address */

#define FLASH_BASE            ((     uint32_t)0x00000000)
#define SRAM_BASE             ((     uint32_t)0x20000000)
#define PERIPH_BASE           ((     uint32_t)0x40000000)


/* ================================================================================ */
/* ================              Peripheral memory map             ================ */
/* ================================================================================ */

/* Peripheral memory map */

#define PMU_BASE                        (PERIPH_BASE        +0x00002000)
#define DBG_BASE                        (PERIPH_BASE        +0x00000000)
#define FLS_BASE                        (PERIPH_BASE        +0x00001000)
#define RMU_BASE                        (PERIPH_BASE        +0x00002800)
#define IWDT_BASE                       (PERIPH_BASE        +0x00011400)
#define WWDT_BASE                       (PERIPH_BASE        +0x00011800)
#define CMU_BASE                        (PERIPH_BASE        +0x00002400)
#define CDIF_BASE                       (PERIPH_BASE        +0x0001E000)
#define VRTC_BASE                       (PERIPH_BASE        +0x0001F800)
#define SVD_BASE                        (PERIPH_BASE        +0x00012800)
#define AES_BASE                        (PERIPH_BASE        +0x00013800)
#define PAE_BASE                        (PERIPH_BASE        +0x00001400)
#define HASH_BASE                       (PERIPH_BASE        +0x00001800)
#define RNG_BASE                        (PERIPH_BASE        +0x00013C00)
#define COMP_BASE                       (PERIPH_BASE        +0x00015400)
#define I2C0_BASE                       (PERIPH_BASE        +0x00012400)
#define I2C1_BASE                       (PERIPH_BASE        +0x00015000)
#define UARTIR_BASE                     (PERIPH_BASE        +0x00017C00)
#define UART0_BASE                      (PERIPH_BASE        +0x00012000)
#define UART1_BASE                      (PERIPH_BASE        +0x00016800)
#define UART2_BASE                      (PERIPH_BASE        +0x00016C00)
#define UART3_BASE                      (PERIPH_BASE        +0x00017000)
#define UART4_BASE                      (PERIPH_BASE        +0x00017400)
#define UART5_BASE                      (PERIPH_BASE        +0x00017800)
#define LPUART0_BASE                    (PERIPH_BASE        +0x00014000)
#define LPUART1_BASE                    (PERIPH_BASE        +0x00014400)
#define SPI0_BASE                       (PERIPH_BASE        +0x00010400)
#define SPI1_BASE                       (PERIPH_BASE        +0x00010800)
#define SPI2_BASE                       (PERIPH_BASE        +0x00014800)
#define SPI3_BASE                       (PERIPH_BASE        +0x00014C00)
#define SPI4_BASE                       (PERIPH_BASE        +0x00016400)
#define U7816_BASE                      (PERIPH_BASE        +0x00011C00)
#define QSPI_BASE                       (PERIPH_BASE        +0x00000800)
#define DMA_BASE                        (PERIPH_BASE        +0x00000400)
#define CRC_BASE                        (PERIPH_BASE        +0x00010000)
#define BT1_BASE                        (PERIPH_BASE        +0x00013000)
#define BT2_BASE                        (PERIPH_BASE        +0x00013044)
#define ET1_BASE                        (PERIPH_BASE        +0x00013090)
#define ET2_BASE                        (PERIPH_BASE        +0x000130B0)
#define ET3_BASE                        (PERIPH_BASE        +0x000130D0)
#define ET4_BASE                        (PERIPH_BASE        +0x000130F0)
#define ETCNT_BASE						(PERIPH_BASE		+0x00013110)
#define BSTIM_BASE                      (PERIPH_BASE        +0x00016000)
#define LPTIM_BASE                      (PERIPH_BASE        +0x00013400)
#define RTC_BASE                        (PERIPH_BASE        +0x00011000)
#define LCD_BASE                        (PERIPH_BASE        +0x00010C00)
#define ADC_BASE                        (PERIPH_BASE        +0x0001FA00)
#define CIC_BASE                        (PERIPH_BASE        +0x00015C00)
#define GPIOA_BASE                      (PERIPH_BASE        +0x00000C00)
#define GPIOB_BASE                      (PERIPH_BASE        +0x00000C40)
#define GPIOC_BASE                      (PERIPH_BASE        +0x00000C80)
#define GPIOD_BASE                      (PERIPH_BASE        +0x00000CC0)
#define GPIOE_BASE                      (PERIPH_BASE        +0x00000D00)
#define GPIOF_BASE                      (PERIPH_BASE        +0x00000D40)
#define GPIOG_BASE                      (PERIPH_BASE        +0x00000D80)
#define GPIOH_BASE                      (PERIPH_BASE        +0x0001FC00)
#define GPIO_BASE                       (PERIPH_BASE        +0x00000DC0)

/* ================================================================================ */
/* ================             Peripheral declaration             ================ */
/* ================================================================================ */

#define PMU                             ((PMU_Type          *) PMU_BASE         )
#define DBG                             ((DBG_Type          *) DBG_BASE         )
#define FLS                             ((FLS_Type          *) FLS_BASE         )
#define RMU                             ((RMU_Type          *) RMU_BASE         )
#define IWDT                            ((IWDT_Type         *) IWDT_BASE        )
#define WWDT                            ((WWDT_Type         *) WWDT_BASE        )
#define CMU                             ((CMU_Type          *) CMU_BASE         )
#define CDIF                            ((CDIF_Type         *) CDIF_BASE        )
#define VRTC                            ((VRTC_Type         *) VRTC_BASE        )
#define SVD                             ((SVD_Type          *) SVD_BASE         )
#define AES                             ((AES_Type          *) AES_BASE         )
#define PAE                             ((PAE_Type          *) PAE_BASE         )
#define HASH                            ((HASH_Type         *) HASH_BASE        )
#define RNG                             ((RNG_Type          *) RNG_BASE         )
#define COMP                            ((COMP_Type         *) COMP_BASE        )
#define I2C0                            ((I2C_Type          *) I2C0_BASE        )
#define I2C1                            ((I2C_Type          *) I2C1_BASE        )
#define UARTIR                          ((UARTIR_Type       *) UARTIR_BASE      )
#define UART0                           ((UART_Type         *) UART0_BASE       )
#define UART1                           ((UART_Type         *) UART1_BASE       )
#define UART2                           ((UART_Type         *) UART2_BASE       )
#define UART3                           ((UART_Type         *) UART3_BASE       )
#define UART4                           ((UART_Type         *) UART4_BASE       )
#define UART5                           ((UART_Type         *) UART5_BASE       )
#define LPUART0                         ((LPUART_Type       *) LPUART0_BASE     )
#define LPUART1                         ((LPUART_Type       *) LPUART1_BASE     )
#define SPI0                            ((SPI_Type          *) SPI0_BASE        )
#define SPI1                            ((SPI_Type          *) SPI1_BASE        )
#define SPI2                            ((SPI_Type          *) SPI2_BASE        )
#define SPI3                            ((SPI_Type          *) SPI3_BASE        )
#define SPI4                            ((SPI_Type          *) SPI4_BASE        )
#define U7816                           ((U7816_Type        *) U7816_BASE       )
#define QSPI                            ((QSPI_Type         *) QSPI_BASE        )
#define DMA                             ((DMA_Type          *) DMA_BASE         )
#define CRC                             ((CRC_Type          *) CRC_BASE         )
#define BT1                             ((BT_Type           *) BT1_BASE         )
#define BT2                             ((BT_Type           *) BT2_BASE         )
#define ET1                             ((ET_Type           *) ET1_BASE         )
#define ET2                             ((ET_Type           *) ET2_BASE         )
#define ET3                             ((ET_Type           *) ET3_BASE         )
#define ET4                             ((ET_Type           *) ET4_BASE         )
#define ETCNT							((ETCNT_Type        *) ETCNT_BASE		)
#define BSTIM                           ((BSTIM_Type        *) BSTIM_BASE       )
#define LPTIM                           ((LPTIM_Type        *) LPTIM_BASE       )
#define RTC                             ((RTC_Type          *) RTC_BASE         )
#define LCD                             ((LCD_Type          *) LCD_BASE         )
#define ADC                             ((ADC_Type          *) ADC_BASE         )
#define CIC                             ((CIC_Type          *) CIC_BASE         )
#define GPIOA                           ((GPIO_Type         *) GPIOA_BASE       )
#define GPIOB                           ((GPIO_Type         *) GPIOB_BASE       )
#define GPIOC                           ((GPIO_Type         *) GPIOC_BASE       )
#define GPIOD                           ((GPIO_Type         *) GPIOD_BASE       )
#define GPIOE                           ((GPIO_Type         *) GPIOE_BASE       )
#define GPIOF                           ((GPIO_Type         *) GPIOF_BASE       )
#define GPIOG                           ((GPIO_Type         *) GPIOG_BASE       )
#define GPIOH                           ((GPIOH_Type        *) GPIOH_BASE       )
#define GPIO                            ((GPIO_COMMON_Type  *) GPIO_BASE        )

/* ================================================================================ */
/* ================             Peripheral include                 ================ */
/* ================================================================================ */

/** @} */ /* End of group Device_Peripheral_Registers */
/** @} */ /* End of group FM33A0XXEV */
/** @} */ /* End of group Keil */

#ifdef __cplusplus
}
#endif

#endif  /* FM33A0XXEV_H */

