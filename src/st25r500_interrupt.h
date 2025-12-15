/**
  ******************************************************************************
  * @file           : st25r500_interrupt.h
  * @brief          : ST25R500 Interrupt handling
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#ifndef ST25R500_IRQ_H
#define ST25R500_IRQ_H

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/


/*
******************************************************************************
* GLOBAL DEFINES
******************************************************************************
*/

#define ST25R500_IRQ_MASK_ALL             (uint32_t)(0xFFFFFFFFUL)  /*!< All ST25R500 interrupt sources                             */
#define ST25R500_IRQ_MASK_NONE            (uint32_t)(0x00000000UL)  /*!< No ST25R500 interrupt source                               */

/* Main interrupt register */
#define ST25R500_IRQ_MASK_SUBC_START      (uint32_t)(0x00000080U)   /*!< ST25R500 subcarrier start interrupt                        */
#define ST25R500_IRQ_MASK_COL             (uint32_t)(0x00000040U)   /*!< ST25R500 bit collision interrupt                           */
#define ST25R500_IRQ_MASK_WL              (uint32_t)(0x00000020U)   /*!< ST25R500 FIFO water level interrupt                        */
#define ST25R500_IRQ_MASK_RX_REST         (uint32_t)(0x00000010U)   /*!< ST25R500 automatic reception restart interrupt             */
#define ST25R500_IRQ_MASK_RXE             (uint32_t)(0x00000008U)   /*!< ST25R500 end of receive interrupt                          */
#define ST25R500_IRQ_MASK_RXS             (uint32_t)(0x00000004U)   /*!< ST25R500 start of receive interrupt                        */
#define ST25R500_IRQ_MASK_TXE             (uint32_t)(0x00000002U)   /*!< ST25R500 end of transmission interrupt                     */
#define ST25R500_IRQ_MASK_RX_ERR          (uint32_t)(0x00000001U)   /*!< ST25R500 Reception error interrupt                         */

/* Timer and Error interrupt register */
#define ST25R500_IRQ_MASK_GPE             (uint32_t)(0x00008000U)   /*!< ST25R500 general purpose timer expired interrupt           */
#define ST25R500_IRQ_MASK_NRE             (uint32_t)(0x00004000U)   /*!< ST25R500 no-response timer expired interrupt               */
#define ST25R500_IRQ_MASK_WPT_STOP        (uint32_t)(0x00002000U)   /*!< ST25R500 WPT stop received interrupt                       */
#define ST25R500_IRQ_MASK_WPT_FOD         (uint32_t)(0x00001000U)   /*!< ST25R500 WPT FOD received interrupt                        */
#define ST25R500_IRQ_MASK_RFU             (uint32_t)(0x00000800U)   /*!< ST25R500 RFU                                               */
#define ST25R500_IRQ_MASK_CE_SC           (uint32_t)(0x00000400U)   /*!< ST25R500 CE state change interrupt                         */
#define ST25R500_IRQ_MASK_RXE_CE          (uint32_t)(0x00000200U)   /*!< ST25R500 end of reception in CE interrupt                  */
#define ST25R500_IRQ_MASK_NFCT            (uint32_t)(0x00000100U)   /*!< ST25R500 bit rate was recognized interrupt                 */

/* Wake-up interrupt register */
#define ST25R500_IRQ_MASK_WUTME           (uint32_t)(0x00800000U)   /*!< ST25R500 WU Measure event interrupt                        */
#define ST25R500_IRQ_MASK_EOF             (uint32_t)(0x00400000U)   /*!< ST25R500 External Field Off interrupt                      */
#define ST25R500_IRQ_MASK_EON             (uint32_t)(0x00200000U)   /*!< ST25R500 External Field On interrupt                       */
#define ST25R500_IRQ_MASK_DCT             (uint32_t)(0x00100000U)   /*!< ST25R500 termination of direct command interrupt           */
#define ST25R500_IRQ_MASK_WUQ             (uint32_t)(0x00080000U)   /*!< ST25R500 wake-up Q-Channel interrupt                       */
#define ST25R500_IRQ_MASK_WUI             (uint32_t)(0x00040000U)   /*!< ST25R500 wake-up I-Channel interrupt                       */
#define ST25R500_IRQ_MASK_WUT             (uint32_t)(0x00020000U)   /*!< ST25R500 wake-up timer interrupt                           */
#define ST25R500_IRQ_MASK_OSC             (uint32_t)(0x00010000U)   /*!< ST25R500 oscillator stable interrupt                       */


/*! Holds current and previous interrupt callback pointer as well as current Interrupt status and mask */
typedef struct {
  void (*prevCallback)(void);      /*!< call back function for ST25R500 interrupt          */
  void (*callback)(void);          /*!< call back function for ST25R500 interrupt          */
  uint32_t  status;                /*!< latest interrupt status                             */
  uint32_t  mask;                  /*!< Interrupt mask. Negative mask = ST25R500 mask regs */
} st25r500Interrupt;

#endif /* ST25R500_IRQ_H */

/**
  * @}
  *
  * @}
  *
  * @}
  *
  * @}
  */
