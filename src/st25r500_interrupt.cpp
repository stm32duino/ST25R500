/**
  ******************************************************************************
  * @file           : st25r500_irq.c
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

#include "rfal_rfst25r500.h"
#include "st25r500_interrupt.h"
#include "st25r500_com.h"
#include "st25r500.h"
#include "st_errno.h"
#include "nfc_utils.h"
/*
 ******************************************************************************
 * LOCAL DATA TYPES
 ******************************************************************************
 */

/*
******************************************************************************
* GLOBAL DEFINES
******************************************************************************
*/

/*! Length of the interrupt registers       */
#define ST25R500_INT_REGS_LEN          ( (ST25R500_REG_IRQ3 - ST25R500_REG_IRQ1) + 1U )

/*
******************************************************************************
* GLOBAL VARIABLES
******************************************************************************
*/

/*
******************************************************************************
* GLOBAL FUNCTIONS
******************************************************************************
*/
void RfalRfST25R500Class::st25r500InitInterrupts(void)
{
  st25r500interrupt.callback     = NULL;
  st25r500interrupt.prevCallback = NULL;
  st25r500interrupt.status       = ST25R500_IRQ_MASK_NONE;
  st25r500interrupt.mask         = ST25R500_IRQ_MASK_NONE;
}


/*******************************************************************************/
void RfalRfST25R500Class::st25r500Isr(void)
{
  st25r500CheckForReceivedInterrupts();

  /* Check if callback is set and run it */
  if (NULL != st25r500interrupt.callback) {
    st25r500interrupt.callback();
  }
}


/*******************************************************************************/
void RfalRfST25R500Class::st25r500CheckForReceivedInterrupts(void)
{
  uint8_t  iregs[ST25R500_INT_REGS_LEN];
  uint32_t irqStatus;

#ifdef ST25R_POLL_IRQ
  /* Exit immediately in case of no IRQ */
  if (digitalRead(int_pin) == LOW) {
    return;
  }
#endif /* ST25R_POLL_IRQ */

  /* Initialize iregs */
  irqStatus = ST25R500_IRQ_MASK_NONE;
  ST_MEMSET(iregs, (int32_t)(ST25R500_IRQ_MASK_ALL & 0xFFU), ST25R500_INT_REGS_LEN);

  /* In case the IRQ is Edge (not Level) triggered read IRQs until done */
  while (digitalRead(int_pin) == HIGH) {
    st25r500ReadMultipleRegisters(ST25R500_REG_IRQ1, iregs, ST25R500_INT_REGS_LEN);

    irqStatus |= (uint32_t)iregs[0];
    irqStatus |= (uint32_t)iregs[1] << 8;
    irqStatus |= (uint32_t)iregs[2] << 16;
  }

  /* Forward all interrupts, even masked ones to application */
  st25r500interrupt.status |= irqStatus;

}


/*******************************************************************************/
void RfalRfST25R500Class::st25r500ModifyInterrupts(uint32_t clr_mask, uint32_t set_mask)
{
  uint8_t  i;
  uint32_t old_mask;
  uint32_t new_mask;


  old_mask = st25r500interrupt.mask;
  new_mask = ((~old_mask & set_mask) | (old_mask & clr_mask));
  st25r500interrupt.mask &= ~clr_mask;
  st25r500interrupt.mask |= set_mask;

  for (i = 0; i < ST25R500_INT_REGS_LEN; i++) {
    if (((new_mask >> (8U * i)) & 0xFFU) == 0U) {
      continue;
    }

    st25r500WriteRegister(ST25R500_REG_IRQ_MASK1 + i, (uint8_t)((st25r500interrupt.mask >> (8U * i)) & 0xFFU));
  }
  return;
}


/*******************************************************************************/
uint32_t RfalRfST25R500Class::st25r500WaitForInterruptsTimed(uint32_t mask, uint16_t tmo)
{
  uint32_t tmrDelay;
  uint32_t status;

  tmrDelay = timerCalculateTimer(tmo);

  /* Run until specific interrupt has happen or the timer has expired */
  do {
#ifdef ST25R_POLL_IRQ
    st25r500CheckForReceivedInterrupts();
#endif /* ST25R_POLL_IRQ */

    status = (st25r500interrupt.status & mask);
  } while (((!timerIsExpired(tmrDelay)) || (tmo == 0U)) && (status == 0U));



  status = st25r500interrupt.status & mask;

  st25r500interrupt.status &= ~status;

  return status;
}


/*******************************************************************************/
uint32_t RfalRfST25R500Class::st25r500GetInterrupt(uint32_t mask)
{
  uint32_t irqs;

  irqs = (st25r500interrupt.status & mask);
  if (irqs != ST25R500_IRQ_MASK_NONE) {
    st25r500interrupt.status &= ~irqs;
  }

  return irqs;
}


/*******************************************************************************/
void RfalRfST25R500Class::st25r500ClearAndEnableInterrupts(uint32_t mask)
{
  st25r500GetInterrupt(mask);
  st25r500EnableInterrupts(mask);
}


/*******************************************************************************/
void RfalRfST25R500Class::st25r500EnableInterrupts(uint32_t mask)
{
  st25r500ModifyInterrupts(mask, 0);
}


/*******************************************************************************/
void RfalRfST25R500Class::st25r500DisableInterrupts(uint32_t mask)
{
  st25r500ModifyInterrupts(0, mask);
}


/*******************************************************************************/
void RfalRfST25R500Class::st25r500ClearInterrupts(void)
{
  uint8_t iregs[ST25R500_INT_REGS_LEN];

  st25r500ReadMultipleRegisters(ST25R500_REG_IRQ1, iregs, ST25R500_INT_REGS_LEN);

  st25r500interrupt.status = ST25R500_IRQ_MASK_NONE;
  return;
}


/*******************************************************************************/
void RfalRfST25R500Class::st25r500IRQCallbackSet(void (*cb)(void))
{
  st25r500interrupt.prevCallback = st25r500interrupt.callback;
  st25r500interrupt.callback     = cb;
}


/*******************************************************************************/
void RfalRfST25R500Class::st25r500IRQCallbackRestore(void)
{
  st25r500interrupt.callback     = st25r500interrupt.prevCallback;
  st25r500interrupt.prevCallback = NULL;
}

