/**
  ******************************************************************************
  * @file           : st25r500_com.c
  * @brief          : Implementation of ST25R500 communication
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

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/
#include "rfal_rfst25r500.h"
#include "st25r500_com.h"
#include "st25r500.h"
#include "nfc_utils.h"

/*
******************************************************************************
* LOCAL DEFINES
******************************************************************************
*/

#define ST25R500_OPTIMIZE              true                           /*!< Optimization switch: false always write value to register     */
#define ST25R500_MOSI_IDLE             (0x00)                         /*!< ST25R500 MOSI IDLE state                                      */
#define ST25R500_BUF_LEN               (ST25R500_CMD_LEN+ST25R500_FIFO_DEPTH) /*!< ST25R500 communication buffer: CMD + FIFO length      */

/*
******************************************************************************
* MACROS
******************************************************************************
*/

/*!
 ******************************************************************************
 * \brief ST25R500 communication Repeat Start
 *
 * This method performs the required actions to repeat start a transmission
 * with ST25R500
 ******************************************************************************
 */


/*
******************************************************************************
* GLOBAL FUNCTIONS
******************************************************************************
*/

/*******************************************************************************/
ReturnCode RfalRfST25R500Class::st25r500ReadRegister(uint8_t reg, uint8_t *val)
{
  return st25r500ReadMultipleRegisters(reg, val, ST25R500_REG_LEN);
}


/*******************************************************************************/
ReturnCode RfalRfST25R500Class::st25r500ReadMultipleRegisters(uint8_t reg, uint8_t *values, uint16_t length)
{
  if (length > 0U) {

    /* Setting Transaction Parameters */
    dev_spi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE1));
    digitalWrite(cs_pin, LOW);

    uint8_t response = dev_spi->transfer((reg | ST25R500_READ_MODE));
    dev_spi->transfer((void *)values, length);

    digitalWrite(cs_pin, HIGH);
    dev_spi->endTransaction();

    if (isr_pending) {
      st25r500Isr();
      isr_pending = false;
    }
  }
  return ERR_NONE;
}


/*******************************************************************************/
ReturnCode RfalRfST25R500Class::st25r500WriteRegister(uint8_t reg, uint8_t val)
{
  uint8_t value = val;               /* MISRA 17.8: use intermediate variable */
  return st25r500WriteMultipleRegisters(reg, &value, ST25R500_REG_LEN);
}


/*******************************************************************************/
ReturnCode RfalRfST25R500Class::st25r500WriteMultipleRegisters(uint8_t reg, const uint8_t *values, uint16_t length)
{
  if (length > 0U) {

    uint8_t tx[256];

    if (values != NULL) {
      (void)memcpy(tx, values, length);
    }
    /* Setting Transaction Parameters */
    dev_spi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE1));
    digitalWrite(cs_pin, LOW);

    uint8_t response = dev_spi->transfer((reg | ST25R500_WRITE_MODE));
    dev_spi->transfer((void *) tx, length);

    digitalWrite(cs_pin, HIGH);
    dev_spi->endTransaction();

    if (isr_pending) {
      st25r500Isr();
      isr_pending = false;
    }
  }

  return ERR_NONE;
}


/*******************************************************************************/
ReturnCode RfalRfST25R500Class::st25r500WriteFifo(const uint8_t *values, uint16_t length)
{
  if (length > ST25R500_FIFO_DEPTH) {
    return ERR_PARAM;
  }

  st25r500WriteMultipleRegisters(ST25R500_FIFO_ACCESS, values, length);

  return ERR_NONE;
}


/*******************************************************************************/
ReturnCode RfalRfST25R500Class::st25r500ReadFifo(uint8_t *buf, uint16_t length)
{
  if (length > 0U) {
    if (length > ST25R500_FIFO_DEPTH) {
      return ERR_PARAM;
    }

    st25r500ReadMultipleRegisters(ST25R500_FIFO_ACCESS, buf, length);
  }

  return ERR_NONE;
}


/*******************************************************************************/
ReturnCode RfalRfST25R500Class::st25r500ExecuteCommand(uint8_t cmd)
{
  dev_spi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE1));
  digitalWrite(cs_pin, LOW);

  dev_spi->transfer((cmd | ST25R500_CMD_MODE));

  digitalWrite(cs_pin, HIGH);
  dev_spi->endTransaction();

  if (isr_pending) {
    st25r500Isr();
    isr_pending = false;
  }

  return ERR_NONE;
}


/*******************************************************************************/
ReturnCode RfalRfST25R500Class::st25r500ReadTestRegister(uint8_t reg, uint8_t *val)
{
  /* Setting Transaction Parameters */
  dev_spi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE1));
  digitalWrite(cs_pin, LOW);

  dev_spi->transfer(ST25R500_CMD_TEST_ACCESS);
  dev_spi->transfer((reg | ST25R500_READ_MODE));

  dev_spi->transfer((void *)val, ST25R500_REG_LEN);

  digitalWrite(cs_pin, HIGH);
  dev_spi->endTransaction();

  if (isr_pending) {
    st25r500Isr();
    isr_pending = false;
  }

  return ERR_NONE;
}




/*******************************************************************************/
ReturnCode RfalRfST25R500Class::st25r500WriteTestRegister(uint8_t reg, uint8_t val)
{
  uint8_t value = val;               /* MISRA 17.8: use intermediate variable */
  /* Setting Transaction Parameters */
  dev_spi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE1));
  digitalWrite(cs_pin, LOW);

  dev_spi->transfer(ST25R500_CMD_TEST_ACCESS);

  dev_spi->transfer((reg | ST25R500_WRITE_MODE));

  dev_spi->transfer((void *)&value, ST25R500_REG_LEN);

  digitalWrite(cs_pin, HIGH);
  dev_spi->endTransaction();


  if (isr_pending) {
    st25r500Isr();
    isr_pending = false;
  }

  return ERR_NONE;
}


/*******************************************************************************/
ReturnCode RfalRfST25R500Class::st25r500WriteMultipleTestRegister(uint8_t reg, const uint8_t *values, uint8_t length)
{
  if (length > 0U) {

    uint8_t tx[256];

    if (values != NULL) {
      (void)memcpy(tx, values, length);
    }

    /* Setting Transaction Parameters */
    dev_spi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE1));
    digitalWrite(cs_pin, LOW);

    dev_spi->transfer(ST25R500_CMD_TEST_ACCESS);

    dev_spi->transfer((reg | ST25R500_WRITE_MODE));

    dev_spi->transfer((void *)values, length);

    digitalWrite(cs_pin, HIGH);
    dev_spi->endTransaction();

    if (isr_pending) {
      st25r500Isr();
      isr_pending = false;
    }
  }

  return ERR_NONE;
}


/*******************************************************************************/
ReturnCode RfalRfST25R500Class::st25r500ClrRegisterBits(uint8_t reg, uint8_t clr_mask)
{
  ReturnCode ret;
  uint8_t    rdVal;

  /* Read current reg value */
  EXIT_ON_ERR(ret, st25r500ReadRegister(reg, &rdVal));

  /* Only perform a Write if value to be written is different */
  if (ST25R500_OPTIMIZE && (rdVal == (uint8_t)(rdVal & ~clr_mask))) {
    return ERR_NONE;
  }

  /* Write new reg value */
  return st25r500WriteRegister(reg, (uint8_t)(rdVal & ~clr_mask));
}


/*******************************************************************************/
ReturnCode RfalRfST25R500Class::st25r500SetRegisterBits(uint8_t reg, uint8_t set_mask)
{
  ReturnCode ret;
  uint8_t    rdVal;

  /* Read current reg value */
  EXIT_ON_ERR(ret, st25r500ReadRegister(reg, &rdVal));

  /* Only perform a Write if the value to be written is different */
  if (ST25R500_OPTIMIZE && (rdVal == (rdVal | set_mask))) {
    return ERR_NONE;
  }

  /* Write new reg value */
  return st25r500WriteRegister(reg, (rdVal | set_mask));
}


/*******************************************************************************/
ReturnCode RfalRfST25R500Class::st25r500ChangeRegisterBits(uint8_t reg, uint8_t valueMask, uint8_t value)
{
  return st25r500ModifyRegister(reg, valueMask, (valueMask & value));
}


/*******************************************************************************/
ReturnCode RfalRfST25R500Class::st25r500ModifyRegister(uint8_t reg, uint8_t clr_mask, uint8_t set_mask)
{
  ReturnCode ret;
  uint8_t    rdVal;
  uint8_t    wrVal;

  /* Read current reg value */
  EXIT_ON_ERR(ret, st25r500ReadRegister(reg, &rdVal));

  /* Compute new value */
  wrVal  = (uint8_t)(rdVal & ~clr_mask);
  wrVal |= set_mask;

  /* Only perform a Write if the value to be written is different */
  if (ST25R500_OPTIMIZE && (rdVal == wrVal)) {
    return ERR_NONE;
  }

  /* Write new reg value */
  return st25r500WriteRegister(reg, wrVal);
}


/*******************************************************************************/
ReturnCode RfalRfST25R500Class::st25r500ChangeTestRegisterBits(uint8_t reg, uint8_t valueMask, uint8_t value)
{
  ReturnCode ret;
  uint8_t    rdVal;
  uint8_t    wrVal;

  /* Read current reg value */
  EXIT_ON_ERR(ret, st25r500ReadTestRegister(reg, &rdVal));

  /* Compute new value */
  wrVal  = (uint8_t)(rdVal & ~valueMask);
  wrVal |= (uint8_t)(value & valueMask);

  /* Only perform a Write if the value to be written is different */
  if (ST25R500_OPTIMIZE && (rdVal == wrVal)) {
    return ERR_NONE;
  }

  /* Write new reg value */
  return st25r500WriteTestRegister(reg, wrVal);
}


/*******************************************************************************/
bool RfalRfST25R500Class::st25r500CheckReg(uint8_t reg, uint8_t mask, uint8_t val)
{
  uint8_t regVal;

  regVal = 0;
  st25r500ReadRegister(reg, &regVal);

  return ((regVal & mask) == val);
}


/*******************************************************************************/
bool RfalRfST25R500Class::st25r500IsRegValid(uint8_t reg)
{
  if (!(((int16_t)reg >= (int16_t)ST25R500_REG_OPERATION) && (reg < ST25R500_REG_FIFO))) {
    return false;
  }
  return true;
}



