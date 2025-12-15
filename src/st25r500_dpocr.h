/**
  ******************************************************************************
  * @file           : st25r500_dpocr.h
  * @brief          : Dynamic Power Output adjustment via Current Regulation
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

#ifndef ST25R500_DPOCR_H
#define ST25R500_DPOCR_H

/*
 ******************************************************************************
 * INCLUDES
 ******************************************************************************
 */
#include "nfc_utils.h"
#include "Arduino.h"

/*
 ******************************************************************************
 * GLOBAL DEFINES
 ******************************************************************************
 */

#define ST25R500_DPOCR_MAX_ENTRIES   4U     /*!< Max number of table entries */

/*
******************************************************************************
* GLOBAL TYPES
******************************************************************************
*/

/*! DPO table entry struct, to have link into AC, similar to rfal_dpo levels */

typedef struct {
  bool     enabled;                                       /*!< DPO Enabled state                                                                                    */
  uint16_t target;                                        /*!< Target value - electric current as per I_VDD_DR measurement                                          */
  uint8_t  maxRege;                                       /*!< Max regulator which will be selected                                                                 */
  uint8_t  minRege;                                       /*!< Min regulator which will be selected                                                                 */
  uint8_t  maxDres;                                       /*!< Max dres which will be selected                                                                      */
  uint8_t  minDres;                                       /*!< Min dres which will be selected                                                                      */
  uint8_t  regeStepSize;                                  /*!< Step size for rege, 1 gives best accuracy                                                            */
  uint16_t currThreshold;                                 /*!< If current value is <(target-curr_threshold) or >(target+curr_thresholds) regulation will again happen */
  uint16_t tableUpThresholds[ST25R500_DPOCR_MAX_ENTRIES]; /*!< Table with rege levels for AC                                                                        */
  uint8_t  numEntries;                                    /*!< Number of entries to be used in table                                                                */
} st25r500DpocrConfig;


/*! DPO information struct */
typedef struct {
  uint8_t            currentEntryIdx;                     /*!< Current DPO table Index                 */
  uint16_t           currentElecCurrent;                  /*!< Current DPO measured electrical current */
  uint8_t            currentRege;                         /*!< Current ST25R500 rege setting           */
  uint8_t            currentDres;                         /*!< Current ST25R500 dres setting           */
} st25r500DpocrInfo;



#endif /* ST25R500_DPOCR_H */

/**
  * @}
  *
  * @}
  *
  * @}
  *
  * @}
  */
