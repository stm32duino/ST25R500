/**
  ******************************************************************************
  * @file           : st25r500.h
  * @brief          : ST25R500 high level interface
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


#ifndef ST25R500_H
#define ST25R500_H

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/
#include "st_errno.h"
#include "st25r500_com.h"

/*
******************************************************************************
* GLOBAL DATATYPES
******************************************************************************
*/


/*
******************************************************************************
* GLOBAL DEFINES
******************************************************************************
*/

/* ST25R500 direct commands */
#define ST25R500_CMD_SET_DEFAULT              0x60U    /*!< Puts the chip in default state (same as after power-up) */
#define ST25R500_CMD_STOP                     0x62U    /*!< Stops all activities and clears FIFO                    */
#define ST25R500_CMD_CLEAR_FIFO               0x64U    /*!< Clears FIFO, Collision and IRQ status                   */
#define ST25R500_CMD_CLEAR_RXGAIN             0x66U    /*!< Clears FIFO, Collision and IRQ status                   */
#define ST25R500_CMD_ADJUST_REGULATORS        0x68U    /*!< Adjust regulators                                       */
#define ST25R500_CMD_TRANSMIT                 0x6AU    /*!< Transmit                                                */
#define ST25R500_CMD_TRANSMIT_EOF             0x6CU    /*!< Transmit ISO15693 EOF                                   */
#define ST25R500_CMD_NFC_FIELD_ON             0x6EU    /*!< Field On                                                */
#define ST25R500_CMD_MASK_RECEIVE_DATA        0x70U    /*!< Mask receive data                                       */
#define ST25R500_CMD_UNMASK_RECEIVE_DATA      0x72U    /*!< Unmask receive data                                     */
#define ST25R500_CMD_CALIBRATE_WU             0x74U    /*!< Calibrate Wake-up Measurement                           */
#define ST25R500_CMD_CLEAR_WU_CALIB           0x76U    /*!< Clear Wake-up Calibratation                             */
#define ST25R500_CMD_MEASURE_WU               0x78U    /*!< Measure Wake-up I and Q components                      */
#define ST25R500_CMD_MEASURE_IQ               0x7AU    /*!< Measure I and Q components                              */
#define ST25R500_CMD_SENSE_RF                 0x7CU    /*!< Sense RF on RFI pins                                    */
#define ST25R500_CMD_TRIGGER_WU_EV            0x7EU    /*!< Trigger Wake-up Event                             */
#define ST25R500_CMD_START_GP_TIMER           0xE2U    /*!< Start the general purpose timer                         */
#define ST25R500_CMD_START_WUT                0xE4U    /*!< Start the wake-up timer                                 */
#define ST25R500_CMD_START_MRT                0xE6U    /*!< Start the mask-receive timer                            */
#define ST25R500_CMD_START_NRT                0xE8U    /*!< Start the no-response timer                             */
#define ST25R500_CMD_STOP_NRT                 0xEAU    /*!< Stop No Response Timer                                  */
#define ST25R500_CMD_CALIBRATE_RC             0xEEU    /*!< Calibrate RC                                            */
#define ST25R500_CMD_TRIGGER_DIAG             0xF8U    /*!< Trigger Diagnostic Measurement                          */
#define ST25R500_CMD_TEST_ACCESS              0xFCU    /*!< Enable R/W access to the test registers                 */

#define ST25R500_BR_DO_NOT_SET                0xFFU    /*!< Indicates not to change this Bit Rate                   */
#define ST25R500_BR_106_26                    0x00U    /*!< ST25R500 Bit Rate  106 kbps (fc/128) / 26 kbps(fc/512)  */
#define ST25R500_BR_212_53                    0x01U    /*!< ST25R500 Bit Rate  212 kbps (fc/64)                     */
#define ST25R500_BR_424                       0x02U    /*!< ST25R500 Bit Rate  424 kbps (fc/32) / 53 kbps(fc/256)   */
#define ST25R500_BR_848                       0x03U    /*!< ST25R500 Bit Rate  848 kbps (fc/16)                     */

#define ST25R500_REG_DROP_200                 0U       /*!< ST25R500 target drop for regulator adjustment: 200mV    */
#define ST25R500_REG_DROP_250                 1U       /*!< ST25R500 target drop for regulator adjustment: 250mV    */
#define ST25R500_REG_DROP_300                 2U       /*!< ST25R500 target drop for regulator adjustment: 300mV    */
#define ST25R500_REG_DROP_350                 3U       /*!< ST25R500 target drop for regulator adjustment: 350mV    */
#define ST25R500_REG_DROP_400                 4U       /*!< ST25R500 target drop for regulator adjustment: 400mV    */
#define ST25R500_REG_DROP_450                 5U       /*!< ST25R500 target drop for regulator adjustment: 450mV    */
#define ST25R500_REG_DROP_500                 6U       /*!< ST25R500 target drop for regulator adjustment: 500mV    */
#define ST25R500_REG_DROP_550                 7U       /*!< ST25R500 target drop for regulator adjustment: 550mV    */
#define ST25R500_REG_DROP_DO_NOT_SET          0xFFU    /*!< Indicates not to change this setting (regd)             */

#define ST25R500_THRESHOLD_DO_NOT_SET         0xFFU    /*!< Indicates not to change this Threshold                  */

#define ST25R500_REG_LEN                      1U       /*!< Number of bytes in a ST25R500 register                  */
#define ST25R500_CMD_LEN                      1U       /*!< ST25R500 CMD length                                     */
#define ST25R500_FIFO_DEPTH                   256U     /*!< Depth of FIFO                                           */
#define ST25R500_TOUT_OSC_STABLE              5U       /*!< Timeout for Oscillator to get stable                    */

#define ST25R500_WRITE_MODE                   (0U << 7)           /*!< ST25R500 Operation Mode: Write               */
#define ST25R500_READ_MODE                    (1U << 7)           /*!< ST25R500 Operation Mode: Read                */
#define ST25R500_CMD_MODE                     ST25R500_WRITE_MODE /*!< ST25R500 Operation Mode: Direct Command      */
#define ST25R500_FIFO_ACCESS                  (0x5FU)             /*!< ST25R500 FIFO Access                         */


#define ST25R500_DIAG_MEAS_CMD                0x01U               /*!< ST25R500 Diagnostic Measurement cmd size     */
#define ST25R500_DIAG_MEAS_CMD_LEN            0x02U               /*!< ST25R500 Diagnostic Measurement cmd length   */
#define ST25R500_DIAG_MEAS_RES_LEN            0x04U               /*!< ST25R500 Diagnostic Measurement res length   */

#define ST25R500_DIAG_MEAS_I_VDD_DR           0x01U               /*!< ST25R500 Diagnostic Measurement: I_VDD_DR    */
#define ST25R500_DIAG_MEAS_VDD_TX             0x02U               /*!< ST25R500 Diagnostic Measurement: VDD_TX      */
#define ST25R500_DIAG_MEAS_VDD_DR             0x03U               /*!< ST25R500 Diagnostic Measurement: VDD_DR      */
#define ST25R500_DIAG_MEAS_VDD_IO             0x04U               /*!< ST25R500 Diagnostic Measurement: VDD_IO      */
#define ST25R500_DIAG_MEAS_VDD_D              0x0CU               /*!< ST25R500 Diagnostic Measurement: VDD_D       */
#define ST25R500_DIAG_MEAS_VDD_A              0x0DU               /*!< ST25R500 Diagnostic Measurement: VDD_A       */
#define ST25R500_DIAG_MEAS_VDD_VDD            0x11U               /*!< ST25R500 Diagnostic Measurement: VDD         */
#define ST25R500_DIAG_MEAS_VDD_AGD            0x12U               /*!< ST25R500 Diagnostic Measurement: AGD         */


/*
******************************************************************************
* GLOBAL MACROS
******************************************************************************
*/

/*! Enables the Transmitter (Field On) and Receiver */
#define st25r500TxRxOn()             st25r500SetRegisterBits( ST25R500_REG_OPERATION, (ST25R500_REG_OPERATION_rx_en | ST25R500_REG_OPERATION_tx_en ) )

/*! Disables the Transmitter (Field Off) and Receiver                                         */
#define st25r500TxRxOff()            st25r500ClrRegisterBits( ST25R500_REG_OPERATION, (ST25R500_REG_OPERATION_rx_en | ST25R500_REG_OPERATION_tx_en ) )

/*! Enables the VDD_DR regulator */
#define st25r500VDDDROn()            st25r500SetRegisterBits( ST25R500_REG_OPERATION, (ST25R500_REG_OPERATION_vdddr_en | ST25R500_REG_OPERATION_vdddr_en ) )

/*! Disables the Transmitter (Field Off) and Receiver                                         */
#define st25r500VDDDROff()           st25r500ClrRegisterBits( ST25R500_REG_OPERATION, (ST25R500_REG_OPERATION_vdddr_en | ST25R500_REG_OPERATION_vdddr_en ) )

/*! Disables the Transmitter (Field Off) */
#define st25r500TxOff()              st25r500ClrRegisterBits( ST25R500_REG_OPERATION, ST25R500_REG_OPERATION_tx_en )

/*! Checks if General Purpose Timer is still running by reading gpt_on flag */
#define st25r500IsGPTRunning( )      st25r500CheckReg( ST25R500_REG_STATUS2, ST25R500_REG_STATUS2_gpt_on, ST25R500_REG_STATUS2_gpt_on )

/*! Checks if External Filed is detected by reading ST25R3916 External Field Detector output    */
//MODI#define st25r500IsExtFieldOn()       st25r500CheckReg( ST25R500_REG_STATUS1, ST25R500_REG_STATUS1_efd_out, ST25R500_REG_STATUS1_efd_out )

/*! Checks if Transmitter is enabled (Field On) */
#define st25r500IsTxEnabled()        st25r500CheckReg( ST25R500_REG_OPERATION, ST25R500_REG_OPERATION_tx_en, ST25R500_REG_OPERATION_tx_en )

/*! Checks if NRT is in EMV mode */
#define st25r500IsNRTinEMV()         st25r500CheckReg( ST25R500_REG_NRT_GPT_CONF, ST25R500_REG_NRT_GPT_CONF_nrt_emv, ST25R500_REG_NRT_GPT_CONF_nrt_emv_on )

/*! Checks if last FIFO byte is complete */
#define st25r500IsLastFIFOComplete() st25r500CheckReg( ST25R500_REG_FIFO_STATUS2, ST25R500_REG_FIFO_STATUS2_fifo_lb_mask, 0 )

/*! Checks if the Oscillator is enabled  */
#define st25r500IsOscOn()            st25r500CheckReg( ST25R500_REG_OPERATION, ST25R500_REG_OPERATION_en, ST25R500_REG_OPERATION_en )

/*! Checks if Transmitter (Field On) is enabled */
#define st25r500IsTxOn()             st25r500CheckReg( ST25R500_REG_OPERATION, ST25R500_REG_OPERATION_tx_en, ST25R500_REG_OPERATION_tx_en )


#endif /* ST25R500_H */

/**
  * @}
  *
  * @}
  *
  * @}
  *
  * @}
  */
