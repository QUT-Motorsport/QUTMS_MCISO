/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "main.h"
#include "queue.h"
#include "stm32f2xx_hal.h"
/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;

extern CAN_HandleTypeDef hcan2;

/* USER CODE BEGIN Private defines */
// can error handling enum
/*
enum
{
	CAN_OK = 0,
	CAN_COMS_ERROR,
	CAN_MSG_ERROR,
	CAN_HANDLE_ERROR,
	CAN_TX_ERROR,
	CAN_RX_ERROR
}CAN_ERROR_e;*/

typedef struct CAN_Generic {
	CAN_RxHeaderTypeDef header;
	uint8_t data[8];
	void *hcan;
} CAN_Generic_t;

typedef enum {
    CAN_PACKET_SET_DUTY = 0,
    CAN_PACKET_SET_CURRENT, //ME
    CAN_PACKET_SET_CURRENT_BRAKE, //ME
    CAN_PACKET_SET_RPM,
    CAN_PACKET_SET_POS,
    CAN_PACKET_FILL_RX_BUFFER,
    CAN_PACKET_FILL_RX_BUFFER_LONG,
    CAN_PACKET_PROCESS_RX_BUFFER,
    CAN_PACKET_PROCESS_SHORT_BUFFER,
    CAN_PACKET_STATUS, //ME
    CAN_PACKET_SET_CURRENT_REL,
    CAN_PACKET_SET_CURRENT_BRAKE_REL,
    CAN_PACKET_SET_CURRENT_HANDBRAKE,
    CAN_PACKET_SET_CURRENT_HANDBRAKE_REL,
    CAN_PACKET_STATUS_2, //ME
    CAN_PACKET_STATUS_3, //ME
    CAN_PACKET_STATUS_4, //ME
    CAN_PACKET_PING, //ME
    CAN_PACKET_PONG, //ME
    CAN_PACKET_DETECT_APPLY_ALL_FOC, //ME ?????
    CAN_PACKET_DETECT_APPLY_ALL_FOC_RES,
    CAN_PACKET_CONF_CURRENT_LIMITS, //ME
    CAN_PACKET_CONF_STORE_CURRENT_LIMITS, //ME
    CAN_PACKET_CONF_CURRENT_LIMITS_IN, //ME
    CAN_PACKET_CONF_STORE_CURRENT_LIMITS_IN, //ME
    CAN_PACKET_CONF_FOC_ERPMS, //ME ???
    CAN_PACKET_CONF_STORE_FOC_ERPMS, //ME
    CAN_PACKET_STATUS_5, //ME
    CAN_PACKET_POLL_TS5700N8501_STATUS,
    CAN_PACKET_CONF_BATTERY_CUT,
    CAN_PACKET_CONF_STORE_BATTERY_CUT,
    CAN_PACKET_SHUTDOWN, //ME
    CAN_PACKET_IO_BOARD_ADC_1_TO_4,
    CAN_PACKET_IO_BOARD_ADC_5_TO_8,
    CAN_PACKET_IO_BOARD_ADC_9_TO_12,
    CAN_PACKET_IO_BOARD_DIGITAL_IN,
    CAN_PACKET_IO_BOARD_SET_OUTPUT_DIGITAL,
    CAN_PACKET_IO_BOARD_SET_OUTPUT_PWM,
    CAN_PACKET_BMS_V_TOT,
    CAN_PACKET_BMS_I,
    CAN_PACKET_BMS_AH_WH,
    CAN_PACKET_BMS_V_CELL,
    CAN_PACKET_BMS_BAL,
    CAN_PACKET_BMS_TEMPS,
    CAN_PACKET_BMS_HUM,
    CAN_PACKET_BMS_SOC_SOH_TEMP_STAT
} CAN_PACKET_ID;

extern uint32_t can1Mb;
extern uint32_t can2Mb;

extern message_queue_t CAN1_Passthrough;
extern message_queue_t CAN2_Passthrough;
/* USER CODE END Private defines */

void MX_CAN1_Init(void);
void MX_CAN2_Init(void);

/* USER CODE BEGIN Prototypes */
void CAN_setup( void );
void CAN_Rx_Interrupt_Handle(CAN_HandleTypeDef *hcan, int fifo);
void CAN1_Tx_Interrupt_Handle(CAN_HandleTypeDef *hcan);
void CAN2_Tx_Interrupt_Handle(CAN_HandleTypeDef *hcan);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

