/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f2xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <queue.h>
#include <QUTMS_can.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
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

typedef struct CAN_Generic {
	CAN_RxHeaderTypeDef header;
	uint8_t data[8];
	void *hcan;
} CAN_Generic_t;

message_queue_t c1Passthrough;
message_queue_t c2Passthrough;

uint32_t can1Mb;
uint32_t can2Mb;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define PRINTF_TO_UART
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void handleCAN(CAN_HandleTypeDef *hcan, int fifo);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CAN2_HV_RX_Pin GPIO_PIN_5
#define CAN2_HV_RX_GPIO_Port GPIOB
#define CAN2_HV_TX_Pin GPIO_PIN_6
#define CAN2_HV_TX_GPIO_Port GPIOB
#define CAN1_GLV_RX_Pin GPIO_PIN_8
#define CAN1_GLV_RX_GPIO_Port GPIOB
#define CAN1_GLV_TX_Pin GPIO_PIN_9
#define CAN1_GLV_TX_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
