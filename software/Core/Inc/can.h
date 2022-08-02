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

