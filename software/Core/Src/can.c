/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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
/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
extern message_queue_t CAN1_Tx_Queue, CAN2_Tx_Queue;

/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 2;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}
/* CAN2 init function */
void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 2;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

}

static uint32_t HAL_RCC_CAN1_CLK_ENABLED=0;

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PB8     ------> CAN1_RX
    PB9     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = CAN1_GLV_RX_Pin|CAN1_GLV_TX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_TX_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
  else if(canHandle->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspInit 0 */

  /* USER CODE END CAN2_MspInit 0 */
    /* CAN2 clock enable */
    __HAL_RCC_CAN2_CLK_ENABLE();
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN2 GPIO Configuration
    PB5     ------> CAN2_RX
    PB6     ------> CAN2_TX
    */
    GPIO_InitStruct.Pin = CAN2_HV_RX_Pin|CAN2_HV_TX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* CAN2 interrupt Init */
    HAL_NVIC_SetPriority(CAN2_TX_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN2_TX_IRQn);
    HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN2_RX1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN2_RX1_IRQn);
  /* USER CODE BEGIN CAN2_MspInit 1 */

  /* USER CODE END CAN2_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }

    /**CAN1 GPIO Configuration
    PB8     ------> CAN1_RX
    PB9     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOB, CAN1_GLV_RX_Pin|CAN1_GLV_TX_Pin);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
  else if(canHandle->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspDeInit 0 */

  /* USER CODE END CAN2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN2_CLK_DISABLE();
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }

    /**CAN2 GPIO Configuration
    PB5     ------> CAN2_RX
    PB6     ------> CAN2_TX
    */
    HAL_GPIO_DeInit(GPIOB, CAN2_HV_RX_Pin|CAN2_HV_TX_Pin);

    /* CAN2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN2_TX_IRQn);
    HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
    HAL_NVIC_DisableIRQ(CAN2_RX1_IRQn);
  /* USER CODE BEGIN CAN2_MspDeInit 1 */

  /* USER CODE END CAN2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

void CAN_setup( void )
{
	queue_init(&CAN1_Passthrough, sizeof(CAN_Generic_t));
	queue_init(&CAN2_Passthrough, sizeof(CAN_Generic_t));
	queue_init(&CAN1_Tx_Queue, sizeof(CAN_Generic_t));
	queue_init(&CAN2_Tx_Queue, sizeof(CAN_Generic_t));

	if (HAL_CAN_Start(&hcan1) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_CAN_Start(&hcan2) != HAL_OK) {
		Error_Handler();
	}

	CAN_FilterTypeDef GLVFilterConfig;

	GLVFilterConfig.FilterBank = 0;
	GLVFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	GLVFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	GLVFilterConfig.FilterIdHigh = 0x0000;
	GLVFilterConfig.FilterIdLow = 0x0000;
	GLVFilterConfig.FilterMaskIdHigh = 0x0000;
	GLVFilterConfig.FilterMaskIdLow = 0x0000;
	GLVFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	GLVFilterConfig.FilterActivation = ENABLE;
	GLVFilterConfig.SlaveStartFilterBank = 14;

	if (HAL_CAN_ConfigFilter(&hcan1, &GLVFilterConfig) != HAL_OK) {
		Error_Handler();
	}

	CAN_FilterTypeDef HVFilterConfig;

	HVFilterConfig.FilterBank = 14;
	HVFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	HVFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	HVFilterConfig.FilterIdHigh = 0x0000;
	HVFilterConfig.FilterIdLow = 0x0000;
	HVFilterConfig.FilterMaskIdHigh = 0x0000;
	HVFilterConfig.FilterMaskIdLow = 0x0000;
	HVFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	HVFilterConfig.FilterActivation = ENABLE;
	HVFilterConfig.SlaveStartFilterBank = 14;
	if (HAL_CAN_ConfigFilter(&hcan2, &HVFilterConfig) != HAL_OK) {
		Error_Handler();
	}


	// Activating CAN Interrupts
	if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY)
				!= HAL_OK) {
			Error_Handler();
	}

	if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_TX_MAILBOX_EMPTY)
				!= HAL_OK) {
			Error_Handler();
	}

}


void CAN_Rx_Interrupt_Handle(CAN_HandleTypeDef *hcan, int fifo)
{
	__disable_irq();

	// Iterate over the CAN FIFO buffer, adding all CAN messages to the CAN Queue.
	CAN_Generic_t msg;

	//uint8_t vesc_ID;
	//uint8_t vesc_type_ID;

	while (HAL_CAN_GetRxFifoFillLevel(hcan, fifo) > 0) {
		if (HAL_CAN_GetRxMessage(hcan, fifo, &(msg.header), msg.data)
				!= HAL_OK) {
			//printf("Failed to recieve good can message\r\n");
		}

		if (hcan == &hcan1) { // passing
			//vesc_ID = (msg.header.ExtId & 0xFF);
			//vesc_type_ID = msg.header.ExtId >> 8;

			// check that this is actually a VESC message
			//if (vesc_type_ID <= VESC_CAN_PACKET_BMS_SOC_SOH_TEMP_STAT && vesc_ID <= 4) {
			queue_add(&CAN2_Passthrough, &msg);
			//}
		} else if (hcan == &hcan2) {
			queue_add(&CAN1_Passthrough, &msg);
		}
	}
	__enable_irq();
}

/**
 * CAN Tx interrupt deals with more CAN tx data than Mailboxes available
 */

void CAN1_Tx_Interrupt_Handle(CAN_HandleTypeDef *hcan)
{
	__disable_irq();

	CAN_Generic_t msg;
	uint32_t TxMailbox;
	CAN_TxHeaderTypeDef Header;

	if(!queue_empty(&CAN1_Tx_Queue)) // transmission queue is not empty
	{
		queue_next(&CAN1_Tx_Queue, (void *)&msg); // grab first element in queue
		// setting up the header for CAN
		Header.DLC = msg.header.DLC;
		Header.RTR = CAN_RTR_DATA;
		Header.IDE = msg.header.IDE == 1 ? CAN_ID_EXT : CAN_ID_STD;
		Header.StdId = msg.header.StdId;
		Header.ExtId = msg.header.ExtId;
		// Adding message to mailbox
		HAL_CAN_AddTxMessage((CAN_HandleTypeDef *)msg.hcan, &Header, msg.data, &TxMailbox);
	}

	__enable_irq();

}


void CAN2_Tx_Interrupt_Handle(CAN_HandleTypeDef *hcan)
{
	__disable_irq();

	CAN_Generic_t msg;
	uint32_t TxMailbox;
	CAN_TxHeaderTypeDef Header;

	if(!queue_empty(&CAN2_Tx_Queue)) // transmission queue is not empty
	{
		queue_next(&CAN2_Tx_Queue, (void *)&msg); // grab first element in queue
		// setting up the header for CAN
		Header.DLC = msg.header.DLC;
		Header.RTR = CAN_RTR_DATA;
		Header.IDE = msg.header.IDE == 1 ? CAN_ID_EXT : CAN_ID_STD;
		Header.StdId = msg.header.StdId;
		Header.ExtId = msg.header.ExtId;
		// Adding message to mailbox
		HAL_CAN_AddTxMessage((CAN_HandleTypeDef *)msg.hcan, &Header, msg.data, &TxMailbox);
	}

	__enable_irq();

}


/* USER CODE END 1 */
