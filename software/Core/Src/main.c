/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <sys/unistd.h>
#include <string.h>
#include <stdlib.h>
#include <CAN_VESC.h>
#include <CAN_MCISO.h>
#include <Timer.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// hcan1 == GLV
// hcan2 == HV
// 0 - left, eg FL (0) and RL (2)
// 1 - right, eg FR (1) and RR (3)
#define MCISO_ID 0

MCISO_HeartbeatState_t MCISO_heartbeatState;

#define INVERTER_COUNT 2
#define HEARTBEAT_TIMEOUT 300U

typedef struct inverter_state {
	bool inverter[INVERTER_COUNT];

	uint32_t hb_inv_start[INVERTER_COUNT];
} inverter_state_t;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
ms_timer_t timer_heartbeat;
inverter_state_t inverters;

void setup_heartbeat();
void heartbeat_timer_cb(void *args);
bool check_bad_heartbeat();
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_CAN1_Init();
	MX_CAN2_Init();
	MX_USART3_UART_Init();
	/* USER CODE BEGIN 2 */

	queue_init(&c1Passthrough, sizeof(CAN_Generic_t));
	queue_init(&c2Passthrough, sizeof(CAN_Generic_t));

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

	if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING)
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

	setup_heartbeat();

//	uint32_t data = 0;
//	uint8_t *d = &data;
//	uint8_t dA[4] = {d[3], d[2], d[1], d[0]};
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	CAN_Generic_t msg;
	CAN_TxHeaderTypeDef header;

	while (1) {
		while (!queue_empty(&c1Passthrough)) {
			while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) <= 0)
				;
			queue_next(&c1Passthrough, &msg);

			uint8_t vesc_ID = (msg.header.ExtId & 0xFF);
			uint8_t vesc_type_ID = msg.header.ExtId >> 8;

			// check that this is actually a VESC message
			if (vesc_type_ID <= VESC_CAN_PACKET_BMS_SOC_SOH_TEMP_STAT) {
				if (vesc_ID == (MCISO_ID)) {
					inverters.hb_inv_start[0] = HAL_GetTick();
					inverters.inverter[0] = true;
					MCISO_heartbeatState.errorFlags.HB_INV0 = 0;
				} else if (vesc_ID == (MCISO_ID + 2)) {
					inverters.hb_inv_start[1] = HAL_GetTick();
					inverters.inverter[1] = true;
					MCISO_heartbeatState.errorFlags.HB_INV1 = 0;
				}
			}

			header.DLC = msg.header.DLC;
			header.ExtId = msg.header.ExtId;
			header.IDE = msg.header.IDE;
			HAL_CAN_AddTxMessage(&hcan1, &header, msg.data, &can1Mb);
		}

		while (!queue_empty(&c2Passthrough)) {
			while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) <= 0)
				;
			queue_next(&c2Passthrough, &msg);

			header.DLC = msg.header.DLC;
			header.ExtId = msg.header.ExtId;
			header.IDE = msg.header.IDE;
			HAL_CAN_AddTxMessage(&hcan2, &header, msg.data, &can2Mb);
		}

		check_bad_heartbeat();

		timer_update(&timer_heartbeat, NULL);

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
#ifdef PRINTF_TO_UART
/** Override _write to log to UART */
int _write(int file, char *data, int len) {
	if ((file != STDOUT_FILENO) && (file != STDERR_FILENO)) {
		return -1;
	}
	HAL_StatusTypeDef s = HAL_UART_Transmit(&huart3, (uint8_t*) data, len,
	HAL_MAX_DELAY);

	return (s == HAL_OK ? len : 0);
}
#endif

void handleCAN(CAN_HandleTypeDef *hcan, int fifo) {
	__disable_irq();

	// Iterate over the CAN FIFO buffer, adding all CAN messages to the CAN Queue.
	CAN_Generic_t msg;

	uint8_t vesc_ID;
	uint8_t vesc_type_ID;

	while (HAL_CAN_GetRxFifoFillLevel(hcan, fifo) > 0) {
		if (HAL_CAN_GetRxMessage(hcan, fifo, &(msg.header), msg.data)
				!= HAL_OK) {
			//printf("Failed to recieve good can message\r\n");
		}

		if (hcan == &hcan1) {
			vesc_ID = (msg.header.ExtId & 0xFF);
			vesc_type_ID = msg.header.ExtId >> 8;

			// check that this is actually a VESC message
			if (vesc_type_ID <= VESC_CAN_PACKET_BMS_SOC_SOH_TEMP_STAT && vesc_ID <= 4) {
				queue_add(&c2Passthrough, &msg);
			}
		} else if (hcan == &hcan2) {
			queue_add(&c1Passthrough, &msg);
		}
	}
	__enable_irq();
}

bool check_bad_heartbeat() {
	bool success = true;

	if ((HAL_GetTick() - inverters.hb_inv_start[0]) > HEARTBEAT_TIMEOUT) {
		success = false;
		inverters.inverter[0] = false;
		MCISO_heartbeatState.errorFlags.HB_INV0 = 1;
	}

	if ((HAL_GetTick() - inverters.hb_inv_start[1]) > HEARTBEAT_TIMEOUT) {
		success = false;
		inverters.inverter[1] = false;
		MCISO_heartbeatState.errorFlags.HB_INV1 = 1;
	}

	return success;
}

void setup_heartbeat() {
	timer_heartbeat = timer_init(100, true, heartbeat_timer_cb);

	inverters.hb_inv_start[0] = 0;
	inverters.hb_inv_start[1] = 0;

	timer_start(&timer_heartbeat);
}

void heartbeat_timer_cb(void *args) {
	MCISO_Heartbeat_t msg = Compose_MCISO_Heartbeat(MCISO_ID,
			&MCISO_heartbeatState);
	CAN_TxHeaderTypeDef header = { .ExtId = msg.id, .IDE =
	CAN_ID_EXT, .RTR = CAN_RTR_DATA, .DLC = sizeof(msg.data),
			.TransmitGlobalTime = DISABLE };

	// send heartbeat on all main CANbus
	HAL_CAN_AddTxMessage(&hcan1, &header, msg.data, &can1Mb);
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
