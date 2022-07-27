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
#include <CAN_AMS.h>
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
#define MCISO_ID 1

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
AMS_HeartbeatState_t AMS_hbState;

void setup_heartbeat();
void heartbeat_timer_cb(void *args);
bool check_bad_heartbeat();
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
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

  	CAN_setup();
  	setup_heartbeat();

//	uint32_t data = 0;
//	uint8_t *d = &data;
//	uint8_t dA[4] = {d[3], d[2], d[1], d[0]};
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	//CAN_Generic_t msg;
	//CAN_TxHeaderTypeDef header;

	//bool first_inv_msg = false;

	while (1) {
		/*
		check_bad_heartbeat();

		timer_update(&timer_heartbeat, NULL);

		while (queue_next(&CAN1_Rx, &msg)) {
			int free_lvl = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);

			while (free_lvl <= 0) {
				free_lvl = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
			}

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
			header.RTR = msg.header.RTR;
			header.TransmitGlobalTime = DISABLE;
			HAL_CAN_AddTxMessage(&hcan1, &header, msg.data, &can1Mb);
		}

		while (queue_next(&CAN2_Rx, &msg)) {
			if (msg.header.ExtId == AMS_Heartbeat_ID) {
				Parse_AMS_Heartbeat(msg.data, &AMS_hbState);
			}

			// if we aren't in precharge / TS active (eg TS is not powered), don't forward anything
			// bc no inverters
			if (!((AMS_hbState.stateID == AMS_STATE_PRECHARGE)
					|| (AMS_hbState.stateID == AMS_STATE_TS_ACTIVE))) {
				continue;
			}

			int free_lvl = HAL_CAN_GetTxMailboxesFreeLevel(&hcan2);

			while (free_lvl <= 0) {
				free_lvl = HAL_CAN_GetTxMailboxesFreeLevel(&hcan2);
			}

			header.DLC = msg.header.DLC;
			header.ExtId = msg.header.ExtId;
			header.IDE = msg.header.IDE;
			header.RTR = msg.header.RTR;
			header.TransmitGlobalTime = DISABLE;
			HAL_CAN_AddTxMessage(&hcan2, &header, msg.data, &can2Mb);
		}

		//HAL_IWDG_Refresh(&hiwdg);*/

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
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
	timer_heartbeat = timer_init(50, true, heartbeat_timer_cb);

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
void Error_Handler(void)
{
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
