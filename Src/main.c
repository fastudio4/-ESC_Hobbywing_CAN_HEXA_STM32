/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
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
#include "main.h"
#include "can.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "hw_esc_telem.h"
#include "canard.h"
#include "canard_stm32.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define DELAY250 250

#define CANARD_MEM_POOL_SIZE 4096
#define DRONECAN_NODE_ID     42
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
CAN_FilterTypeDef filter = { 0 };
CanardInstance canard;
uint8_t canard_memory_pool[CANARD_MEM_POOL_SIZE];
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
char buf_uart[20];
uint32_t T;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void sendUART(void);
void CAN_FilterAcceptAll(void);
void CAN_SendTestFrame(void);

void onTransferReceived(CanardInstance *ins, CanardRxTransfer *transfer);
bool shouldAcceptTransfer(const CanardInstance *ins,
		uint64_t *out_data_type_signature, uint16_t data_type_id,
		CanardTransferType transfer_type, uint8_t source_node_id);
void DroneCAN_Init(void);
void DroneCAN_ProcessTxQueue(void);
uint64_t micros64(void);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
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
  MX_DMA_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */
	T = HAL_GetTick();
	CAN_FilterAcceptAll();
	if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_CAN_Start(&hcan1) != HAL_OK) {
		Error_Handler();
	}

	DroneCAN_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		sendUART();
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void sendUART(void) {
	if (HAL_GetTick() - T >= DELAY250) {
		T = HAL_GetTick();
		snprintf(buf_uart, sizeof(buf_uart), "TEST\r\n");
		if (huart1.gState == HAL_UART_STATE_READY) {
			HAL_UART_Transmit_DMA(&huart1, (uint8_t*) buf_uart,
					sizeof(buf_uart));
//			CAN_SendTestFrame();
			DroneCAN_ProcessTxQueue();
		}
	}
}

void CAN_FilterAcceptAll(void) {
	filter.FilterBank = 0;
	filter.FilterMode = CAN_FILTERMODE_IDMASK;
	filter.FilterScale = CAN_FILTERSCALE_32BIT;
	filter.FilterIdHigh = 0x0000;
	filter.FilterIdLow = 0x0000;
	filter.FilterMaskIdHigh = 0x0000;
	filter.FilterMaskIdLow = 0x0000;
	filter.FilterFIFOAssignment = CAN_RX_FIFO0;
	filter.FilterActivation = ENABLE;
	filter.SlaveStartFilterBank = 14;
	if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK) {
		Error_Handler();
	}
}

void CAN_SendTestFrame(void) {
	CAN_TxHeaderTypeDef tx_header = { 0 };
	uint8_t data[8] = { 1, 2, 3, 4, 5, 6, 7, 8 };
	uint32_t mailbox = 0;
	tx_header.ExtId = 0x1234567;
	tx_header.IDE = CAN_ID_EXT;
	tx_header.RTR = CAN_RTR_DATA;
	tx_header.DLC = 8;
	tx_header.TransmitGlobalTime = DISABLE;
	if (HAL_CAN_AddTxMessage(&hcan1, &tx_header, data, &mailbox) != HAL_OK) {
		Error_Handler();
	}
}

void onTransferReceived(CanardInstance *ins, CanardRxTransfer *transfer) {
	(void) ins;
	(void) transfer;
}

bool shouldAcceptTransfer(const CanardInstance *ins,
		uint64_t *out_data_type_signature, uint16_t data_type_id,
		CanardTransferType transfer_type, uint8_t source_node_id) {
	(void) ins;
	(void) out_data_type_signature;
	(void) data_type_id;
	(void) transfer_type;
	(void) source_node_id;
	return false;
}

void DroneCAN_Init(void) {
	canardInit(&canard, canard_memory_pool, sizeof(canard_memory_pool),
			onTransferReceived, shouldAcceptTransfer,
			NULL);
	canardSetLocalNodeID(&canard, DRONECAN_NODE_ID);
}

void DroneCAN_ProcessTxQueue(void) {
	const CanardCANFrame *txf = NULL;
	while ((txf = canardPeekTxQueue(&canard)) != NULL) {
		if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {
			break;
		}
		CAN_TxHeaderTypeDef tx_header = { 0 };
		uint32_t mailbox = 0;
		tx_header.ExtId = txf->id & CANARD_CAN_EXT_ID_MASK;
		tx_header.IDE = CAN_ID_EXT;
		tx_header.RTR = CAN_RTR_DATA;
		tx_header.DLC = txf->data_len;
		tx_header.TransmitGlobalTime = DISABLE;
		if (HAL_CAN_AddTxMessage(&hcan1, &tx_header, (uint8_t*) txf->data,
				&mailbox) == HAL_OK) {
			canardPopTxQueue(&canard);
		} else {
			break;
		}
	}
}

uint64_t micros64(void) {
	return ((uint64_t) HAL_GetTick()) * 1000ULL;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	if (hcan->Instance != CAN1) {
		return;
	}
	CAN_RxHeaderTypeDef rx_header = { 0 };
	uint8_t rx_data[8] = { 0 };
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data)
			!= HAL_OK) {
		return;
	}
	if (rx_header.IDE != CAN_ID_EXT) {
		return;
	}
	CanardCANFrame frame = { 0 };
	frame.id = rx_header.ExtId;
	frame.data_len = rx_header.DLC;
	memcpy(frame.data, rx_data, rx_header.DLC);
	canardHandleRxFrame(&canard, &frame, micros64());
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
#ifdef USE_FULL_ASSERT
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
