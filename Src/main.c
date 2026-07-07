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
#include "uavcan.equipment.esc.Status.h"
#include "uavcan.protocol.NodeStatus.h"
#include "uavcan.protocol.GetNodeInfo_res.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define DELAY1000 1000

#define CANARD_MEM_POOL_SIZE 2048

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
uint8_t node_status_transfer_id = 0;
uint8_t canard_memory_pool[CANARD_MEM_POOL_SIZE];
CanardInstance canard;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint32_t T;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void sendStatus(void);

void droneCAN_Init(void);
void sendNodeStatus(void);

bool shouldAcceptTransfer(const CanardInstance *ins,
		uint64_t *out_data_type_signature, uint16_t data_type_id,
		CanardTransferType transfer_type, uint8_t source_node_id);
void onTransferReceived(CanardInstance *ins, CanardRxTransfer *transfer);
void usleep(uint32_t us);
void sendGetNodeInfoResponse(CanardRxTransfer *transfer);
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

	if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_CAN_Start(&hcan1) != HAL_OK) {
		Error_Handler();
	}
	droneCAN_Init();

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		sendStatus();
		// ПЕРЕДАЧА: Беремо кадри з черги Canard і відправляємо в залізо STM32
		const CanardCANFrame *tx_frame = canardPeekTxQueue(&canard);
		while (tx_frame != NULL) {
			int16_t res = canardSTM32Transmit(tx_frame);
			if (res > 0) {
				canardPopTxQueue(&canard); // Видаляємо, якщо відправлено успішно
			} else {
				break; // Якщо поштові скриньки STM32 повні
			}
			tx_frame = canardPeekTxQueue(&canard);
		}
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
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Activate the Over-Drive mode
	 */
	if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
void sendStatus(void) {
	if (HAL_GetTick() - T >= DELAY1000) {
		T = HAL_GetTick();
		sendNodeStatus();
	}
}

void droneCAN_Init(void) {
	// 2. ІНІЦІАЛІЗАЦІЯ DRONECAN
	canardInit(&canard, canard_memory_pool, sizeof(canard_memory_pool),
			onTransferReceived, shouldAcceptTransfer, NULL);
	// Ініціалізація драйвера (без фільтрів - приймаємо все для тесту)
	canardSTM32Init(NULL, 0);

	// Встановлення Node ID (обов'язково!)
	canardSetLocalNodeID(&canard, 42);
}

bool shouldAcceptTransfer(const CanardInstance *ins,
		uint64_t *out_data_type_signature, uint16_t data_type_id,
		CanardTransferType transfer_type, uint8_t source_node_id) {
	// 1. Приймаємо запит на інформацію про вузол (GetNodeInfo)
	// Це критично для відображення в Mission Planner
	if (data_type_id == UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_ID) {
		*out_data_type_signature =
		UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_SIGNATURE;
		return true;
	}

	// 2. Якщо ви хочете приймати команди керування ESC (RawCommand)
	/*
	 if (data_type_id == UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_ID)
	 {
	 *out_data_type_signature = UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_SIGNATURE;
	 return true;
	 }
	 */

	return false; // Решту повідомлень ігноруємо
}

void onTransferReceived(CanardInstance *ins, CanardRxTransfer *transfer) {
	// Перевіряємо, чи це запит на GetNodeInfo
	if (transfer->data_type_id == UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_ID) {
		sendGetNodeInfoResponse(transfer); // Функція-відповідь (див. нижче)
	}

	// Обробка вхідних статусів інших ESC, якщо потрібно
	if (transfer->data_type_id == UAVCAN_EQUIPMENT_ESC_STATUS_ID) {
		struct uavcan_equipment_esc_Status esc_msg;
		// Використовуємо згенеровану функцію декодування [2]
		if (_uavcan_equipment_esc_Status_decode(transfer, &esc_msg) == 0) {
			// Логіка обробки отриманих даних ESC
		}
	}
}

void sendNodeStatus(void) {
	struct uavcan_protocol_NodeStatus msg;
	// 1. Заповнюємо поля структури
	msg.uptime_sec = HAL_GetTick() / 1000; 		// Час роботи вузла в секундах
	msg.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK; // Стан (OK, WARNING, ERROR, CRITICAL)
	msg.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL; // Режим роботи
	msg.sub_mode = 0;
	msg.vendor_specific_status_code = 0;

	// 2. Кодуємо повідомлення у буфер
	uint8_t buffer[UAVCAN_PROTOCOL_NODESTATUS_MAX_SIZE];
	uint16_t len = uavcan_protocol_NodeStatus_encode(&msg, buffer);

	// 3. Публікуємо повідомлення (Broadcast)
	// ID повідомлення NodeStatus — 341
	canardBroadcast(&canard,
	UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE,
	UAVCAN_PROTOCOL_NODESTATUS_ID, &node_status_transfer_id,
	CANARD_TRANSFER_PRIORITY_LOW, buffer, len);

}

void usleep(uint32_t us) {
	/* Для частоти 180 МГц один мікросекундний цикл з NOP
	 займає приблизно 30-45 ітерацій залежно від оптимізації */
	uint32_t count = us * 30;
	while (count--) {
		__asm__ volatile ("nop");
	}
}

void sendGetNodeInfoResponse(CanardRxTransfer *transfer) {
	struct uavcan_protocol_GetNodeInfoResponse resp;
	memset(&resp, 0, sizeof(resp));

	// Налаштування інформації про ваш девайс
	resp.status.uptime_sec = HAL_GetTick() / 1000;
	resp.status.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
	resp.status.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;

	resp.software_version.major = 1;
	resp.software_version.minor = 0;
	resp.hardware_version.major = 1;

	// Назва пристрою, яку ви побачите в Mission Planner
	const char *node_name = "fastudio4.esc_hexa";
	memcpy(resp.name.data, node_name, strlen(node_name));
	resp.name.len = strlen(node_name);

	uint8_t buffer[UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_MAX_SIZE];
	uint16_t len = uavcan_protocol_GetNodeInfoResponse_encode(&resp, buffer);

	// Відправляємо відповідь саме тому вузлу, який запитав (source_node_id)
	canardRequestOrRespond(&canard, transfer->source_node_id,
	UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_SIGNATURE,
	UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_ID, &transfer->transfer_id,
			transfer->priority, CanardResponse, buffer, len);
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
