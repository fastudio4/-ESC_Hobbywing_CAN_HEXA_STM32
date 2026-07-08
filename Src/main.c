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
//#include "hw_esc_telem.h"

#include "canard.h"
#include "canard_stm32.h"
//#include "uavcan.equipment.esc.Status.h"
#include "uavcan.protocol.NodeStatus.h"
#include "uavcan.protocol.GetNodeInfo_res.h"
#include "uavcan.protocol.GetNodeInfo_req.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define DELAY1000 1000

#define DRONECAN_NODE_ID                 125U
#define CANARD_MEMORY_POOL_SIZE          4096U
#define NODE_STATUS_PERIOD_MS            1000U

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
CanardInstance canard;
uint8_t memory_pool_canard[CANARD_MEMORY_POOL_SIZE];
uint8_t node_status_transfer_id = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void blink(void);
uint64_t micros64(void);
void canFilter(void);
void canard_Init(void);

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
bool shouldAcceptTransfer(const CanardInstance *ins,
		uint64_t *out_data_type_signature, uint16_t data_type_id,
		CanardTransferType transfer_type, uint8_t source_node_id);
void onTransferReceived(CanardInstance *ins, CanardRxTransfer *transfer);

void send_heartbeat(void);
void transferCanard(void);
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
	canard_Init();
	canFilter();
	if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_CAN_Start(&hcan1) != HAL_OK) {
		Error_Handler();
	}

	uint32_t last_heartbeat_ms = HAL_GetTick();
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		// Відправляємо Heartbeat кожну секунду
		if (HAL_GetTick() - last_heartbeat_ms >= 1000) {
			last_heartbeat_ms = HAL_GetTick();

			send_heartbeat();

		}

		transferCanard();

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

void blink(void) {
	GPIOB->BSRR = (GPIOB->ODR & LED_Pin) ? (LED_Pin << 16) : LED_Pin;
}
uint64_t micros64(void) {
	return (uint64_t) HAL_GetTick() * NODE_STATUS_PERIOD_MS;
}

void canFilter(void) {
	CAN_FilterTypeDef canFilterConfig;
	canFilterConfig.FilterBank = 0;                // Номер банку фільтра
	canFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK; // Режим маски
	canFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; // 32-бітний фільтр
	canFilterConfig.FilterIdHigh = 0x0000;        // ID = 0
	canFilterConfig.FilterIdLow = 0x0000;
	canFilterConfig.FilterMaskIdHigh = 0x0000;    // Маска = 0 → пропускає все
	canFilterConfig.FilterMaskIdLow = 0x0000;
	canFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0; // В який FIFO
	canFilterConfig.FilterActivation = ENABLE;    // Активувати
	canFilterConfig.SlaveStartFilterBank = 14;    // Для двох CAN (F4/F7)

	HAL_CAN_ConfigFilter(&hcan1, &canFilterConfig);
}
void canard_Init(void) {
	canardInit(&canard, memory_pool_canard, CANARD_MEMORY_POOL_SIZE,
			onTransferReceived, shouldAcceptTransfer, NULL);
	canardSetLocalNodeID(&canard, DRONECAN_NODE_ID);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	blink();
	CAN_RxHeaderTypeDef rx_header;
	uint8_t data[8];
	// Вичитуємо всі повідомлення, поки FIFO не стане порожнім
	while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
		if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, data)
				== HAL_OK) {
			CanardCANFrame frame;
			frame.id = rx_header.ExtId | CANARD_CAN_FRAME_EFF;
			frame.data_len = rx_header.DLC;
			memcpy(frame.data, data, 8);
			// Передаємо кадр у libcanard
			canardHandleRxFrame(&canard, &frame, micros64());
		}
	}
}

bool shouldAcceptTransfer(const CanardInstance *ins,
		uint64_t *out_data_type_signature, uint16_t data_type_id,
		CanardTransferType transfer_type, uint8_t source_node_id) {
	(void) ins;
	(void) source_node_id; // Ігноруємо невикористані параметри

	// Наприклад, ми хочемо приймати повідомлення "NodeStatus" від інших вузлів
	// ID типів даних зазвичай беруться з ваших згенерованих DSDL заголовків
	if (data_type_id == UAVCAN_PROTOCOL_NODESTATUS_ID) {
		// Також потрібно вказати сигнатуру типу (вона є у згенерованих DSDL файлах)
		*out_data_type_signature = UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE;
		return true;
	}
	if (data_type_id == UAVCAN_PROTOCOL_GETNODEINFO_REQUEST_ID) {
		*out_data_type_signature =
				UAVCAN_PROTOCOL_GETNODEINFO_REQUEST_SIGNATURE;
		return true;
	}

	return false; // Все інше ігноруємо
}

void onTransferReceived(CanardInstance *ins, CanardRxTransfer *transfer) {
	// 1. Обробка повідомлення NodeStatus (Heartbeat від інших вузлів)
	if (transfer->data_type_id == UAVCAN_PROTOCOL_NODESTATUS_ID) {
		struct uavcan_protocol_NodeStatus msg;
		if (uavcan_protocol_NodeStatus_decode(transfer, &msg) >= 0) {
			// Тут можна обробляти статус інших вузлів
		}
	}
	// 2. Обробка запиту GetNodeInfo (Сервісний запит від польотного контролера)
	if (transfer->data_type_id == UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_ID) {
		struct uavcan_protocol_GetNodeInfoResponse res;
		memset(&res, 0, sizeof(res));

		// Заповнюємо дані про статус (як у вашому Heartbeat)
		res.status.uptime_sec = HAL_GetTick() / 1000;
		res.status.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
		res.status.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;

		// Версії ПЗ та апаратного забезпечення
		res.software_version.major = 1;
		res.software_version.minor = 0;
		res.hardware_version.major = 1;
		res.hardware_version.minor = 0;

		// Ім'я вашого пристрою (максимум 80 символів)
		const char *name = "dronecan";
		res.name.len = strlen(name);
		memcpy(res.name.data, name, res.name.len);

		// Кодуємо відповідь у буфер
		uint8_t buffer[UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_MAX_SIZE];
		uint16_t total_size = uavcan_protocol_GetNodeInfoResponse_encode(&res,
				buffer);

		// ВІДПРАВЛЯЄМО ВІДПОВІДЬ
		// Важливо: використовуємо transfer_id та source_node_id з отриманого запиту!
		canardRequestOrRespond(ins, transfer->source_node_id, // 2. Кому (ID запитувача)
				UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_SIGNATURE,  // 3. Сигнатура
				UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_ID,        // 4. ID сервісу
				&transfer->transfer_id,             // 5. АДРЕСА (&) transfer_id
				transfer->priority,                     		// 6. Пріоритет
				CanardTransferTypeResponse,             // 7. Тип (відповідь)
				buffer,                                 			// 8. Дані
				total_size);                            		// 9. Довжина
	}

}

void send_heartbeat(void) {
	struct uavcan_protocol_NodeStatus msg;
	uint8_t buffer[UAVCAN_PROTOCOL_NODESTATUS_MAX_SIZE];
// 1. Заповнюємо структуру повідомлення даними
	msg.uptime_sec = HAL_GetTick() / NODE_STATUS_PERIOD_MS;
	msg.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
	msg.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;
	msg.sub_mode = 0;
	msg.vendor_specific_status_code = 0;
// 2. Кодуємо структуру в масив байтів (використовуємо згенеровану функцію)
	uint32_t len = uavcan_protocol_NodeStatus_encode(&msg, buffer);
// 3. Передаємо дані в чергу libcanard для трансляції (Broadcast)
// Використовуємо ID та Signature, які визначені у згенерованому заголовку
	int16_t res = canardBroadcast(&canard,
	UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE,
	UAVCAN_PROTOCOL_NODESTATUS_ID, &node_status_transfer_id,
	CANARD_TRANSFER_PRIORITY_LOWEST, buffer, len);
}

void transferCanard(void) {
	const CanardCANFrame *tx_frame = canardPeekTxQueue(&canard);
	while (tx_frame != NULL) {
		CAN_TxHeaderTypeDef tx_header;
		uint32_t tx_mailbox;

		tx_header.ExtId = tx_frame->id & CANARD_CAN_EXT_ID_MASK;
		tx_header.IDE = CAN_ID_EXT;
		tx_header.RTR = CAN_RTR_DATA;
		tx_header.DLC = tx_frame->data_len;

		if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0) {
			if (HAL_CAN_AddTxMessage(&hcan1, &tx_header,
					(uint8_t*) tx_frame->data, &tx_mailbox) == HAL_OK) {
				canardPopTxQueue(&canard);
				tx_frame = canardPeekTxQueue(&canard);
				continue;
			}
		}
		break;
	}
	canardCleanupStaleTransfers(&canard, micros64());
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
