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
#include "uavcan.equipment.esc.Status.h"
#include "uavcan.protocol.NodeStatus.h"
#include "uavcan.protocol.GetNodeInfo_res.h"
#include "uavcan.protocol.GetNodeInfo_req.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define DRONECAN_NODE_ID                         12U
#define UAVCAN_PROTOCOL_GETNODEINFO_ID           1U
#define CANARD_MEMORY_POOL_SIZE                  4096U
#define NODE_STATUS_PERIOD_MS                    1000U
#define CANARD_CLEANUP_PERIOD_MS                 1000U
#define ESC_STATUS_PERIOD_MS 					 100U
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static CanardInstance canard;
static const CanardSTM32CANTimings can_timings = { .bit_rate_prescaler = 3,
		.bit_segment_1 = 11, .bit_segment_2 = 3,
		.max_resynchronization_jump_width = 1 };
static uint8_t canard_memory_pool[CANARD_MEMORY_POOL_SIZE];
static uint8_t node_status_transfer_id = 0;
static uint32_t last_node_status_ms = 0;
static uint32_t last_cleanup_ms = 0;

static uint8_t esc_status_transfer_id = 0;
static uint32_t last_esc_status_ms = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void blink(void);
static uint64_t micros64(void);
void canFilter(void);
static void dronecanInit(void);
static void DroneCAN_ProcessRx(void);
static void DroneCAN_ProcessTx(void);
static void onTransferReceived(CanardInstance *ins, CanardRxTransfer *transfer);
static bool shouldAcceptTransfer(const CanardInstance *ins,
		uint64_t *out_data_type_signature, uint16_t data_type_id,
		CanardTransferType transfer_type, uint8_t source_node_id);
static void sendNodeStatus(void);
static void handleGetNodeInfoRequest(const CanardRxTransfer *transfer);
static void sendEscStatusTest(void);
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
//	canFilter();
//	if (HAL_CAN_Start(&hcan1) != HAL_OK) {
//		Error_Handler();
//	}
//	HAL_Delay(1);
	dronecanInit();
	last_node_status_ms = HAL_GetTick();
	last_cleanup_ms = HAL_GetTick();
	last_esc_status_ms = HAL_GetTick();
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		DroneCAN_ProcessRx();
		DroneCAN_ProcessTx();
		const uint32_t now_ms = HAL_GetTick();
		if ((now_ms - last_node_status_ms) >= NODE_STATUS_PERIOD_MS) {
			last_node_status_ms = now_ms;
			sendNodeStatus();
		}
		if ((now_ms - last_esc_status_ms) >= ESC_STATUS_PERIOD_MS) {
			last_esc_status_ms = now_ms;
			sendEscStatusTest();
		}
		DroneCAN_ProcessTx();
		if ((now_ms - last_cleanup_ms) >= CANARD_CLEANUP_PERIOD_MS) {
			last_cleanup_ms = now_ms;
			canardCleanupStaleTransfers(&canard, micros64());
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

static void dronecanInit(void) {
	canardInit(&canard, canard_memory_pool, sizeof(canard_memory_pool),
			onTransferReceived, shouldAcceptTransfer,
			NULL);
	__HAL_RCC_CAN1_CLK_ENABLE();
	canardSTM32Init(&can_timings, CanardSTM32IfaceModeNormal);
	canardSetLocalNodeID(&canard, DRONECAN_NODE_ID);
	CanardSTM32AcceptanceFilterConfiguration filters[CANARD_STM32_NUM_ACCEPTANCE_FILTERS];
	for (uint8_t i = 0; i < CANARD_STM32_NUM_ACCEPTANCE_FILTERS; i++) {
		filters[i].id = 0;
		filters[i].mask = 0;
	}
	canardSTM32ConfigureAcceptanceFilters(filters,
	CANARD_STM32_NUM_ACCEPTANCE_FILTERS);
}

static void DroneCAN_ProcessTx(void) {
	for (;;) {
		const CanardCANFrame *tx_frame = canardPeekTxQueue(&canard);
		if (tx_frame == NULL) {
			break;
		}
		/*
		 * Офіційний драйвер:
		 * result > 0 : кадр прийнято CAN контролером
		 * result = 0 : mailbox/FIFO зайнятий, спробувати пізніше
		 * result < 0 : помилка
		 */
		const int16_t tx_result = canardSTM32Transmit(tx_frame);
		if (tx_result < 0) {
			break;
		}
		if (tx_result == 0) {
			break;
		}
		/*
		 * Pop тільки після успішного canardSTM32Transmit().
		 */
		canardPopTxQueue(&canard);
	}
}

static void DroneCAN_ProcessRx(void) {
	for (;;) {
		CanardCANFrame rx_frame;
		memset(&rx_frame, 0, sizeof(rx_frame));
		/*
		 * Офіційний драйвер:
		 * result > 0 : кадр прийнято
		 * result = 0 : кадрів немає
		 * result < 0 : помилка
		 */
		const int16_t rx_result = canardSTM32Receive(&rx_frame);
		if (rx_result < 0) {
			break;
		}
		if (rx_result == 0) {
			break;
		}
		const int16_t handle_result = canardHandleRxFrame(&canard, &rx_frame,
				micros64());
		if (handle_result < 0) {
			break;
		}
	}
}

static bool shouldAcceptTransfer(const CanardInstance *ins,
		uint64_t *out_data_type_signature, uint16_t data_type_id,
		CanardTransferType transfer_type, uint8_t source_node_id) {
	(void) ins;
	(void) source_node_id;
	if ((transfer_type == CanardTransferTypeRequest)
			&& (data_type_id == UAVCAN_PROTOCOL_GETNODEINFO_ID)) {
		*out_data_type_signature =
		UAVCAN_PROTOCOL_GETNODEINFO_REQUEST_SIGNATURE;
		return true;
	}
	if ((transfer_type == CanardTransferTypeBroadcast)
			&& (data_type_id == UAVCAN_PROTOCOL_NODESTATUS_ID)) {
		*out_data_type_signature = UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE;
		return true;
	}
	return false;
}

static void onTransferReceived(CanardInstance *ins, CanardRxTransfer *transfer) {
	(void) ins;
	if ((transfer->transfer_type == CanardTransferTypeBroadcast)
			&& (transfer->data_type_id == UAVCAN_PROTOCOL_NODESTATUS_ID)) {
		struct uavcan_protocol_NodeStatus msg;
		memset(&msg, 0, sizeof(msg));
		if (uavcan_protocol_NodeStatus_decode(transfer, &msg) >= 0) {
			__NOP();
		}
		return;
	}
	/*
	 * У класичному libcanard v0 немає transfer->destination_node_id.
	 * Service request-и не для нашого node id зазвичай відсіюються всередині libcanard,
	 * коли встановлений local node id.
	 */
	if ((transfer->transfer_type == CanardTransferTypeRequest)
			&& (transfer->data_type_id == UAVCAN_PROTOCOL_GETNODEINFO_ID)) {
		handleGetNodeInfoRequest(transfer);
		return;
	}
}

static void sendNodeStatus(void) {
	struct uavcan_protocol_NodeStatus msg;
	memset(&msg, 0, sizeof(msg));
	msg.uptime_sec = HAL_GetTick() / 1000U;
	msg.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
	msg.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;
	msg.sub_mode = 0;
	msg.vendor_specific_status_code = 0;
	uint8_t buffer[UAVCAN_PROTOCOL_NODESTATUS_MAX_SIZE];
	memset(buffer, 0, sizeof(buffer));
	const uint32_t total_size = uavcan_protocol_NodeStatus_encode(&msg, buffer);
	const int16_t result = canardBroadcast(&canard,
	UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE,
	UAVCAN_PROTOCOL_NODESTATUS_ID, &node_status_transfer_id,
	CANARD_TRANSFER_PRIORITY_LOW, buffer, total_size);
	if (result < 0) {
		__NOP();
	}
}

static void handleGetNodeInfoRequest(const CanardRxTransfer *transfer) {
	static uint32_t last_request_ms = 0;
	static uint8_t last_source_node_id = 255U;
	static uint8_t last_transfer_id = 255U;
	const uint32_t now_ms = HAL_GetTick();
	if ((last_source_node_id == transfer->source_node_id)
			&& (last_transfer_id == transfer->transfer_id)
			&& ((now_ms - last_request_ms) < 500U)) {
		return;
	}
	last_source_node_id = transfer->source_node_id;
	last_transfer_id = transfer->transfer_id;
	last_request_ms = now_ms;
	struct uavcan_protocol_GetNodeInfoResponse res;
	memset(&res, 0, sizeof(res));
	res.status.uptime_sec = HAL_GetTick() / 1000U;
	res.status.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
	res.status.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;
	res.status.sub_mode = 0;
	res.status.vendor_specific_status_code = 0;
	res.software_version.major = 1;
	res.software_version.minor = 0;
	res.software_version.optional_field_flags = 0;
	res.software_version.vcs_commit = 0;
	res.software_version.image_crc = 0;
	res.hardware_version.major = 1;
	res.hardware_version.minor = 0;
	memset(res.hardware_version.unique_id, 0,
			sizeof(res.hardware_version.unique_id));

	const uint32_t *uid = (const uint32_t*) 0x1FFF7A10U;
	memcpy(&res.hardware_version.unique_id[0], &uid[0], 4);
	memcpy(&res.hardware_version.unique_id[4], &uid[1], 4);
	memcpy(&res.hardware_version.unique_id[8], &uid[2], 4);
	/*
	 * Для тесту коротке ім'я.
	 */
	const char *node_name = "org.Node";
	size_t node_name_len = strlen(node_name);
	if (node_name_len > 80U) {
		node_name_len = 80U;
	}
	res.name.len = node_name_len;
	memcpy(res.name.data, node_name, node_name_len);
	uint8_t buffer[UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_MAX_SIZE];
	memset(buffer, 0, sizeof(buffer));
	const uint32_t total_size = uavcan_protocol_GetNodeInfoResponse_encode(&res,
			buffer);
	/*
	 * Service response використовує transfer-ID із request.
	 */
	uint8_t tid = transfer->transfer_id;
	const int16_t result = canardRequestOrRespond(&canard,
			transfer->source_node_id,
			UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_SIGNATURE,
			UAVCAN_PROTOCOL_GETNODEINFO_ID, &tid, transfer->priority,
			CanardTransferTypeResponse, buffer, total_size);
	if (result < 0) {
		__NOP();
	}
}

static void sendEscStatusTest(void) {
	static float test_voltage = 24.0F;
	static float test_current = 1.0F;
	static int32_t test_rpm = 1000;
	struct uavcan_equipment_esc_Status msg;
	memset(&msg, 0, sizeof(msg));
	/*
	 * esc_index - номер ESC.
	 * Для першого ESC ставимо 0.
	 */
	msg.esc_index = 0;
	/*
	 * Тестові значення.
	 */
	msg.error_count = 0;
	msg.voltage = test_voltage;
	msg.current = test_current;
	msg.temperature = 35.0F;
	msg.rpm = 2000;
	msg.power_rating_pct = 10;
	uint8_t buffer[UAVCAN_EQUIPMENT_ESC_STATUS_MAX_SIZE];
	memset(buffer, 0, sizeof(buffer));
	const uint32_t total_size = uavcan_equipment_esc_Status_encode(&msg,
			buffer);
	const int16_t result = canardBroadcast(&canard,
	UAVCAN_EQUIPMENT_ESC_STATUS_SIGNATURE,
	UAVCAN_EQUIPMENT_ESC_STATUS_ID, &esc_status_transfer_id,
	CANARD_TRANSFER_PRIORITY_LOW, buffer, total_size);
	if (result < 0) {
		__NOP();
	}
	/*
	 * Невелика зміна значень, щоб у моніторі було видно, що telemetry жива.
	 */
	test_current += 0.1F;
	if (test_current > 5.0F) {
		test_current = 1.0F;
	}
	test_rpm += 100;
	if (test_rpm > 3000) {
		test_rpm = 1000;
	}
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
