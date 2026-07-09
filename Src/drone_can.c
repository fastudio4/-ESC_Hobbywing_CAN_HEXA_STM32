#include "drone_can.h"

#define DRONECAN_NODE_ID                         12U
#define UAVCAN_PROTOCOL_GETNODEINFO_ID           1U
#define CANARD_MEMORY_POOL_SIZE                  4096U
#define NODE_STATUS_PERIOD_MS                    1000U
#define CANARD_CLEANUP_PERIOD_MS                 1000U
#define ESC_STATUS_PERIOD_MS 					 100U

#include "stm32f4xx_hal.h"
#include "canard_stm32.h"
#include "uavcan.equipment.esc.Status.h"
#include "uavcan.protocol.NodeStatus.h"
#include "uavcan.protocol.GetNodeInfo_res.h"
#include "uavcan.protocol.GetNodeInfo_req.h"

CanardInstance canard;
const CanardSTM32CANTimings can_timings = { .bit_rate_prescaler = 3,
		.bit_segment_1 = 11, .bit_segment_2 = 3,
		.max_resynchronization_jump_width = 1 };
uint8_t canard_memory_pool[CANARD_MEMORY_POOL_SIZE];
uint8_t node_status_transfer_id = 0;
uint32_t last_node_status_ms = 0;
uint32_t last_cleanup_ms = 0;
uint32_t now_ms = 0;
uint8_t esc_status_transfer_id = 0;
uint32_t last_esc_status_ms = 0;

uint64_t micros64(void) {
	return (uint64_t) HAL_GetTick() * NODE_STATUS_PERIOD_MS;
}

void DroneCan_Init(void) {
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
	last_node_status_ms = HAL_GetTick();
	last_cleanup_ms = HAL_GetTick();
	last_esc_status_ms = HAL_GetTick();
	now_ms = HAL_GetTick();
}

void DroneCAN_ProcessTx(void) {
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

void DroneCAN_ProcessRx(void) {
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

void DroneCAN_Loop(void) {
	DroneCAN_ProcessRx();
	DroneCAN_ProcessTx();
	now_ms = HAL_GetTick();
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

bool shouldAcceptTransfer(const CanardInstance *ins,
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

void onTransferReceived(CanardInstance *ins, CanardRxTransfer *transfer) {
	(void) ins;
	if ((transfer->transfer_type == CanardTransferTypeBroadcast)
			&& (transfer->data_type_id == UAVCAN_PROTOCOL_NODESTATUS_ID)) {
		struct uavcan_protocol_NodeStatus msg;
		memset(&msg, 0, sizeof(msg));
		if (uavcan_protocol_NodeStatus_decode(transfer, &msg)) {
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

void sendNodeStatus(void) {
	struct uavcan_protocol_NodeStatus msg;
	memset(&msg, 0, sizeof(msg));
	msg.uptime_sec = HAL_GetTick() / 1000U;
	msg.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
	msg.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;
	msg.sub_mode = 0;
	msg.vendor_specific_status_code = 6;
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

void handleGetNodeInfoRequest(const CanardRxTransfer *transfer) {
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
	const char *node_name = "org.custom.esp.telemetry:1";
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

void sendEscStatusTest(void) {
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
