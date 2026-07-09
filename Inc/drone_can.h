#ifndef INC_DRONE_CAN_H_
#define INC_DRONE_CAN_H_

#include "canard.h"


uint64_t micros64(void);

void DroneCan_Init(void);
void DroneCAN_ProcessRx(void);
void DroneCAN_ProcessTx(void);
void DroneCAN_Loop(void);
void onTransferReceived(CanardInstance *ins, CanardRxTransfer *transfer);
bool shouldAcceptTransfer(const CanardInstance *ins,
		uint64_t *out_data_type_signature, uint16_t data_type_id,
		CanardTransferType transfer_type, uint8_t source_node_id);
void sendNodeStatus(void);
void handleGetNodeInfoRequest(const CanardRxTransfer *transfer);
void sendEscStatusTest(void);

#endif /* INC_DRONE_CAN_H_ */
