#ifndef INC_HW_ESC_TELEM_H_
#define INC_HW_ESC_TELEM_H_

#include <stdint.h>
#include "usart.h"

#define HWESC_HEADER           0x9B
#define HWESC_LENGTH_BYTE      0x16
#define HWESC_FRAME_SIZE       24
#define HWESC_COUNT 			6 // Кількість портів

typedef struct {
	uint16_t rpm;
	uint16_t voltage_v;
	uint16_t phase_current_a;
	uint16_t current_a;
} hwesc_telemetry;

typedef struct {
    UART_HandleTypeDef *huart;
    uint8_t buf[HWESC_FRAME_SIZE];
    uint8_t rx_done;
    uint8_t rx_error;
} hwesc_io;

typedef struct {
    hwesc_io io;
    hwesc_telemetry telem;
} hwesc_ctx;

void hwescStart(hwesc_ctx *ctx, UART_HandleTypeDef *huart);
void hwescParse(hwesc_ctx *ctx);
hwesc_ctx *hwesc_find_by_huart(UART_HandleTypeDef *huart);

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);


#endif /* INC_HW_ESC_TELEM_H_ */
