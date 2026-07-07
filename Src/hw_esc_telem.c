#include "hw_esc_telem.h"

hwesc_ctx esc_list[HWESC_COUNT];

void hwescStart(hwesc_ctx *ctx, UART_HandleTypeDef *huart)
{
    ctx->io.huart = huart;
    ctx->io.rx_done = 0;
    ctx->io.rx_error = 0;
    HAL_UART_Receive_DMA(ctx->io.huart, ctx->io.buf, HWESC_FRAME_SIZE);
}

hwesc_ctx *hwesc_find_by_huart(UART_HandleTypeDef *huart) {
	for(uint8_t i = 0; i < HWESC_COUNT; i++) {
	        if (esc_list[i].io.huart == huart) {
	            return &esc_list[i];
	        }
	    }
	    return NULL;
}

void hwescParse(hwesc_ctx *ctx)
{
    const uint8_t *buf = ctx->io.buf;
    ctx->telem.rpm = (uint16_t)((buf[0] << 8) | buf[1]);
    ctx->telem.voltage_v = (uint16_t)((buf[2] << 8) | buf[3]);
    ctx->telem.phase_current_a = (uint16_t)((buf[4] << 8) | buf[5]);
    ctx->telem.current_a = (uint16_t)((buf[6] << 8) | buf[7]);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	hwesc_ctx *ctx = hwesc_find_by_huart(huart);
	if (ctx == NULL) return;
	hwescParse(ctx);
	HAL_UART_Receive_DMA(ctx->io.huart, ctx->io.buf, HWESC_FRAME_SIZE);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	hwesc_ctx *ctx = hwesc_find_by_huart(huart);
	if (ctx == NULL) return;
	HAL_UART_DMAStop(ctx->io.huart);
	HAL_UART_Receive_DMA(ctx->io.huart, ctx->io.buf, HWESC_FRAME_SIZE);
}
