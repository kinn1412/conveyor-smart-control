#include "uart_link.h"

#include "main.h"

extern UART_HandleTypeDef huart1;

static uint8_t rx_dma[64];
static uint16_t rx_pos;
static uart_rx_cb_t rx_cb;
static uart_tx_cb_t tx_cb;

static void rx_restart(void)
{
    rx_pos = 0;
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_dma, sizeof rx_dma);
}

void uart_link_start(uart_rx_cb_t on_rx, uart_tx_cb_t on_tx_done)
{
    rx_cb = on_rx;
    tx_cb = on_tx_done;
    rx_restart();
}

void uart_link_send(const uint8_t *buf, size_t len)
{
    HAL_UART_Transmit_DMA(&huart1, (uint8_t *)buf, (uint16_t)len);
}

/* Called on HT, TC and IDLE. size = DMA write position in rx_dma. */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart != &huart1 || size == rx_pos)
        return;
    if (size > rx_pos) {
        rx_cb(&rx_dma[rx_pos], size - rx_pos);
    } else {
        rx_cb(&rx_dma[rx_pos], sizeof rx_dma - rx_pos);
        if (size)
            rx_cb(rx_dma, size);
    }
    rx_pos = size == sizeof rx_dma ? 0 : size;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1)
        tx_cb();
}

/* Overrun / noise / framing stop the DMA reception: start it again */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1)
        rx_restart();
}
