#ifndef UART_LINK_H
#define UART_LINK_H

#include <stddef.h>
#include <stdint.h>

typedef void (*uart_rx_cb_t)(const uint8_t *data, size_t len); /* ISR context */
typedef void (*uart_tx_cb_t)(void);                            /* ISR context */

/* USART1 RX DMA circular + IDLE. Call from a task, after the scheduler has started. */
void uart_link_start(uart_rx_cb_t on_rx, uart_tx_cb_t on_tx_done);
/* TX DMA; buf must stay valid until on_tx_done. TxTask only (REQ-F05). */
void uart_link_send(const uint8_t *buf, size_t len);

#endif
