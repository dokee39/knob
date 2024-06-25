#ifndef BSP_USART_H
#define BSP_USART_H

#include <stdint.h>

#define USART1_TX_BUF_LEN 128

extern void usart6_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);

extern void usart1_tx_dma_init(void);
extern void usart1_tx_dma_enable(uint8_t *data, uint16_t len);
void usart1_printf(const char *fmt, ...);
#endif
