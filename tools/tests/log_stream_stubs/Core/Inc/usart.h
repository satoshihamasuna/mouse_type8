#ifndef TEST_LOG_STREAM_USART_H
#define TEST_LOG_STREAM_USART_H
#include <stdint.h>
enum { HAL_OK = 0, HAL_ERROR = 1, DMA_IT_HT = 2,
       HAL_UART_STATE_READY = 0, HAL_UART_STATE_BUSY_TX = 1 };
struct DMA_HandleTypeDef { uint32_t half_interrupt_disables; };
inline void test_disable_dma_it(DMA_HandleTypeDef *dma, uint32_t mask)
{
    if(mask == DMA_IT_HT) ++dma->half_interrupt_disables;
}
#define __HAL_DMA_DISABLE_IT(dma, mask) test_disable_dma_it(dma, mask)
struct UART_HandleTypeDef { void *Instance; uint32_t gState; uint32_t RxState; DMA_HandleTypeDef *hdmatx; };
#define USART1 reinterpret_cast<void *>(1)
extern UART_HandleTypeDef huart1;
int HAL_UART_Receive(UART_HandleTypeDef *, uint8_t *, uint16_t, uint32_t);
int HAL_UART_Receive_DMA(UART_HandleTypeDef *, uint8_t *, uint16_t);
int HAL_UART_Transmit_DMA(UART_HandleTypeDef *, uint8_t *, uint16_t);
int HAL_UART_AbortReceive(UART_HandleTypeDef *);
int HAL_UART_DMAStop(UART_HandleTypeDef *);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *);
#endif
