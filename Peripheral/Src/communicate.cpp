/*
 * communicate.cpp
 *
 *  Created on: Jul 4, 2025
 *      Author: sato1
 */



#include "Peripheral/Inc/communicate.h"
#include <stdio.h>
#include <string.h>
#include "Component/Inc/queue.h"
#include "Module/Inc/log_stream.h"

#define _BS				0x08	// バックスペース
#define _ESC			0x1b	// エスケープシーケンス
#define RX_BUFFER_SIZE 128		// 受信バッファサイズ
#define TX_BUFFER_SIZE 512		// 複数のログフレームを DMA 完了前に予約する


static uint8_t tx_buffer[TX_BUFFER_SIZE];
static volatile uint16_t tx_read, tx_write, tx_used, tx_dma_size;
ring_queue<RX_BUFFER_SIZE,volatile uint8_t> rx_buffer;

volatile uint8_t		rx_data;
volatile uint8_t		rx_continuous_enabled;

/* ---------------------------------------------------------------
	UART1で1文字受信する関数
--------------------------------------------------------------- */
uint8_t Communicate_TerminalRecv( void )
{
	uint8_t data[1];
	HAL_UART_Receive( &huart1, (uint8_t*)data, sizeof(data), 1 );
	return (*data);
}


static void rx_push_byte(uint8_t byte)
{
// head（DMACが受信データを書き込む位置）に新しく受信データが蓄積される．
// tailから読み出せばいい
// headがtailに追いつくとバッファオーバーフローとなり正しく読み出せない

	if(rx_buffer.queue_length() >= RX_BUFFER_SIZE)
	{
		return;
	}

	rx_buffer.push(byte);					// 書き込みポインタにデータを格納

}

void Communicate_RxPushData( void )
{
	LogStream::receive_byte(rx_data, HAL_GetTick(), rx_push_byte);
}

uint8_t Communicate_RxPopData( void )
{
	uint8_t ch;
	while(Communicate_RxTryPopData(&ch) == 0U) {
		if(huart1.RxState == HAL_UART_STATE_READY) {
			HAL_UART_Receive_DMA(&huart1, (uint8_t*)(&rx_data), 1);
		}
	}
	return ch;
}

uint8_t Communicate_RxTryPopData( uint8_t *data )
{
	if(data == NULL) {
		return 0U;
	}

	__disable_irq();
	LogStream::flush_rx(HAL_GetTick(), rx_push_byte);
	if(rx_buffer.queue_length() == 0) {
		__enable_irq();
		return 0U;
	}

	*data = rx_buffer.pop();
	__enable_irq();
	return 1U;
}

void Communicate_RxStartContinuous( void )
{
	rx_continuous_enabled = 1U;
	if(huart1.RxState == HAL_UART_STATE_READY) {
		HAL_UART_Receive_DMA(&huart1, (uint8_t*)(&rx_data), 1);
	}
}

void Communicate_RxStopContinuous( void )
{
	rx_continuous_enabled = 0U;
	HAL_UART_AbortReceive(&huart1);
}


/* ---------------------------------------------------------------
	UART1で1文字送信する関数
--------------------------------------------------------------- */
static void tx_start_dma()
{
	// Caller holds PRIMASK. Bytes remain reserved until UART TC, not DMA TC.
	if(tx_dma_size != 0 || tx_used == 0 || huart1.gState != HAL_UART_STATE_READY) return;
	const uint16_t contiguous = TX_BUFFER_SIZE - tx_read;
	const uint16_t length = tx_used < contiguous ? tx_used : contiguous;
	if(HAL_UART_Transmit_DMA(&huart1, tx_buffer + tx_read, length) == HAL_OK)
	{
		tx_dma_size = length;
		// Only completion is needed; avoid an otherwise unused half-transfer IRQ.
		__HAL_DMA_DISABLE_IT(huart1.hdmatx, DMA_IT_HT);
	}
}

void Communicate_TxPushData( int8_t data )
{
	const uint8_t byte = static_cast<uint8_t>(data);
	while(Communicate_TxTryPushBuffer(&byte, 1) == 0U) {}
}

void Communicate_TxPushBuffer(uint8_t *buf, uint16_t len)
{
	while(len != 0)
	{
		const uint16_t length = len < TX_BUFFER_SIZE ? len : TX_BUFFER_SIZE;
		while(Communicate_TxTryPushBuffer(buf, length) == 0U) {}
		buf += length;
		len -= length;
	}
}

uint8_t Communicate_TxTryPushBuffer(const uint8_t *buf, uint16_t len)
{
	if(buf == NULL || len == 0 || len > TX_BUFFER_SIZE) return 0U;
	const uint32_t primask = __get_PRIMASK();
	__disable_irq();
	if(tx_used + len > TX_BUFFER_SIZE)
	{
		__set_PRIMASK(primask);
		return 0U;
	}
	const uint16_t contiguous = TX_BUFFER_SIZE - tx_write;
	const uint16_t first = len < contiguous ? len : contiguous;
	memcpy(tx_buffer + tx_write, buf, first);
	if(first < len) memcpy(tx_buffer, buf + first, len - first);
	tx_write = (tx_write + len) % TX_BUFFER_SIZE;
	tx_used += len;
	tx_start_dma();
	__set_PRIMASK(primask);
	return 1U;
}

void Communicate_TxPopData( void )
{
	const uint32_t primask = __get_PRIMASK();
	__disable_irq();
	tx_read = (tx_read + tx_dma_size) % TX_BUFFER_SIZE;
	tx_used -= tx_dma_size;
	tx_dma_size = 0;
	if(tx_used == 0) tx_read = tx_write = 0;
	tx_start_dma();
	__set_PRIMASK(primask);
}

void Communicate_Service( void )
{
	// Low-priority SysTick: retry a failed DMA start and prepare at most one frame.
	if(tx_used != 0 && tx_dma_size == 0)
	{
		const uint32_t primask = __get_PRIMASK();
		__disable_irq();
		tx_start_dma();
		__set_PRIMASK(primask);
	}
	LogStream::service(HAL_GetTick());
}


/* ---------------------------------------------------------------
	受信・送信完了時のコールバック関数
--------------------------------------------------------------- */
void HAL_UART_RxCpltCallback( UART_HandleTypeDef *huart )
{
    if( huart->Instance == USART1 ) {
        Communicate_RxPushData();
		if(rx_continuous_enabled != 0U) {
			HAL_UART_Receive_DMA(&huart1, (uint8_t*)(&rx_data), 1);
		}
    } else;
}

void HAL_UART_TxCpltCallback( UART_HandleTypeDef *huart )
{
    if( huart->Instance == USART1 ) {
        Communicate_TxPopData();
    } else;
}

/* ---------------------------------------------------------------
	printfとscanfを使用するための設定
--------------------------------------------------------------- */
void Communicate_Initialize( void )
{
	setbuf(stdout, NULL);
	setbuf(stdin, NULL);
	rx_continuous_enabled = 0U;
	tx_read = tx_write = tx_used = tx_dma_size = 0;
}

/* ---------------------------------------------------------------
	printfを使用するための設定
--------------------------------------------------------------- */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
extern "C" PUTCHAR_PROTOTYPE
{
	Communicate_TxPushData(ch);
	return 1;
}

/* ---------------------------------------------------------------
	scanfを使用するための設定
--------------------------------------------------------------- */
#ifdef __GNUC__
#define GETCHAR_PROTOTYPE int __io_getchar(void)
#else
#define GETCHAR_PROTOTYPE int fgetc(FILE *f)
#endif /* __GNUC__ */
extern "C" GETCHAR_PROTOTYPE
{
	//return Communicate_TerminalRecv();
	return Communicate_RxPopData();

}
