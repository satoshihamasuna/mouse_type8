/*
 * communicate.cpp
 *
 *  Created on: Jul 4, 2025
 *      Author: sato1
 */



#include "communicate.h"
#include <stdio.h>
#include "queue.h"

#define _BS				0x08	// バックスペース
#define _ESC			0x1b	// エスケープシーケンス
#define TRX_BUFFER_SIZE 128		// 送受信バッファサイズ


ring_queue<TRX_BUFFER_SIZE,volatile uint8_t> tx_buffer;
ring_queue<TRX_BUFFER_SIZE,volatile uint8_t> rx_buffer;

volatile uint8_t		tx_data;
volatile uint8_t		rx_data;

/* ---------------------------------------------------------------
	UART1で1文字受信する関数
--------------------------------------------------------------- */
uint8_t Communicate_TerminalRecv( void )
{
	uint8_t data[1];
	HAL_UART_Receive( &huart1, (uint8_t*)data, sizeof(data), 1 );
	return (*data);
}


void Communicate_RxPushData( void )
{
// head（DMACが受信データを書き込む位置）に新しく受信データが蓄積される．
// tailから読み出せばいい
// headがtailに追いつくとバッファオーバーフローとなり正しく読み出せない

	if(rx_buffer.queue_length() >= TRX_BUFFER_SIZE)
	{
		return;
	}

	rx_buffer.push(rx_data);					// 書き込みポインタにデータを格納

}

uint8_t Communicate_RxPopData( void )
{
	uint8_t ch;

	// この関数は多重に実行されるとまずいので割り込みを禁止する
	__disable_irq();

	// データがない場合
	if(rx_buffer.queue_length() == 0){
		// 割り込み許可
		__enable_irq();

		// データを受信するまで待機
		while(rx_buffer.queue_length() == 0)
		{
			HAL_UART_Receive_DMA( &huart1, (uint8_t*)(&rx_data), 1 );
		}
		__disable_irq();
	} else;

	ch = rx_buffer.pop();	// 読み出しデータの取り出し


	// 割り込み許可
	__enable_irq();
	return ch;
}


/* ---------------------------------------------------------------
	UART1で1文字送信する関数
--------------------------------------------------------------- */
void Communicate_TxPushData( int8_t data )
{
//// headに新しく追加する
//// tailは次に送信されるデータを指す
//// バッファに空きが無い（headがtailに追いついた）場合は待機する
//
//	// バッファ内データ数をカウントし，空きがない場合待機する
//	// バッファフルで待機しているときには割り込みを許可するためにwhileループになっている
	while(1) {
		// この関数は多重に実行されるとまずいので割り込みを禁止する
		__disable_irq();

		// DMAを一時的に停止
		HAL_DMA_Abort(huart1.hdmatx);

		// バッファに空きがあればループから抜ける
		if( tx_buffer.queue_length() < TRX_BUFFER_SIZE ) {
				break;
		} else;

		// DMA動作再開
		HAL_UART_Transmit_DMA(&huart1, (uint8_t*)(&tx_data), 1);

		// 割り込み許可
		__enable_irq();

		// バッファに空きができるまで待機（この間割り込みが発生してもよい）
		while(tx_buffer.queue_length() == TRX_BUFFER_SIZE);
	}
	// ここの時点でDMACは停止，割り込みは禁止されている

	// 書き込みポインタにデータを格納
	tx_buffer.push((uint8_t)data);

	// DMA動作再開
	HAL_UART_Transmit_DMA(&huart1, (uint8_t*)(&tx_data), 1);

	// 割り込み許可
	__enable_irq();
}

void Communicate_TxPopData( void )
{
	// データがない場合
	if(tx_buffer.queue_length() == 0)
	{
		tx_data = '\0';
		// DMAを停止
		HAL_UART_DMAStop(&huart1);
	}
	else
	{
		// 読み出しデータの取り出し
		tx_data = tx_buffer.pop();
		// DMA動作再開
		HAL_UART_Transmit_DMA(&huart1, (uint8_t*)(&tx_data), 1);
	}
}


/* ---------------------------------------------------------------
	受信・送信完了時のコールバック関数
--------------------------------------------------------------- */
void HAL_UART_RxCpltCallback( UART_HandleTypeDef *huart )
{
    if( huart->Instance == USART1 ) {
        Communicate_RxPushData();
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
