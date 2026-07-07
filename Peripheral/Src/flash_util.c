/*
 * flash_util.c
 *
 *  Created on: Jul 18, 2025
 *      Author: sato1
 */

#include <string.h>
#include <stdint.h>
#include "stm32u5xx_hal.h"
#include "flash_util.h"


extern uint8_t _user_data_start[]; // リンカスクリプトのシンボル（開始アドレス）
extern uint8_t _user_data_end[];   // 終了アドレス

#define USER_DATA_START_ADDRESS (0x0807E000)
#define USER_DATA_SIZE          (0x2000)

uint8_t store_ram[USER_DATA_SIZE];

void eraseFlash() {
    FLASH_EraseInitTypeDef erase;
    erase.TypeErase  = FLASH_TYPEERASE_PAGES;
    erase.Page       = 31;               // Page 31
    erase.NbPages    = 1;
    erase.Banks      = FLASH_BANK_2;     // Bank 2

    uint32_t pageError = 0;
    if (HAL_FLASHEx_Erase(&erase, &pageError) != HAL_OK) {
        // エラー処理
    }
}

void writeFlash(uint32_t address, uint8_t *data) {
    if (address % 16 != 0) return; // 16バイトアライン必須

    uint64_t quadword[2];
    memcpy(quadword, data, sizeof(quadword));

    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD, address, (uint32_t)(uintptr_t)quadword) != HAL_OK) {
        // エラー処理
    }
}

uint8_t* Flash_Load(void) {
    memcpy(store_ram, (uint8_t*)USER_DATA_START_ADDRESS, USER_DATA_SIZE);
    return store_ram;
}

uint8_t Flash_Save(void) {
    HAL_FLASH_Unlock();
    eraseFlash();

    HAL_StatusTypeDef result = HAL_OK;
    size_t write_cnt = USER_DATA_SIZE / 16;

    uint32_t p_work_ram = (uint32_t)(uintptr_t)store_ram;

    for (size_t i = 0; i < write_cnt; i++) {
        result = HAL_FLASH_Program(
            FLASH_TYPEPROGRAM_QUADWORD,
            USER_DATA_START_ADDRESS + (i * 16),
            (p_work_ram + (i * 16))
        );

        if (result != HAL_OK) break;
    }

    HAL_FLASH_Lock();

    return (result == HAL_OK) ? 1 : 0;
}

void work_ram_set(uint32_t position, uint8_t data) {
    if (position < USER_DATA_SIZE) {
        store_ram[position] = data;
    }
}

uint8_t work_ram_read(uint32_t position) {
    if (position < USER_DATA_SIZE) {
        return store_ram[position];
    }
    return 0xFF; // エラー値
}
