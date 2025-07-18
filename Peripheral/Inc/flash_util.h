/*
 * flash_util.h
 *
 *  Created on: Jul 18, 2025
 *      Author: sato1
 */

#ifndef INC_FLASH_UTIL_H_
#define INC_FLASH_UTIL_H_

#include <stdio.h>
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

void eraseFlash() ;
void writeFlash(uint32_t address, uint8_t *data) ;

uint8_t* Flash_Load( void );
uint8_t Flash_Save( void );

//static uint8_t _flash_start;

void work_ram_set(uint32_t position,uint8_t data);

uint8_t work_ram_read(uint32_t position);

#ifdef __cplusplus
}
#endif

#endif /* INC_FLASH_UTIL_H_ */
