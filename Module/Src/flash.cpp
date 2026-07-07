/*
 * flash.cpp
 *
 *  Created on: 2024/03/07
 *      Author: sato1
 */

#include "../Inc/flash.h"
#include "../../Subsys/Inc/wall_class.h"
#include "typedef.h"
#include "flash_util.h"

#define WORK_RAM_WALL_ADDRESS_START 	0x0000
#define WORK_RAM_WALL_ADDRESS_END   	0x0FFF
#define WORK_RAM_HISTRY_DATA_ADDRESS_START 	0x1000
#define WORK_RAM_HISTRY_DATA_ADDRESS_END 	0x13FF
#define WORK_RAM_HISTRY_X_ADDRESS_START		0x1400
#define WORK_RAM_HISTRY_X_ADDRESS_END		0x17FF
#define WORK_RAM_HISTRY_Y_ADDRESS_START		0x1800
#define WORK_RAM_HISTRY_Y_ADDRESS_END		0x1BFF


void read_wall_WorkRam(wall_class *wall_property){
	//Flash_Load();
	for(uint32_t x = 0 ; x < MAZE_SIZE_X ; x++ ){
		for(uint32_t y = 0 ; y < MAZE_SIZE_Y ; y++ ){
			uint8_t data = work_ram_read((x << 6 | y));
			wall_property->wall[x][y].north = (data >> 6) & 0x0003;
			wall_property->wall[x][y].east  = (data >> 4) & 0x0003;
			wall_property->wall[x][y].south = (data >> 2) & 0x0003;
			wall_property->wall[x][y].west  = (data >> 0) & 0x0003;
		}
	}
}

void write_wall_WorkRam(wall_class *wall_property){
	for(uint32_t x = 0 ; x < MAZE_SIZE_X ; x++ ){
		for(uint32_t y = 0 ; y < MAZE_SIZE_Y ; y++ ){
			uint8_t data = (((uint8_t)(wall_property->wall[x][y].north)<< 6) | ((uint8_t)(wall_property->wall[x][y].east) << 4)
					  | ((uint8_t)(wall_property->wall[x][y].south) << 2) | ((uint8_t)(wall_property->wall[x][y].west) << 0));
			work_ram_set((x << 6 | y),data);
		}
	}
}


void write_wall_flash(wall_class *wall_property){
	write_histry_WorkRam(&(wall_property->wall_histry));
	Flash_Save();
}


void read_histry_WorkRam(wall_histry_class *_wall_histry){
	//Flash_Load();
	_wall_histry->histry_init();
	for(uint32_t i = 0 ; i < MAZE_SIZE ; i++ ){
			uint8_t data = work_ram_read((WORK_RAM_HISTRY_DATA_ADDRESS_START + i));
			uint8_t    x = work_ram_read((WORK_RAM_HISTRY_X_ADDRESS_START + i));
			uint8_t    y = work_ram_read((WORK_RAM_HISTRY_Y_ADDRESS_START + i));
			if(!((int8_t)(x) == -1 && (int8_t)(y) == -1))
			{
				t_wall wall_data;
				wall_data.north = (data >> 6) & 0x0003;
				wall_data.east  = (data >> 4) & 0x0003;
				wall_data.south = (data >> 2) & 0x0003;
				wall_data.west  = (data >> 0) & 0x0003;
				_wall_histry->histry_set((int8_t)(x), (int8_t)(y), wall_data);
			}
	}

}

void write_histry_WorkRam(wall_histry_class *_wall_histry){
	for(uint32_t i = 0 ; i < MAZE_SIZE ; i++ ){
			uint8_t data = ( ((uint8_t)(_wall_histry->histry_wall[i].wall.north)<< 6)
							|((uint8_t)(_wall_histry->histry_wall[i].wall.east) << 4)
							|((uint8_t)(_wall_histry->histry_wall[i].wall.south)<< 2)
							|((uint8_t)(_wall_histry->histry_wall[i].wall.west) << 0));
			uint8_t x = (uint8_t)(_wall_histry->histry_wall[i].x);
			uint8_t y = (uint8_t)(_wall_histry->histry_wall[i].y);
			work_ram_set((WORK_RAM_HISTRY_DATA_ADDRESS_START + i),data);
			work_ram_set((WORK_RAM_HISTRY_X_ADDRESS_START + i),x);
			work_ram_set((WORK_RAM_HISTRY_Y_ADDRESS_START + i),y);
	}
}


void write_histry_flash(wall_histry_class *_wall_histry){
	write_histry_WorkRam(_wall_histry);
	Flash_Save();
}



void write_save_data(wall_class *wall_property){
	write_wall_WorkRam(&(*wall_property));
	write_histry_WorkRam(&(wall_property->wall_histry));
	Flash_Save();

}

void read_save_data(wall_class *wall_property){
	Flash_Load();
	read_wall_WorkRam(&(*wall_property));
	read_histry_WorkRam(&(wall_property->wall_histry));
}
