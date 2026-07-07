/*
 * flash.h
 *
 *  Created on: 2023/07/03
 *      Author: sato1
 */

#ifndef CPP_INC_FLASH_H_
#define CPP_INC_FLASH_H_

#include "../../Subsys/Inc/wall_class.h"
#include "typedef.h"
#include "flash_util.h"

void write_save_data(wall_class *wall_property);
//ここをポインタのポインタに変更する
void read_save_data(wall_class *wall_property);
//ここをポインタのポインタに変更する

void write_wall_WorkRam(wall_class *wall_property);
void read_wall_WorkRam(wall_class *wall_property);

//void read_wall_flash(wall_class *wall_property);
void write_wall_flash(wall_class *wall_property);

void write_histry_WorkRam(wall_histry_class *_wall_histry);
void read_histry_WorkRam(wall_histry_class *_wall_histry);


//void read_histry_flash(wall_histry_class *_wall_histry);
void write_histry_flash(wall_histry_class *_wall_histry);


#endif /* CPP_INC_FLASH_H_ */
