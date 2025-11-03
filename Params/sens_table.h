/*
 * sens_table.h
 *
 *  Created on: 2023/06/24
 *      Author: sato1
 */

#ifndef INCLUDE_SENS_TABLE_H_
#define INCLUDE_SENS_TABLE_H_

#include "mouse_config.h"

// ===== 共通部分 =====
const static int16_t sens_side_length_table[] =
{
    25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80
};

const static int16_t sens_front_length_table[] =
{
    40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95,
    100, 105, 110, 115, 120, 125
};


// センサタイプごとのパラメータ読み込み
#ifdef MOUSE_A
  #include "sens_table_A.h"
#elif defined(MOUSE_B)
  #include "sens_table_B.h"
#else
  #error "No SENSOR_TYPE defined. Please define SENSOR_TYPE_A or SENSOR_TYPE_B."
#endif



#endif /* INCLUDE_SENS_TABLE_H_ */
