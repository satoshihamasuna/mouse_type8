/*
 * sens_table.h
 *
 *  Created on: 2023/06/24
 *      Author: sato1
 */

#ifndef INCLUDE_SENS_TABLE_H_
#define INCLUDE_SENS_TABLE_H_

#include "Params/mouse_config.h"

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
  #include "Params/sens_table_A.h"
#elif defined(MOUSE_B)
  #include "Params/sens_table_B.h"
#else
  #error "No SENSOR_TYPE defined. Please define SENSOR_TYPE_A or SENSOR_TYPE_B."
#endif


static inline float clampf(float v, float min, float max)
{
    if (v < min) return min;
    if (v > max) return max;
    return v;
}

static inline float interpolate(uint16_t v0, uint16_t v1,
                                 float d0, float d1,
                                 uint16_t value)
{
    float m = (float)(v0 - value);
    float n = (float)(value - v1);
    return (n * d0 + m * d1) / (m + n);
}

/* 等間隔距離テーブル用 汎用変換関数 */
static inline float tableToDistanceStep(
    const int16_t *sens_table,
    int array_length,
    float d_start,
    float d_step,
    int16_t value
)
{
    if (value >= sens_table[0])
        return d_start;

    if (value <= sens_table[array_length - 1])
        return d_start + d_step * (array_length - 1);

    int idx;
    for (idx = 0; idx < array_length - 1; idx++)
    {
        if (value <= sens_table[idx] && value > sens_table[idx + 1])
            break;
    }

    float d0 = d_start + d_step * idx;
    float d1 = d_start + d_step * (idx + 1);

    return interpolate(
        sens_table[idx],
        sens_table[idx + 1],
        d0, d1,
        value
    );
}


#endif /* INCLUDE_SENS_TABLE_H_ */
