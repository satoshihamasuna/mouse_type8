/* Copyright 2026 hmsn
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef TEST_SIDE_WALL_PERIPHERAL_H_
#define TEST_SIDE_WALL_PERIPHERAL_H_

#include <cstdint>

enum t_sensor_dir { sensor_fl = 3, sensor_fr = 4, sensor_sl = 1, sensor_sr = 2 };
extern int16_t side_wall_test_adc[5];
inline int16_t Sensor_GetValue(t_sensor_dir direction) { return side_wall_test_adc[direction]; }
inline void Indicate_LED(uint8_t) {}
inline uint8_t Return_LED_Status() { return 0; }

#endif
