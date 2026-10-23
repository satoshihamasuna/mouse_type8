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
#ifndef TEST_SIDE_WALL_VEHICLE_H_
#define TEST_SIDE_WALL_VEHICLE_H_

#include "Peripheral/Inc/peripheral.h"

class param_element
{
    float value = 0.0f;
public:
    void set(float next) { value = next; }
    void init() { value = 0.0f; }
    float get() const { return value; }
};

struct TestVehicleState
{
    param_element velo, accel, length, radian, rad_velo, rad_accel, x_point;
};

struct Vehicle
{
    TestVehicleState ego, ideal;
};

#endif
