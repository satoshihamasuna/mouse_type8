/*
 * half_float.cpp
 *
 *  Created on: Jul 4, 2025
 *      Author: sato1
 */

#include "half_float.h"
#include <stdint.h>
#include <math.h>


float half_to_float(half_float hf)
{
	return halfToFloat(hf);
}

half_float float_to_half(float f)
{
	return floatToHalf(f);
}


