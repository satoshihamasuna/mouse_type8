/*
 * half_float.h
 *
 *  Created on: Jul 4, 2025
 *      Author: sato1
 */

#ifndef INC_HALF_FLOAT_H_
#define INC_HALF_FLOAT_H_


#include <stdint.h>
#include "../codegen/rtwhalf.h"

half_float float_to_half(float f) ;
float half_to_float(half_float hf);


#endif /* INC_HALF_FLOAT_H_ */
