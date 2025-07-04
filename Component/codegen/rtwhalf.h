/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: rtwhalf.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 07-Mar-2024 12:28:31
 */

#ifndef RTWHALF_H
#define RTWHALF_H

/* Include Files */

#include "rtwtypes.h"
#include "stdint.h"

typedef struct {
	uint16_T bitPattern;
} half_float;

typedef half_float real16_T;

typedef struct {
  real16_T re;
  real16_T im;
} creal16_T;

/* Utility functions */
uint16_T getBitfieldFromHalf(real16_T a);
real16_T getHalfFromBitfield(uint16_T a);

uint32_T getBitfieldFromFloat(float a);
float getFloatFromBitfield(uint32_T a);

/* Data Type Conversion */
float halfToFloat(real16_T a);
double halfToDouble(real16_T a);

real16_T floatToHalf(float a);
real16_T doubleToHalf(double a);

/* Math functions */
real16_T sin_half(real16_T a);
real16_T cos_half(real16_T a);
real16_T ceil_half(real16_T a);
real16_T fix_half(real16_T a);
real16_T floor_half(real16_T a);
real16_T exp_half(real16_T a);
real16_T log_half(real16_T a);
real16_T log10_half(real16_T a);
real16_T sqrt_half(real16_T a);


#endif
/*
 * File trailer for rtwhalf.h
 *
 * [EOF]
 */
