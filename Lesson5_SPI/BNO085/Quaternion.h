#ifndef _QUATERNION_H
#define _QUATERNION_H

#include "main.h"
#include <math.h>

extern float BNO085_Roll;
extern float BNO085_Pitch;
extern float BNO085_Yaw;

void Quaternion_Update(float* q);
float invSqrt(float x);

#endif
