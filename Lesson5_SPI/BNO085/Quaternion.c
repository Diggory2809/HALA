#include "Quaternion.h"

#define _180_DIV_PI 57.295779515f // = 180 / PI

float BNO085_Roll;
float BNO085_Pitch;
float BNO085_Yaw;

void Quaternion_Update(float* q)
{
	float q1, q2, q3, q4;
	float norm;

	norm = invSqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);    // normalize quaternion
	
	q1 = q[0] * norm; //x
	q2 = q[1] * norm; //y
	q3 = q[2] * norm; //z
	q4 = q[3] * norm; //w

	BNO085_Pitch = atan2f(2.0f * (q2*q3 + q1*q4), q1*q1 + q2*q2 - q3*q3 - q4*q4);
	BNO085_Roll  = -asinf(2.0f * (q2*q4 - q1*q3));
	BNO085_Yaw   = atan2f(2.0f * (q1*q2 + q3*q4), q1*q1 - q2*q2 - q3*q3 + q4*q4);

	BNO085_Pitch *= _180_DIV_PI;
	BNO085_Roll  *= _180_DIV_PI;
	BNO085_Yaw   *= _180_DIV_PI;
	
	if(BNO085_Yaw>=0)
		BNO085_Yaw = 360.f - BNO085_Yaw;
	else	
		BNO085_Yaw = -BNO085_Yaw;
	
	
	if(BNO085_Pitch>=0)
		BNO085_Pitch = 180.f - BNO085_Pitch;
	else
		BNO085_Pitch = -(BNO085_Pitch + 180.f);
}

// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}
