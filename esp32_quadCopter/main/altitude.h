#ifndef __ALTITUDE_H__
#define __ALTITUDE_H__

typedef struct{
	float x;
	float y;
	float z;
	float vx;
	float vy;
	float vz;
	float ax;
	float ay;
	float az;
}Nav_t;

extern Nav_t nav;

void AltitudeCombineUpdate(float _altitude);

#endif
