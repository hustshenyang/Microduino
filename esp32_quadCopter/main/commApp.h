#ifndef __COMM_APP_H__
#define __COMM_APP_H__

#include "stdint.h"

#define LOST_RC_TIME_MAX  	800
#define PI_F            	3.141592657f

//瀵板懘锟界喕娴�
#define SLOW_THRO 			100
#define LAND_TIME_MAX 		4000
//鐎规矮绠熸鐐存簚閺堬拷婢堆冿拷鐐灘鐟欐帒瀹�
#define RC_ANGLE_MAX  		30.0
#define RC_YAW_RATE_MAX  	180.0f/PI_F		//deg/s  
#define RC_YAW_DB	 		70 //dead band 
#define RC_PR_DB			50


enum {REQ_DISARM=0, REQ_ARM, DISARMED, ARMED};
enum{ROLL,PITCH,YAW,THROTTLE};

#define CUT_DB(x,mid,DB) 		{if(fabs(x-mid)<DB) x=mid; \
								else if(x-mid>0) x=x-DB;\
								else if(x-mid<0) x=x+DB;}								
																						
typedef struct{
	uint8_t armState;
	uint8_t flyUp;
	uint8_t landOff;
	uint8_t headMode;
	uint8_t altMode;
	float headYaw;
	
}CopterState_t;


typedef struct{
	float data[4];
	uint32_t time;
	uint8_t lostFlag;
	
}RCState_t;
											

extern CopterState_t copterState;
extern RCState_t rcState;
 
float DBScaleLinear(float x, float x_end, float deadband);
void RCDataReceive(void);
void RCDataLost(void);
void RCDataProcess(void);

#endif

