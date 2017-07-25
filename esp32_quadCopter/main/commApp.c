#include "commApp.h"
#include "imu.h"
#include "control.h"
#include "esp32delay.h"
#include "altitude.h"
#include "battery.h"
#include "gattsCom.h"
#include "fbm320.h"
#include <math.h>

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

uint16_t rcData[4];
CopterState_t copterState;
RCState_t rcState;

//----------to be tested--//
//cut deadband, move linear
float DBScaleLinear(float x, float x_end, float deadband)
{
	if (x > deadband) {
		return (x - deadband) / (x_end - deadband);

	} else if (x < -deadband) {
		return (x + deadband) / (x_end - deadband);

	} else {
		return 0.0f;
	}
}


void RCDataReceive()
{
	static uint16_t buttonCache[2] = {1000, 1000};
	static uint8_t reqCache = 0;

	rcData[THROTTLE] = constrain(bleData[THROTTLE], 1000, 2000);
	rcData[YAW] = constrain(bleData[YAW], 1000, 2000);
	rcData[PITCH] = constrain(bleData[PITCH], 1000, 2000);
	rcData[ROLL] = constrain(bleData[ROLL], 1000, 2000);

	if(bleData[4] <1500 && buttonCache[0] > 1500)
	{
		if(reqCache)
			reqCache = 0;
		else
			reqCache = 1;
		copterState.armState = reqCache;
	}
	else if(bleData[5] <1500 && buttonCache[1] > 1500)
	{
		if(!copterState.flyUp){
			Fbm320CaliInit();
			imu.caliFlag = 1;
			copterState.landOff = 0;
		}
	}
	buttonCache[0] = bleData[4];
	buttonCache[1] = bleData[5];
	rcState.time = millis();
}


void RCDataLost(void)
{
	uint32_t nowTime;
	uint16_t lostRCTime;

	nowTime = millis();	//ms
	lostRCTime = (nowTime>rcState.time)?(nowTime-rcState.time):(65536-rcState.time+nowTime);
	if(lostRCTime > LOST_RC_TIME_MAX)
	{
		rcState.lostFlag = 1;
		if(copterState.flyUp)
		{
			copterState.altMode = LANDING;
		}
	}
	else
		rcState.lostFlag=0;

}


void AutoLand(void)
{
	static uint32_t landStartTime=0;
	
	if(landStartTime==0){
		landStartTime = millis();
		return;
	}

	if(millis() - landStartTime > LAND_TIME_MAX /*&& ( -nav.vz <0.35)  &&( -thrustZSp < 0.6)*/  )	//闂勫秴鍩岄崷鐗堫梾濞达拷 //||(fabs(nav.vz)<0.1 && fabs(nav.az)<0.1)
	{
		landStartTime = 0;
		copterState.altMode = MANUAL;
		motorState.enable = 0;		
		copterState.flyUp = 0;
		copterState.landOff = 1;
	}
}


#define SAFE_ANGLE_MAX		40
//copter crash down
void FailSafeCrash(void)
{
	if(fabs(imu.pitch)>SAFE_ANGLE_MAX || fabs(imu.roll)>SAFE_ANGLE_MAX)
	{
		copterState.altMode = MANUAL;
		motorState.enable = 0;
		copterState.flyUp = 0;
	}
}

// input: rcData , raw data from remote control source
// output: RC_DATA, desired  thro, pitch, roll, yaw
void RCDataProcess(void)
{
	rcState.data[THROTTLE] = rcData[THROTTLE] - 1000;
	rcState.data[YAW] = RC_YAW_RATE_MAX * DBScaleLinear((rcData[YAW] - 1500), 500, RC_YAW_DB);
	rcState.data[PITCH] = -RC_ANGLE_MAX * DBScaleLinear((rcData[PITCH] - 1500), 500, RC_PR_DB);
	rcState.data[ROLL] = -RC_ANGLE_MAX * DBScaleLinear((rcData[ROLL] - 1500), 500, RC_PR_DB);

	switch(copterState.armState)
	{
		case REQ_ARM:
			if(!battery.alarm && IMUCheck())
			{	
				copterState.armState = ARMED;
				motorState.enable = 0xA5;
			}
			else
			{
				copterState.armState = DISARMED;				
				motorState.enable = 0;
			}
			break;
		case REQ_DISARM:
			copterState.altMode = MANUAL;		//娑撳﹪鏀ｉ崥搴″閻ㄥ嫬顦╅悶锟�
			copterState.armState = DISARMED;
			motorState.enable = 0;		
			copterState.flyUp = 0;	
			break;
		default:
			break;			
	}	

	if(copterState.altMode==LANDING)
	{
		AutoLand();
		rcState.data[THROTTLE] = 0;
		rcState.data[YAW] = 0;
		rcState.data[PITCH] = 0;
		rcState.data[ROLL] = 0;
	}
		
	if(motorState.enable)
	{
		if(rcState.data[THROTTLE]>=600 && copterState.altMode!=CLIMB_RATE)
		{
			copterState.altMode = CLIMB_RATE;
			copterState.flyUp = 1;
			zIntReset = 1;
			thrustZSp = 0;
			altLand = -nav.z;		//鐠佹澘缍嶇挧鐑筋棧閺冨墎娈戞妯哄
			if(copterState.headMode)
				copterState.headYaw = imu.yaw;
		}
		else if(copterState.altMode==MANUAL)
			rcState.data[THROTTLE] = SLOW_THRO;           //閹靛濮╁Ο鈥崇础瀵板懏婧�鏉烇拷200

		FailSafeCrash();
	} 

}

