#include <stdbool.h>
#include "control.h"
#include "motor.h"
#include "math.h"
#include "imu.h"
#include "commApp.h"
#include "altitude.h"
#include "battery.h"
#include "esp32delay.h"

uint8_t isAltLimit=0;
float altLand;
float rollSp =0,pitchSp =0, thrustZSp=0;		//鏍规嵁鍔ㄥ姏鍒嗛厤閲嶆柊璁＄畻寰楀埌鐨勬湡鏈況oll pitch
float thrustZInt=0;
uint8_t zIntReset=1;	//integral reset at first . when change manual mode to climb rate mode
uint8_t satZ=0,satXY=0;	//鏄惁杩囬ケ鍜�

MotorState_t motorState; 

//----PID缁撴瀯浣撳疄渚嬪寲----
PID_Typedef pitch_angle_PID;	//pitch瑙掑害鐜殑PID
PID_Typedef pitch_rate_PID;		//pitch瑙掗�熺巼鐜殑PID

PID_Typedef roll_angle_PID;   //roll瑙掑害鐜殑PID
PID_Typedef roll_rate_PID;    //roll瑙掗�熺巼鐜殑PID

PID_Typedef yaw_angle_PID;    //yaw瑙掑害鐜殑PID
PID_Typedef yaw_rate_PID;     //yaw瑙掗�熺巼鐜殑PID

PID_Typedef alt_PID;
PID_Typedef alt_vel_PID;


//-----------浣嶇疆寮廝ID-----------
void PID_Postion_Cal(PID_Typedef * PID, float target,float measure, int32_t dertT, bool updataInteg)
{
	float termI=0;
	float dt= dertT/1000000.0;
	//-----------浣嶇疆寮廝ID-----------
	//璇樊=鏈熸湜鍊�-娴嬮噺鍊�
	PID->Error=target-measure;
	
	PID->Deriv= (PID->Error-PID->PreError)/dt;
	
	PID->Output=(PID->P * PID->Error) + (PID->I * PID->Integ) + (PID->D * PID->Deriv);    //PID:姣斾緥鐜妭+绉垎鐜妭+寰垎鐜妭
	
	PID->PreError=PID->Error;
	//浠呯敤浜庤搴︾幆鍜岃閫熷害鐜殑

	if(updataInteg)
	{
//		if(fabs(PID->Output) < motorState.Thro)		              //姣旀补闂ㄨ繕澶ф椂涓嶇Н鍒�
		{
			termI=(PID->Integ) + (PID->Error) * dt;     //绉垎鐜妭
			if(termI > - PID->iLimit && termI < PID->iLimit && PID->Output > - PID->iLimit && PID->Output < PID->iLimit)       //鍦�-300~300鏃舵墠杩涜绉垎鐜妭
				PID->Integ=termI;
		}
	}
	else
		PID->Integ= 0;
}


//鍑芥暟鍚嶏細CtrlAlti()
//杈撳叆锛氭棤
//杈撳嚭: 鏈�缁堢粨鏋滆緭鍑哄埌鍏ㄥ眬鍙橀噺thrustZSp
//鎻忚堪锛氭帶鍒堕珮搴︼紝涔熷氨鏄珮搴︽偓鍋滄帶鍒跺嚱鏁�
//only in climb rate mode and landind mode. now we don't work on manual mode
void CtrlAltitude(void)
{
	static uint32_t tPrev=0;
	uint32_t now;
	float dt;	
	float manThr=0,alt=0,velZ=0;
	float spZMoveRate;
	float altSp=0;
	float posZVelSp=0;
	float altSpOffset,altSpOffsetMax=0;
	static float velZPrev=0;
	float posZErr=0,velZErr=0,valZErrD=0;
	float thrustXYSp[2]={0,0};	//roll pitch
	float thrustXYSpLen=0,thrustSpLen=0;
	float thrustXYMax=0;

	
	//get dt		//淇濊瘉dt杩愮畻涓嶈兘琚墦鏂紝淇濇寔鏇存柊锛屽惁鍒檇t杩囧ぇ锛岀Н鍒嗙垎婊°��
	if(tPrev==0){
		tPrev = micros();
		return;
	}else{
		now = micros();
		dt = (float)(now-tPrev) * 0.000001f;
		tPrev = now;
	}
	
	if(copterState.altMode==MANUAL || !motorState.enable)
		return;
	
	//--------------pos z ctrol---------------//
	//get current alt 
	alt = -nav.z;
	//get desired move rate from stick
	manThr = (float)rcState.data[THROTTLE] * 0.001f;
	spZMoveRate = -DBScaleLinear(manThr-0.5f, 0.5f, ALT_CTRL_Z_DB);	// scale to -1~1 . NED frame
	spZMoveRate = spZMoveRate * ALT_VEL_MAX;	// scale to vel min max

	//get alt setpoint in CLIMB rate mode
	altSp = -nav.z;						//only alt is not in ned frame.
	altSp -= spZMoveRate * dt;	 
	//limit alt setpoint
	altSpOffsetMax = ALT_VEL_MAX / alt_PID.P * 2.0f;
	altSpOffset = altSp - alt; 
	if(altSpOffset > altSpOffsetMax)		//or alt - alt > altSpOffsetMax
		altSp = alt + altSpOffsetMax;
	else if(altSpOffset < -altSpOffsetMax)
		altSp = alt - altSpOffsetMax;

	//闄愰珮
	if(isAltLimit)
	{
		if(altSp - altLand > ALT_LIMIT)
		{
			altSp = altLand + ALT_LIMIT;
			spZMoveRate = 0;
		}
	}
	
	// pid and feedforward control . in ned frame
	posZErr = -(altSp - alt);
	posZVelSp = posZErr * alt_PID.P + spZMoveRate * ALT_FEED_FORWARD;
	//consider landing mode
	if(copterState.altMode==LANDING)
		posZVelSp = LAND_SPEED;
	
	//--------------pos z vel ctrl -----------//
	if(zIntReset)		//tobe tested .  how to get hold throttle. give it a estimated value!!!!!!!!!!!
	{
		thrustZInt = HOVER_THRU * VCC_VOLTAGE / battery.val; //-manThr;		//650/1000 = 0.65
		zIntReset = 0;
	}
	velZ = nav.vz;	
	velZErr = posZVelSp - velZ;
	valZErrD = (spZMoveRate - velZ) * alt_PID.P - (velZ - velZPrev) / dt;	//spZMoveRate is from manual stick vel control
	velZPrev = velZ;
	
	thrustZSp = velZErr * alt_vel_PID.P + valZErrD * alt_vel_PID.D + thrustZInt;	//in ned frame. thrustZInt contains hover thrust
	
	//limit thrust min !!
	if(copterState.altMode!=LANDING)
	{
		if(-thrustZSp < THR_MIN)
		{
			thrustZSp = -THR_MIN; 
		} 				
	}

	//涓庡姩鍔涘垎閰嶇浉鍏�	testing
	satXY = 0;
	satZ = 0;
	
	thrustXYSp[0] = sinf(rcState.data[ROLL] * RAD_PI_REC);//鐩爣瑙掑害杞姞閫熷害
	thrustXYSp[1] = sinf(rcState.data[PITCH] * RAD_PI_REC); 	//褰掍竴鍖�
	thrustXYSpLen = sqrtf(thrustXYSp[0] * thrustXYSp[0] + thrustXYSp[1] * thrustXYSp[1]);
	//limit tilt max
	if(thrustXYSpLen >0.01f )
	{
		thrustXYMax = -thrustZSp * tanf(TILT_MAX);
		if(thrustXYSpLen > thrustXYMax)
		{
			float k = thrustXYMax / thrustXYSpLen;
			thrustXYSp[1] *= k;
			thrustXYSp[0] *= k;
			satXY = 1;
			thrustXYSpLen = sqrtf(thrustXYSp[0] * thrustXYSp[0] + thrustXYSp[1] * thrustXYSp[1]);
		}	
	}
	//limit max thrust!! 
	thrustSpLen = sqrtf(thrustXYSpLen * thrustXYSpLen + thrustZSp * thrustZSp);
	if(thrustSpLen > THR_MAX)
	{
		if(thrustZSp < 0.0f)	//going up
		{
			if (-thrustZSp > THR_MAX) 
			{
				/* thrust Z component is too large, limit it */
				thrustXYSp[0] = 0.0f;
				thrustXYSp[1] = 0.0f;
				thrustZSp = -THR_MAX;
				satXY = 1;
				satZ = 1;
			} 
			else 
			{
				float k = 0;
				/* preserve thrust Z component and lower XY, keeping altitude is more important than position */
				thrustXYMax = sqrtf(THR_MAX * THR_MAX- thrustZSp * thrustZSp);
				k = thrustXYMax / thrustXYSpLen;
				thrustXYSp[0] *= k;
				thrustXYSp[1] *= k;
				satXY = 1;
			}
		}
		else 
		{		//going down
			/* Z component is negative, going down, simply limit thrust vector */
			float k = THR_MAX / thrustSpLen;
			thrustZSp *= k;
			thrustXYSp[0] *= k;
			thrustXYSp[1] *= k;						
			satXY = 1;
			satZ = 1;
		}		
	} 
	rollSp = asinf(thrustXYSp[0]) * RAD_PI;
	pitchSp = asinf(thrustXYSp[1]) * RAD_PI;				
		
	// if saturation ,don't integral
	if(!satZ )//&& fabs(thrustZSp)<THR_MAX
	{
		thrustZInt += velZErr * alt_vel_PID.I * dt;		//
		if(thrustZInt > 0.0f)
			thrustZInt = 0.0f;
	}
}


//run after get rc cmd
void CtrlAttiAng(void)
{
	static uint32_t tPrev=0;
	uint32_t dt, now;
	float angTarget[3]={0};
	
	now = micros();
	dt = (tPrev>0)?(now-tPrev):0;
	tPrev = now;
		
	if(copterState.altMode==MANUAL)
	{
		angTarget[ROLL] = (float)(rcState.data[ROLL]);
		angTarget[PITCH] = (float)(rcState.data[PITCH]);
	}
	else
	{
		angTarget[ROLL] = rollSp;
		angTarget[PITCH] = pitchSp;
	}

	if(copterState.headMode)
	{
		float radDiff = -(imu.yaw - copterState.headYaw) * RAD_PI_REC; 
		float cosDiff = cosf(radDiff);
		float sinDiff = sinf(radDiff);
		float tarPitFree = angTarget[PITCH] * cosDiff + angTarget[ROLL] * sinDiff;
		angTarget[ROLL] = angTarget[ROLL] * cosDiff - angTarget[PITCH] * sinDiff;
		angTarget[PITCH] = tarPitFree;
	}
 
	PID_Postion_Cal(&pitch_angle_PID,angTarget[PITCH],imu.pitch,dt, copterState.flyUp);
	PID_Postion_Cal(&roll_angle_PID,angTarget[ROLL],imu.roll,dt, copterState.flyUp);
}



//run in 200Hz or 400Hz loop 
void CtrlAttiRate(void)
{
	static uint32_t tPrev=0; 
	uint32_t dt, now;
 	float yawRateTarget;
	
	now = micros();
	dt = (tPrev>0)?(now-tPrev):0;
	tPrev = now;
		
	yawRateTarget = -(float)rcState.data[YAW];

	PID_Postion_Cal(&pitch_rate_PID,pitch_angle_PID.Output,imu.gyro[PITCH]*RAD_PI, dt, copterState.flyUp);
	PID_Postion_Cal(&roll_rate_PID,roll_angle_PID.Output,imu.gyro[ROLL]*RAD_PI, dt, copterState.flyUp);
	PID_Postion_Cal(&yaw_rate_PID,yawRateTarget,imu.gyro[YAW]*RAD_PI, dt, copterState.flyUp);
		
	motorState.Pitch = pitch_rate_PID.Output;
	motorState.Roll  = roll_rate_PID.Output;
	motorState.Yaw   = yaw_rate_PID.Output; 
}


//鍑芥暟鍚嶏細CtrlMotor()
//杈撳叆锛氭棤
//杈撳嚭: 4涓數鏈虹殑PWM杈撳嚭
//鎻忚堪锛氳緭鍑篜WM锛屾帶鍒剁數鏈猴紝鏈嚱鏁颁細琚富寰幆涓�100Hz寰幆璋冪敤
void CtrlMotor(void)
{
//	static float thrAngCorrect;	//瀵瑰�炬枩鍋氫慨姝�
//	float  cosTilt = imu.accb[2] / CONSTANTS_ONE_G;
	
	if(copterState.altMode==MANUAL)
	{
//	DIF_ACC.Z =  imu.accb[2] - ONE_G;
		motorState.Thro = rcState.data[THROTTLE];
		// Thr = Thr/(cos) ;                             //瀵筞杞寸敤涓�娆¤礋鍙嶉鎺у埗
		//way1
		//thrAngCorrect = ANG_COR_COEF * (1-cosTilt) ;
		//Thr += thrAngCorrect;				//閲囩敤姘斿帇瀹氶珮鏃讹紝涓嶇敤姝や慨姝ｃ��

		//way2
		motorState.Thro = motorState.Thro/imu.DCMgb[2][2];

		//way3
		//thrAngCorrect=THR_HOLD_LEVEL * (1.0f/cosTilt - 1.0);
		//Thro += thrAngCorrect;
	}
	else
	{
		motorState.Thro = (-thrustZSp) * PWM_MAX;// /imu.DCMgb[2][2];  //鍊捐琛ュ伩鍚庢晥鏋滀笉閿欙紝鏈夋椂杩囩寷
		if(motorState.Thro > PWM_MAX)
			motorState.Thro = PWM_MAX;
//		motorState.Thro = rcState.data[THROTTLE];
	}
		
	//灏嗚緭鍑哄�艰瀺鍚堝埌鍥涗釜鐢垫満
	motorState.motor[0] = (int16_t)(motorState.Thro + motorState.Pitch - motorState.Roll + motorState.Yaw);    //M3
	motorState.motor[1] = (int16_t)(motorState.Thro + motorState.Pitch + motorState.Roll - motorState.Yaw);    //M1
	motorState.motor[2] = (int16_t)(motorState.Thro - motorState.Pitch + motorState.Roll + motorState.Yaw);    //M4
	motorState.motor[3] = (int16_t)(motorState.Thro - motorState.Pitch - motorState.Roll - motorState.Yaw);    //M2
	
	if(motorState.enable)
		Motor_pwm(motorState.motor);
	else                  
		Motor_off();
}


