#ifndef __CONTROL_H__
#define __CONTROL_H__

#include "stdint.h"
#include "stdbool.h"

//瀹氶珮閮ㄥ垎
#define LAND_SPEED				1.0f		//m/s^2
#define HOVER_THRU				-0.45f  //-0.5f  //鎮仠

#define THR_MIN					0.25f		//min thrust 锛屾牴鎹満閲嶅拰鏈�灏忛檷閫熻�屽畾锛岀敤浜庝笅闄嶉�熷害杩囧ぇ鏃讹紝娌归棬杩囧皬锛屽鑷村け琛°�傚啀澧炲姞fuzzy control 锛屽湪娌归棬灏忔椂鐢ㄦ洿澶х殑濮挎�佸弬鏁�
#define THR_MAX					1.0f		//max thrust
#define TILT_MAX				(RC_ANGLE_MAX * RAD_PI_REC)

#define ALT_FEED_FORWARD		0.7f
#define ALT_CTRL_Z_DB			0.1f
#define ALT_VEL_MAX 			4.0f
#define ALT_LIMIT				2.0f		//闄愰珮 3.5


enum {CLIMB_RATE=0,MANUAL,LANDING};

// PID缁撴瀯浣�
typedef struct
{
    float P;
    float I;
    float D;
    float Error;
    float PreError;
    float Integ;
    float iLimit;
    float Deriv;
    float Output;
 
}PID_Typedef;


typedef struct{
	int16_t motor[4];
	uint8_t enable;
	float Thro;
	float Roll;
	float Pitch;
	float Yaw;
	
}MotorState_t;


extern uint8_t zIntReset;
extern float altLand;
extern uint8_t isAltLimit;
extern float thrustZSp,thrustZInt;

extern MotorState_t motorState;

extern PID_Typedef pitch_angle_PID;	  //pitch瑙掑害鐜殑PID
extern PID_Typedef pitch_rate_PID;		//pitch瑙掗�熺巼鐜殑PID

extern PID_Typedef roll_angle_PID;    //roll瑙掑害鐜殑PID
extern PID_Typedef roll_rate_PID;     //roll瑙掗�熺巼鐜殑PID

extern PID_Typedef yaw_angle_PID;     //yaw瑙掑害鐜殑PID
extern PID_Typedef yaw_rate_PID;      //yaw鐨勮閫熺巼鐜殑PID

extern PID_Typedef alt_PID;
extern PID_Typedef alt_vel_PID;

void CtrlAttiAng(void);
void CtrlAttiRate(void);
void CtrlAltitude(void);
void CtrlMotor(void);

#endif


