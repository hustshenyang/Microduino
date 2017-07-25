#ifndef __BATTERY_H__
#define __BATTERY_H__

#include "stdint.h"

#define VCC_VOLTAGE			3.3
#define BAT_RATE			(51.0/33.0)

#define BAT_ALARM  			3.3	  //on ground
#define BAT_TRESHOLD		(BAT_ALARM-0.75)
#define BAT_CHARGE    		1.0	  // charge battery val.  unit :v

typedef struct
{    
	float val;            	//鐢靛帇瀹為檯鍊�
	uint8_t alarm;									//鎶ヨ浣�
	uint8_t charge;							//鍏呯數鐘舵��
}Battery_t;

extern Battery_t battery;

void BAT_init(void);
void BatteryCheck(uint8_t _power);

#endif
                
        



