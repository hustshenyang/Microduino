#ifndef __LED_H__
#define __LED_H__

#include <stdint.h>

#define LED_NUM 	3

#define LED_RED	     0x01
#define LED_GREEN    0x02
#define LED_BLUE     0x04
//

enum RGB_EVENT{
	E_BatChg=0,
	E_READY,
	E_MOTOR_ENABLE,
	E_CALI,
	E_CALI_FAIL,
	E_BAT_LOW,
	E_LOST_RC,
	E_AUTO_LANDED

};


typedef union{
	uint8_t byte;
	struct {
		uint8_t R	:1;
		uint8_t G	:1;
		uint8_t B	:1;
		uint8_t reserved	:5;
	}bits;
}LedByte_u;


typedef struct{
	uint8_t event;
	uint8_t cnt;

}LedState_t;

extern LedState_t ledState;

void LedInit(void);   //Led鍒濆鍖栧嚱鏁板閮ㄥ０鏄�
void LedFSM();

#endif

