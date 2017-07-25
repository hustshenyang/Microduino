
#include "led.h"
#include "driver/gpio.h"

LedState_t ledState;
LedByte_u ledByte;


void LedInit(void)
{
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = (1<<23) | (1<<18) | (1<<5);
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
}


//搴曞眰鏇存柊 锛�10Hz
void LedReflash(void)
{
	if(ledByte.bits.R)
		gpio_set_level(18, 0);
	else
		gpio_set_level(18, 1);
		
	if(ledByte.bits.G)
		gpio_set_level(23, 0);
	else
		gpio_set_level(23, 1);
		
	if(ledByte.bits.B)
		gpio_set_level(5, 0);
	else
		gpio_set_level(5, 1);
}

//浜嬩欢椹卞姩灞�
void LedFSM()
{	
	switch(ledState.event)
	{
		case E_READY:
			if(++ledState.cnt >= 6)		//0 1 2 in loop, 0 on ,else off
				ledState.cnt = 0;
			if(ledState.cnt < 2)
				ledByte.byte = LED_GREEN;
			else
				ledByte.byte = 0;
			break;
		
		case E_MOTOR_ENABLE:
			if(++ledState.cnt >= 3)		//0 1 2 in loop, 0 on ,else off
				ledState.cnt = 0;
			if(ledState.cnt < 2)
				ledByte.byte = LED_GREEN;
			else
				ledByte.byte = 0;
			break;			
			
		case E_CALI:
			ledByte.byte = LED_GREEN;
			break;		
			
		case E_CALI_FAIL:
			if(++ledState.cnt >= 6)
				ledState.cnt = 0;
			if(ledState.cnt < 2)
				ledByte.byte = LED_BLUE;
			else
				ledByte.byte = 0;
			break;
			
		case E_LOST_RC:
			if(++ledState.cnt >= 5)
				ledState.cnt = 0;
			ledByte.byte = 1<<(ledState.cnt/2);
			break;
			
		case E_AUTO_LANDED:
			ledByte.byte = LED_RED | LED_GREEN | LED_BLUE;
			break;

		case E_BAT_LOW:
			if(++ledState.cnt >= 5)		//0 1  in loop
				ledState.cnt = 0;
			if(ledState.cnt==0)
				ledByte.byte = LED_RED;
			else
				ledByte.byte = 0;
			break;
		
		case E_BatChg:
			ledByte.byte=0x00;
			break;		
	}
	LedReflash();
}


