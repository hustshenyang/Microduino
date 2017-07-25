#include "battery.h"
#include "driver/adc.h"

#define ADC1_BAT_CHANNEL	(0)

Battery_t battery;

void BAT_init()
{
    adc1_config_width(ADC_WIDTH_12Bit);
    adc1_config_channel_atten(ADC1_BAT_CHANNEL,ADC_ATTEN_11db);
}


void BatteryCheck(uint8_t _power)
{
	battery.val = (adc1_get_voltage(ADC1_BAT_CHANNEL)/4096.0) * VCC_VOLTAGE * (BAT_RATE + 1);
	if(_power)
		battery.alarm = (battery.val < BAT_TRESHOLD);
	else
		battery.alarm = ((battery.val < BAT_ALARM)&&(battery.val > BAT_CHARGE));	//浣庝簬3.7v 涓斿ぇ浜庡厖鐢垫娴嬬數鍘� BAT_CHG_VAL
		
	battery.charge = (battery.val < BAT_CHARGE);
}


