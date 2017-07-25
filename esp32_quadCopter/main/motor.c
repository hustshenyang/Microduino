
#include "motor.h"
#include "driver/ledc.h"

#define PWM_FREQUENCE   	10000
#define PWM_TIMER          	LEDC_TIMER_0
#define PWM_MODE           	LEDC_HIGH_SPEED_MODE
#define PWM_CH0_GPIO       	(25)
#define PWM_CH0_CHANNEL    	LEDC_CHANNEL_0
#define PWM_CH1_GPIO       	(27)
#define PWM_CH1_CHANNEL    	LEDC_CHANNEL_1
#define PWM_CH2_GPIO       	(12)
#define PWM_CH2_CHANNEL    	LEDC_CHANNEL_2
#define PWM_CH3_GPIO       	(4)
#define PWM_CH3_CHANNEL    	LEDC_CHANNEL_3

#define PWM_CH_NUM		(4)

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

typedef struct {
    int channel;
    int io;
    int mode;
    int timer_idx;
} pwm_info_t;

static pwm_info_t pwm_ch[PWM_CH_NUM];

void Motor_pwm(int16_t *motor)
{    
	uint8_t ch;
	for(ch = 0; ch < PWM_CH_NUM; ch++){
		ledc_set_duty(pwm_ch[ch].mode, pwm_ch[ch].channel, (uint32_t)constrain(motor[ch], 0, PWM_MAX));
		ledc_update_duty(pwm_ch[ch].mode, pwm_ch[ch].channel);
	}
}


void Motor_off()
{
	uint8_t ch;
	for(ch = 0; ch < PWM_CH_NUM; ch++){
		ledc_set_duty(pwm_ch[ch].mode, pwm_ch[ch].channel, 0);
		ledc_update_duty(pwm_ch[ch].mode, pwm_ch[ch].channel);
	}
}


void Motor_init(void)
{
	uint8_t ch;
	ledc_timer_config_t pwm_timer;
	ledc_channel_config_t pwm_channel;

	pwm_timer.bit_num = LEDC_TIMER_10_BIT; //set timer counter bit number
	pwm_timer.freq_hz = PWM_FREQUENCE;     //set frequency of pwm
	pwm_timer.speed_mode = PWM_MODE;   //timer mode,
	pwm_timer.timer_num = PWM_TIMER;   //timer index

    ledc_timer_config(&pwm_timer);
//    ledc_timer_resume(PWM_MODE, PWM_TIMER);

	//set the configuration pwm0
    pwm_ch[0].channel = PWM_CH0_CHANNEL;		//set LEDC channel 1
    pwm_ch[0].io = PWM_CH0_GPIO;		//GPIO number
	pwm_ch[0].mode = PWM_MODE;
	pwm_ch[0].timer_idx = PWM_TIMER;

	//set the configuration pwm1
    pwm_ch[1].channel = PWM_CH1_CHANNEL;		//set LEDC channel 1
    pwm_ch[1].io = PWM_CH1_GPIO;		//GPIO number
	pwm_ch[1].mode = PWM_MODE;
	pwm_ch[1].timer_idx = PWM_TIMER;

	//set the configuration pwm2
    pwm_ch[2].channel = PWM_CH2_CHANNEL;		//set LEDC channel 1
    pwm_ch[2].io = PWM_CH2_GPIO;		//GPIO number
	pwm_ch[2].mode = PWM_MODE;
	pwm_ch[2].timer_idx = PWM_TIMER;

	//set the configuration pwm3
    pwm_ch[3].channel = PWM_CH3_CHANNEL;		//set LEDC channel 1
    pwm_ch[3].io = PWM_CH3_GPIO;		//GPIO number
	pwm_ch[3].mode = PWM_MODE;
	pwm_ch[3].timer_idx = PWM_TIMER;


    for (ch = 0; ch < PWM_CH_NUM; ch++) {
    	pwm_channel.channel = pwm_ch[ch].channel,
    	pwm_channel.duty = 0,
		pwm_channel.gpio_num = pwm_ch[ch].io,
		pwm_channel.intr_type = LEDC_INTR_DISABLE,
		pwm_channel.speed_mode = pwm_ch[ch].mode,
		pwm_channel.timer_sel = pwm_ch[ch].timer_idx,
        ledc_channel_config(&pwm_channel);
    }
}


void Motor_deinit(void)
{
	uint8_t ch;
    for (ch = 0; ch < PWM_CH_NUM; ch++) {
        ledc_stop(pwm_ch[ch].mode, pwm_ch[ch].channel, 0);
    }
    ledc_timer_pause(PWM_MODE, PWM_TIMER);
}

