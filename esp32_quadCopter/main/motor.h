#ifndef __MOTOR_H__
#define __MOTOR_H__

#include <stdint.h>

#define PWM_MAX			1000

void mGpio_init();
void Motor_init(void);
void Motor_deinit(void);
void Motor_off(void);
void Motor_pwm(int16_t *motor);
void PPM_init();
void PPM_deinit(void);
void PPMsend(uint8_t _ch, int16_t _val);

#endif


