/*
 * motor_control.h
 *
 *  Created on: Jun 19, 2024
 *      Author: max
 */

#ifndef SRC_MOTOR_CONTROL_H_
#define SRC_MOTOR_CONTROL_H_

#include "main.h"




typedef enum
{
	CCW = 0,
	CW = 1
}direction;

struct BLDC_Motor {
    uint8_t step_number;
    uint32_t speed_pulse;
    uint32_t dir;

	TIM_HandleTypeDef	*tim_com;
    TIM_HandleTypeDef	*tim_pwm;
    TIM_OC_InitTypeDef sConfigOC;
} ;


void bldc_motor_init(TIM_HandleTypeDef *_tim_pwm, TIM_HandleTypeDef *_tim_com);
void bldc_motor_Config_Channel_Init(void);
void bldc_motor_PWM_Config_Channel(uint32_t pulse, uint32_t channel);
void bldc_motor_OC_Config_Channel(uint32_t mode, uint32_t channel);
void bldc_motor_six_step_algorithm(void);

void bldc_motor_set_speed(uint32_t speed);


#endif /* SRC_MOTOR_CONTROL_H_ */
