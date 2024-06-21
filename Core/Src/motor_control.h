/*
 * motor_control.h
 *
 *  Created on: Jun 19, 2024
 *      Author: max
 */

#ifndef SRC_MOTOR_CONTROL_H_
#define SRC_MOTOR_CONTROL_H_

#include "main.h"

//extern TIM_HandleTypeDef htim1;  // Declare htim1 as extern
//
////void Motor_Init(void);
//void Motor_Start(void);
//
//void Delay(volatile uint32_t delay);
//void Spin_Motor(void);
//
//void Motor_Stop(void);
//void Motor_SetSpeed(uint16_t speed);
//
//

extern TIM_HandleTypeDef htim1;  // Declare htim1 as extern

typedef struct {
    uint8_t step_number;
    uint32_t speed_pulse;
    uint32_t dir;
    TIM_HandleTypeDef* tim_pwm;
    TIM_OC_InitTypeDef sConfigOC;
} BLDC_Motor;

extern BLDC_Motor bldc;

void Motor_Start(void);
void Delay(volatile uint32_t delay);
void Spin_Motor(void);
void Motor_Stop(void);
void Motor_SetSpeed(uint16_t speed);
void bldc_motor_PWM_Config_Channel(uint32_t pulse, uint32_t channel);
void bldc_motor_OC_Config_Channel(uint32_t mode, uint32_t channel);
void bldc_motor_six_step_algorithm(void);


#endif /* SRC_MOTOR_CONTROL_H_ */
