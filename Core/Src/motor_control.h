/*
 * motor_control.h
 *
 *  Created on: Jun 19, 2024
 *      Author: max
 */

#ifndef SRC_MOTOR_CONTROL_H_
#define SRC_MOTOR_CONTROL_H_

#include "main.h"

extern TIM_HandleTypeDef htim1;  // Declare htim1 as extern

//void Motor_Init(void);
void Motor_Start(void);

void Delay(volatile uint32_t delay);
void Spin_Motor(void);

void Motor_Stop(void);
void Motor_SetSpeed(uint16_t speed);


#endif /* SRC_MOTOR_CONTROL_H_ */
