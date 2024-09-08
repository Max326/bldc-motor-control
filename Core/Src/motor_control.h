/*
 * motor_control.h
 *
 *  Created on: Jun 19, 2024
 *      Author: max
 */

#ifndef SRC_MOTOR_CONTROL_H_
#define SRC_MOTOR_CONTROL_H_

#include "main.h"

#include <stdbool.h>
#include "cmsis_os.h"
#include <math.h>
#include <stdio.h>



//extern TIM_HandleTypeDef htim1;  // Declare htim1 as extern
//
////void Motor_Init(void);
//void Motor_Start(void);
//
//void Delay(volatile uint32_t delay);
//void Spin_Motor(void);
//
//void Motor_Stop(void);
//
//
#define PWM_MAX_VALUE 4096
#define PI 3.14159265358979323846
#define CONTROL_LOOP_FREQUENCY 1000

extern TIM_HandleTypeDef htim1;  // Declare htim1 as extern
extern TIM_HandleTypeDef htim3;  // Declare htim3 as extern


typedef enum
{
	CCW = 0,
	CW = 1
}direction;

struct BLDC_Motor {
    uint8_t step_number;
    volatile float step_size;
    volatile uint32_t torque;
    uint32_t dir;
    uint32_t speed;
    uint32_t speed_change_delay;

	TIM_HandleTypeDef	*tim_com;
    TIM_HandleTypeDef	*tim_pwm;
    TIM_OC_InitTypeDef sConfigOC;
} ;


void MotorInit(TIM_HandleTypeDef *_tim_pwm, TIM_HandleTypeDef *_tim_com);
void MotorConfigChannelInit(void);
void MotorPWMConfigChannel(uint32_t pulse, uint32_t channel);
void MotorOCConfigChannel(uint32_t mode, uint32_t channel);
void MotorSixStepAlgorithm(void);
void MotorSine(void);


extern bool wasButtonPressed;
void CheckButtonPress();

void MotorSetPWM(uint16_t torqueA, uint16_t torqueB, uint16_t torqueC);
bool MotorSetTorque(uint32_t torque);
void MotorStart(int torque);
void MotorDirChange();
void MotorSetDir(direction dir);
bool MotorSetSpeed(uint32_t newSpeed);
void MotorSetStepSize(float newStepSize);

extern osMutexId_t stepSizeMutexHandle;
extern osMutexId_t torqueMutexHandle;


#endif /* SRC_MOTOR_CONTROL_H_ */
