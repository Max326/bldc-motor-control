#include "MotorControl.h"

#include "tim.h"

#include "math.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "Pot/Pot.h"

#define MAX_SPEED 1.0
#define MIN_SPEED 0.0

#define MAX_TORQUE 1.0
#define MIN_TORQUE 0.0

#define SPEED_STEP	0.01
#define TORQUE_STEP 0.01

static void MotorControlInit();
static void MotorSetPWM(uint16_t U, uint16_t V, uint16_t W);
static void MotorSinusoidalCalculate(float step, float power);
static float Map(float inValue, float inMin, float inMax, float outMin, float outMax);

static float sSpeed = 0.0;
static float sTorque = 0.5;
static float sDirection = 1.0;
static bool sPotMode = false;

void MotorControlTask() {
	MotorControlInit();

    while(1) {
    	if(sPotMode) {
    		sSpeed = Map((float)PotGetValue(), 0.0, 4095.0, 0.0, 1.0);
    		MotorSinusoidalCalculate(sDirection * sSpeed, sTorque);
    	}
    	else {
            MotorSinusoidalCalculate(sDirection * sSpeed, sTorque);
    	}

        osDelay(1);
    }
}

void MotorIncreaseSpeed() {
	sSpeed += SPEED_STEP;

	if(sSpeed > MAX_SPEED) {
		sSpeed = MAX_SPEED;
	}
}

void MotorDecreaseSpeed() {
	sSpeed -= SPEED_STEP;

	if(sSpeed < MIN_SPEED) {
		sSpeed = MIN_SPEED;
	}
}

void MotorChangeDirection() {
	sDirection = -sDirection;
}

void MotorIncreaseTorque() {
	sTorque += TORQUE_STEP;

	if(sTorque > MAX_TORQUE) {
		sTorque = MAX_TORQUE;
	}
}

void MotorDecreaseTorque() {
	sTorque -= TORQUE_STEP;

	if(sTorque < MIN_TORQUE) {
		sTorque = MIN_TORQUE;
	}
}

void MotorPotModeToggle() {
	sPotMode = !sPotMode;
}

float MotorGetSpeed() {
	return sSpeed;
}

float MotorGetTorque() {
	return sTorque;
}

float MotorGetDirection() {
	return sDirection;
}

bool MotorGetPotModeState() {
	return sPotMode;
}

static void MotorControlInit() {
    HAL_GPIO_WritePin(PWM1EN_GPIO_Port, PWM1EN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(PWM2EN_GPIO_Port, PWM2EN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(PWM3EN_GPIO_Port, PWM3EN_Pin, GPIO_PIN_SET);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}

static void MotorSinusoidalCalculate(float step, float power) {
    const uint16_t period = TIM1->ARR;
    static float phase = 0.0;

    float phaseA = 0.5f + 0.5f * sinf(phase);
    float phaseB = 0.5f + 0.5f * sinf(phase + 2.0f * M_PI / 3.0f);
    float phaseC = 0.5f + 0.5f * sinf(phase + 4.0f * M_PI / 3.0f);

    uint16_t pwmA = (uint16_t)(phaseA * power * period);
    uint16_t pwmB = (uint16_t)(phaseB * power * period);
    uint16_t pwmC = (uint16_t)(phaseC * power * period);

    MotorSetPWM(pwmA, pwmB, pwmC);

    phase += step;

    if (phase > 2.0f * M_PI) {
        phase -= 2.0f * M_PI;
    }
}

static void MotorSetPWM(uint16_t U, uint16_t V, uint16_t W) {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, U);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, V);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, W);
}

static float Map(float inValue, float inMin, float inMax, float outMin, float outMax) {
    return (inValue - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}
