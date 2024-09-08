/*
 * motor_control.c
 *
 *  Created on: Jun 19, 2024
 *      Author: max
 */

#include "motor_control.h"
#include "stm32f4xx_hal_gpio.h"



struct BLDC_Motor bldc;

#define ARR_TIM3_VALUE			100
#define BLDC_MOTOR_MAX_TORQUE	100
#define BLDC_MOTOR_MAX_SPEED	10000

bool wasButtonPressed = false;


void MotorInit(TIM_HandleTypeDef *_tim_pwm, TIM_HandleTypeDef *_tim_com)
{
	bldc.tim_pwm = _tim_pwm;
	bldc.tim_com = _tim_com;

	bldc.step_number = 1;
	bldc.step_size = 2;
	bldc.torque = 0;
	bldc.speed = 0;
	bldc.dir = CW;
	bldc.speed_change_delay = 50;

	MotorConfigChannelInit();

	__HAL_TIM_SET_AUTORELOAD(bldc.tim_com, ARR_TIM3_VALUE);

	HAL_TIM_Base_Start(bldc.tim_com);
	HAL_TIMEx_ConfigCommutationEvent_IT(bldc.tim_pwm, TIM_TS_ITR2, TIM_COMMUTATION_TRGI);

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

	MotorSetPWM(0, 0, 0);  // Ensure all PWM channels are low at the beginning
}



bool MotorSetTorque(uint32_t torque)
{
	bool isTorqueChanged = false;

	if(torque > BLDC_MOTOR_MAX_TORQUE)
	{
		bldc.torque = BLDC_MOTOR_MAX_TORQUE;
	}
	else
	{
		if (bldc.torque != torque){
			isTorqueChanged = true;
		}
//		if (torque > bldc.torque){
//			while (bldc.torque <= torque){
//				bldc.torque++;
//				HAL_Delay(bldc.torque_change_delay);
//			}
//		}
//		else {
//			while (bldc.torque >= torque){
//				bldc.torque--;
//				HAL_Delay(bldc.torque_change_delay);
//			}
//		}
		bldc.torque = torque;
	}

	return isTorqueChanged;
}

bool MotorSetSpeed(uint32_t newSpeed)
{
	bool isSpeedChanged = false;

	if(newSpeed > BLDC_MOTOR_MAX_SPEED)
	{
		bldc.speed = BLDC_MOTOR_MAX_SPEED;
	}
	else
	{
		if (bldc.speed != newSpeed){
			isSpeedChanged = true;
		}

//		bldc.speed = newSpeeed;

//		htim3.Init.Prescaler = newSpeed-1;
	}

	return isSpeedChanged;
}

void MotorSetDir(direction dir){
	bldc.dir = dir;
}


void MotorConfigChannelInit(void)
{
	bldc.sConfigOC.OCMode = TIM_OCMODE_PWM1;
	bldc.sConfigOC.Pulse = 0;
	bldc.sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	bldc.sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	bldc.sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	bldc.sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	bldc.sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

//	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC1|TIM_IT_CC3|TIM_IT_CC4);

}

void MotorPWMConfigChannel(uint32_t pulse, uint32_t channel)
{
    bldc.sConfigOC.OCMode = TIM_OCMODE_PWM1;
    bldc.sConfigOC.Pulse = pulse;
    HAL_TIM_PWM_ConfigChannel(bldc.tim_pwm, &bldc.sConfigOC, channel);

    HAL_TIM_PWM_Start(bldc.tim_pwm, channel);
//	HAL_TIMEx_PWMN_Start(bldc.tim_pwm, channel);
}

void MotorOCConfigChannel(uint32_t mode, uint32_t channel)
{
    bldc.sConfigOC.OCMode = mode;
    HAL_TIM_OC_ConfigChannel(bldc.tim_pwm, &bldc.sConfigOC, channel);

    HAL_TIM_OC_Stop(bldc.tim_pwm, channel);
//	HAL_TIMEx_OCN_Start(bldc.tim_pwm, channel);
}

void MotorStart(int torque){
	int delay = 50;
	while(bldc.torque <= torque){
		bldc.torque++;
		HAL_Delay(delay);
	}
}

void MotorDirChange(){
	int old_torque = bldc.torque;
	int delay = 50;

	while(bldc.torque > 0){
		bldc.torque--;
		HAL_Delay(delay);
	}
	if (bldc.dir == CW){
		bldc.dir = CCW;
	} else {
		bldc.dir = CW;
	}
	MotorStart(old_torque);
}



void MotorSetPWM(uint16_t torqueA, uint16_t torqueB, uint16_t torqueC)
{
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, torqueA);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, torqueB);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, torqueC);
}


void CheckButtonPress() {
    static bool button_state = false;
    bool new_button_state = HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin);

    if (new_button_state && !button_state) {
        wasButtonPressed = true;
    } else {
        wasButtonPressed = false;
    }

    button_state = new_button_state;
}

volatile void MotorSetStepSize(volatile float newStepSize) {
    if (bldc.step_size != newStepSize){
		bldc.step_size = newStepSize;
		printf("New step size set: %.2f\n", bldc.step_size);  // Debug output
    }

}

volatile void MotorSine(void) {
//    osMutexWait(stepSizeMutexHandle, osWaitForever);

    static float phase = 0.0;
    float torqueA, torqueB, torqueC;

    // Calculate sine wave values for each phase
    torqueA = bldc.torque * sinf(phase);
    torqueB = bldc.torque * sinf(phase + 2.0 * PI / 3.0);
    torqueC = bldc.torque * sinf(phase + 4.0 * PI / 3.0);

    // Convert float torque values to uint16_t for PWM output (Assuming signed center-aligned PWM)
    uint16_t pwmA = (uint16_t)((torqueA + bldc.torque) / (2 * bldc.torque) * PWM_MAX_VALUE);
    uint16_t pwmB = (uint16_t)((torqueB + bldc.torque) / (2 * bldc.torque) * PWM_MAX_VALUE);
    uint16_t pwmC = (uint16_t)((torqueC + bldc.torque) / (2 * bldc.torque) * PWM_MAX_VALUE);

//    printf("PWM A: %u, PWM B: %u, PWM C: %u\n", (unsigned int)(pwmA), (unsigned int)(pwmB), (unsigned int)(pwmC));

//    printf("Torque: %i, Step Size: %i\n", (int)(bldc.torque), (int)(bldc.step_size));

    // Set PWM values
    MotorSetPWM(pwmA, pwmB, pwmC);

    // Increment phase
    phase += bldc.step_size;

    // Keep the phase within the range of 0 to 2*PI
    if (phase >= 2.0 * PI) {
        phase -= 2.0 * PI;
    }

//    osMutexRelease(stepSizeMutexHandle);

    HAL_GPIO_WritePin(PWM1EN_GPIO_Port, PWM1EN_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(PWM2EN_GPIO_Port, PWM2EN_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(PWM3EN_GPIO_Port, PWM3EN_Pin, GPIO_PIN_SET);
}



void MotorSixStepAlgorithm(void)
{
    switch (bldc.step_number)
    {
        case 1:
        	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, SET);
        	HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, RESET);
        	HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, RESET);

        	HAL_GPIO_WritePin(PWM1EN_GPIO_Port, PWM1EN_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(PWM2EN_GPIO_Port, PWM2EN_Pin, GPIO_PIN_SET);
        	HAL_GPIO_WritePin(PWM3EN_GPIO_Port, PWM3EN_Pin, GPIO_PIN_RESET);

        	MotorSetPWM(bldc.torque, 0, 0);
//        	HAL_Delay(0.025);


//            bldc_motor_PWM_Config_Channel(bldc.torque, TIM_CHANNEL_1);
//            bldc_motor_OC_Config_Channel(TIM_OCMODE_FORCED_ACTIVE, TIM_CHANNEL_1);
//            bldc_motor_OC_Config_Channel(TIM_OCMODE_FORCED_ACTIVE, TIM_CHANNEL_4);
//            bldc_motor_OC_Config_Channel(TIM_OCMODE_FORCED_INACTIVE, TIM_CHANNEL_3);
            break;
        case 2:
        	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, SET);
			HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, RESET);
			HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, RESET);

			HAL_GPIO_WritePin(PWM1EN_GPIO_Port, PWM1EN_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(PWM2EN_GPIO_Port, PWM2EN_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(PWM3EN_GPIO_Port, PWM3EN_Pin, GPIO_PIN_SET);

        	MotorSetPWM(bldc.torque, 0, 0);
//        	HAL_Delay(0.025);

//			bldc_motor_PWM_Config_Channel(bldc.torque, TIM_CHANNEL_1);
//			bldc_motor_OC_Config_Channel(TIM_OCMODE_FORCED_ACTIVE, TIM_CHANNEL_1);
//			bldc_motor_OC_Config_Channel(TIM_OCMODE_FORCED_INACTIVE, TIM_CHANNEL_4);
//			bldc_motor_OC_Config_Channel(TIM_OCMODE_FORCED_INACTIVE, TIM_CHANNEL_3);

            break;
        case 3:
        	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, RESET);
			HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, SET);
			HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, RESET);

			HAL_GPIO_WritePin(PWM1EN_GPIO_Port, PWM1EN_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(PWM2EN_GPIO_Port, PWM2EN_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(PWM3EN_GPIO_Port, PWM3EN_Pin, GPIO_PIN_SET);


        	MotorSetPWM(0, bldc.torque, 0);
//        	HAL_Delay(0.025);

//			bldc_motor_OC_Config_Channel(TIM_OCMODE_FORCED_ACTIVE, TIM_CHANNEL_1);
////			bldc_motor_PWM_Config_Channel(bldc.torque, TIM_CHANNEL_4);
//			bldc_motor_OC_Config_Channel(TIM_OCMODE_FORCED_INACTIVE, TIM_CHANNEL_4);
//			bldc_motor_OC_Config_Channel(TIM_OCMODE_FORCED_ACTIVE, TIM_CHANNEL_3);


            break;
        case 4:
        	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, RESET);
			HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, SET);
			HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, SET);

			HAL_GPIO_WritePin(PWM1EN_GPIO_Port, PWM1EN_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(PWM2EN_GPIO_Port, PWM2EN_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(PWM3EN_GPIO_Port, PWM3EN_Pin, GPIO_PIN_RESET);

//        	bldc_motor_set_pwm(0, bldc.torque, bldc.torque);
        	MotorSetPWM(0, bldc.torque, 0);
//        	HAL_Delay(0.025);

//			bldc_motor_OC_Config_Channel(TIM_OCMODE_FORCED_INACTIVE, TIM_CHANNEL_1);
//			bldc_motor_OC_Config_Channel(TIM_OCMODE_FORCED_INACTIVE, TIM_CHANNEL_4);
//			bldc_motor_OC_Config_Channel(TIM_OCMODE_FORCED_ACTIVE, TIM_CHANNEL_3);

            break;
        case 5:
        	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, RESET);
			HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, SET);
			HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, SET);

			HAL_GPIO_WritePin(PWM1EN_GPIO_Port, PWM1EN_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(PWM2EN_GPIO_Port, PWM2EN_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(PWM3EN_GPIO_Port, PWM3EN_Pin, GPIO_PIN_SET);


//        	bldc_motor_set_pwm(0, bldc.torque, bldc.torque);
        	MotorSetPWM(0, 0, bldc.torque);
//        	HAL_Delay(0.025);

//			bldc_motor_OC_Config_Channel(TIM_OCMODE_FORCED_INACTIVE, TIM_CHANNEL_1);
//			bldc_motor_OC_Config_Channel(TIM_OCMODE_FORCED_ACTIVE, TIM_CHANNEL_4);
//			bldc_motor_OC_Config_Channel(TIM_OCMODE_FORCED_ACTIVE, TIM_CHANNEL_3);


            break;
        case 6:
        	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, RESET);
			HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, RESET);
			HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, SET);

			HAL_GPIO_WritePin(PWM1EN_GPIO_Port, PWM1EN_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(PWM2EN_GPIO_Port, PWM2EN_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(PWM3EN_GPIO_Port, PWM3EN_Pin, GPIO_PIN_SET);

        	MotorSetPWM(0, 0, bldc.torque);
//        	HAL_Delay(0.025);

//			bldc_motor_OC_Config_Channel(TIM_OCMODE_FORCED_INACTIVE, TIM_CHANNEL_1);
//			bldc_motor_OC_Config_Channel(TIM_OCMODE_FORCED_ACTIVE, TIM_CHANNEL_4);
//			bldc_motor_OC_Config_Channel(TIM_OCMODE_FORCED_INACTIVE, TIM_CHANNEL_3);
            break;
    }

    if (bldc.dir == 1)  // CW direction
    {
        bldc.step_number++;
        if (bldc.step_number > 6)
            bldc.step_number = 1;
    }
    else if (bldc.dir == 0)  // CCW direction
    {
        bldc.step_number--;
        if (bldc.step_number < 1)
            bldc.step_number = 6;
    }
}



