/*
 * motor_control.c
 *
 *  Created on: Jun 19, 2024
 *      Author: max
 */

#include "motor_control.h"
#include "stm32f4xx_hal_gpio.h"


//void Motor_Start(void)
//{
//    // enabling pwm
//	HAL_GPIO_WritePin(PWM1EN_GPIO_Port, PWM1EN_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(PWM2EN_GPIO_Port, PWM2EN_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(PWM3EN_GPIO_Port, PWM3EN_Pin, GPIO_PIN_SET);
//
//    // Start PWM signals
//    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
//    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
//    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
//}
//


// from tutorial

struct BLDC_Motor bldc;

#define ARR_TIM3_VALUE			100
#define BLDC_MOTOR_MAX_SPEED	100

bool was_button_pressed = false;


void bldc_motor_init(TIM_HandleTypeDef *_tim_pwm, TIM_HandleTypeDef *_tim_com)
{
	bldc.tim_pwm = _tim_pwm;
	bldc.tim_com = _tim_com;

	bldc.step_number = 1;
	bldc.speed_pulse = 0;
	bldc.dir = CW;
	bldc.speed_change_delay = 50;

	bldc_motor_Config_Channel_Init();

	__HAL_TIM_SET_AUTORELOAD(bldc.tim_com, ARR_TIM3_VALUE);

	HAL_TIM_Base_Start(bldc.tim_com);
	HAL_TIMEx_ConfigCommutationEvent_IT(bldc.tim_pwm, TIM_TS_ITR2, TIM_COMMUTATION_TRGI);

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
}



bool bldc_motor_set_speed(uint32_t speed)
{
	bool is_speed_changed = false;

	if(speed > BLDC_MOTOR_MAX_SPEED)
	{
		bldc.speed_pulse = BLDC_MOTOR_MAX_SPEED;
	}
	else
	{
		if (bldc.speed_pulse != speed){
			is_speed_changed = true;
		}
//		if (speed > bldc.speed_pulse){
//			while (bldc.speed_pulse <= speed){
//				bldc.speed_pulse++;
//				HAL_Delay(bldc.speed_change_delay);
//			}
//		}
//		else {
//			while (bldc.speed_pulse >= speed){
//				bldc.speed_pulse--;
//				HAL_Delay(bldc.speed_change_delay);
//			}
//		}
		bldc.speed_pulse = speed;
	}

	return is_speed_changed;
}

void MotorSetDir(direction dir){
	bldc.dir = dir;
}


void bldc_motor_Config_Channel_Init(void)
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

void bldc_motor_PWM_Config_Channel(uint32_t pulse, uint32_t channel)
{
    bldc.sConfigOC.OCMode = TIM_OCMODE_PWM1;
    bldc.sConfigOC.Pulse = pulse;
    HAL_TIM_PWM_ConfigChannel(bldc.tim_pwm, &bldc.sConfigOC, channel);

    HAL_TIM_PWM_Start(bldc.tim_pwm, channel);
//	HAL_TIMEx_PWMN_Start(bldc.tim_pwm, channel);
}

void bldc_motor_OC_Config_Channel(uint32_t mode, uint32_t channel)
{
    bldc.sConfigOC.OCMode = mode;
    HAL_TIM_OC_ConfigChannel(bldc.tim_pwm, &bldc.sConfigOC, channel);

    HAL_TIM_OC_Stop(bldc.tim_pwm, channel);
//	HAL_TIMEx_OCN_Start(bldc.tim_pwm, channel);
}

void MotorStart(int speed){
	int delay = 50;
	while(bldc.speed_pulse <= speed){
		bldc.speed_pulse++;
		HAL_Delay(delay);
	}
}

void MotorDirChange(){
	int old_speed = bldc.speed_pulse;
	int delay = 50;

	while(bldc.speed_pulse > 0){
		bldc.speed_pulse--;
		HAL_Delay(delay);
	}
	if (bldc.dir == CW){
		bldc.dir = CCW;
	} else {
		bldc.dir = CW;
	}
	MotorStart(old_speed);
}



void bldc_motor_set_pwm(uint16_t speedA, uint16_t speedB, uint16_t speedC)
{
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speedA);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, speedB);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, speedC);
}


void check_button_press() {
    static bool button_state = false;
    bool new_button_state = HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin);

    if (new_button_state && !button_state) {
        was_button_pressed = true;
    } else {
        was_button_pressed = false;
    }

    button_state = new_button_state;
}



void bldc_motor_six_step_algorithm(void)
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

        	bldc_motor_set_pwm(bldc.speed_pulse, 0, 0);

//            bldc_motor_PWM_Config_Channel(bldc.speed_pulse, TIM_CHANNEL_1);
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

        	bldc_motor_set_pwm(bldc.speed_pulse, 0, 0);

//			bldc_motor_PWM_Config_Channel(bldc.speed_pulse, TIM_CHANNEL_1);
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


        	bldc_motor_set_pwm(0, bldc.speed_pulse, 0);

//			bldc_motor_OC_Config_Channel(TIM_OCMODE_FORCED_ACTIVE, TIM_CHANNEL_1);
////			bldc_motor_PWM_Config_Channel(bldc.speed_pulse, TIM_CHANNEL_4);
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

//        	bldc_motor_set_pwm(0, bldc.speed_pulse, bldc.speed_pulse);
        	bldc_motor_set_pwm(0, bldc.speed_pulse, 0);

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


//        	bldc_motor_set_pwm(0, bldc.speed_pulse, bldc.speed_pulse);
        	bldc_motor_set_pwm(0, 0, bldc.speed_pulse);

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

        	bldc_motor_set_pwm(0, 0, bldc.speed_pulse);

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



