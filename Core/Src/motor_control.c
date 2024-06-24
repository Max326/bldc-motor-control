/*
 * motor_control.c
 *
 *  Created on: Jun 19, 2024
 *      Author: max
 */

#include "motor_control.h"
#include "stm32f4xx_hal_gpio.h"

#define ARR_TIM3_VALUE			100
#define BLDC_MOTOR_MAX_SPEED	100


struct BLDC_Motor bldc;


void bldc_motor_init(TIM_HandleTypeDef *_tim_pwm, TIM_HandleTypeDef *_tim_com)
{

	HAL_GPIO_WritePin(PWM1EN_GPIO_Port, PWM1EN_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(PWM2EN_GPIO_Port, PWM2EN_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(PWM3EN_GPIO_Port, PWM3EN_Pin, GPIO_PIN_SET);

	bldc.tim_pwm = _tim_pwm;
	bldc.tim_com = _tim_com;

	bldc.step_number = 1;
	bldc.speed_pulse = 0;
	bldc.dir = CW;

	bldc_motor_Config_Channel_Init();

	__HAL_TIM_SET_AUTORELOAD(bldc.tim_com, ARR_TIM3_VALUE);

	HAL_TIM_Base_Start(bldc.tim_com);
	HAL_TIMEx_ConfigCommutationEvent_IT(bldc.tim_pwm, TIM_TS_ITR2, TIM_COMMUTATION_TRGI);

}

void bldc_motor_six_step_algorithm(void)
{
    switch (bldc.step_number)
    {
        case 1:
            bldc_motor_PWM_Config_Channel(bldc.speed_pulse, TIM_CHANNEL_1);
            //bldc_motor_PWM_Stop_Channel(TIM_CHANNEL_2);
            //bldc_motor_PWM_Stop_Channel(TIM_CHANNEL_3);
            HAL_GPIO_WritePin(PWM1EN_GPIO_Port, PWM1EN_Pin, GPIO_PIN_SET); // U+
            HAL_GPIO_WritePin(PWM2EN_GPIO_Port, PWM2EN_Pin, GPIO_PIN_RESET); // V-
            HAL_GPIO_WritePin(PWM3EN_GPIO_Port, PWM3EN_Pin, GPIO_PIN_RESET); // W floating
            break;

        case 2:
            bldc_motor_PWM_Config_Channel(bldc.speed_pulse, TIM_CHANNEL_1);
            //bldc_motor_PWM_Stop_Channel(TIM_CHANNEL_2);
            //bldc_motor_PWM_Stop_Channel(TIM_CHANNEL_3);
            HAL_GPIO_WritePin(PWM1EN_GPIO_Port, PWM1EN_Pin, GPIO_PIN_SET); // U+
            HAL_GPIO_WritePin(PWM2EN_GPIO_Port, PWM2EN_Pin, GPIO_PIN_RESET); // V floating
            HAL_GPIO_WritePin(PWM3EN_GPIO_Port, PWM3EN_Pin, GPIO_PIN_SET); // W-
            break;

        case 3:
            //bldc_motor_PWM_Stop_Channel(TIM_CHANNEL_1);
            bldc_motor_PWM_Config_Channel(bldc.speed_pulse, TIM_CHANNEL_2);
            //bldc_motor_PWM_Stop_Channel(TIM_CHANNEL_3);
            HAL_GPIO_WritePin(PWM1EN_GPIO_Port, PWM1EN_Pin, GPIO_PIN_RESET); // U floating
            HAL_GPIO_WritePin(PWM2EN_GPIO_Port, PWM2EN_Pin, GPIO_PIN_SET); // V+
            HAL_GPIO_WritePin(PWM3EN_GPIO_Port, PWM3EN_Pin, GPIO_PIN_SET); // W-
            break;

        case 4:
            //bldc_motor_PWM_Stop_Channel(TIM_CHANNEL_1);
            bldc_motor_PWM_Config_Channel(bldc.speed_pulse, TIM_CHANNEL_2);
            //bldc_motor_PWM_Stop_Channel(TIM_CHANNEL_3);
            HAL_GPIO_WritePin(PWM1EN_GPIO_Port, PWM1EN_Pin, GPIO_PIN_SET); // U-
            HAL_GPIO_WritePin(PWM2EN_GPIO_Port, PWM2EN_Pin, GPIO_PIN_SET); // V+
            HAL_GPIO_WritePin(PWM3EN_GPIO_Port, PWM3EN_Pin, GPIO_PIN_RESET); // W floating
            break;

        case 5:
            //bldc_motor_PWM_Stop_Channel(TIM_CHANNEL_1);
            //bldc_motor_PWM_Stop_Channel(TIM_CHANNEL_2);
            bldc_motor_PWM_Config_Channel(bldc.speed_pulse, TIM_CHANNEL_3);
            HAL_GPIO_WritePin(PWM1EN_GPIO_Port, PWM1EN_Pin, GPIO_PIN_SET); // U-
            HAL_GPIO_WritePin(PWM2EN_GPIO_Port, PWM2EN_Pin, GPIO_PIN_RESET); // V floating
            HAL_GPIO_WritePin(PWM3EN_GPIO_Port, PWM3EN_Pin, GPIO_PIN_SET); // W+
            break;

        case 6:
            //bldc_motor_PWM_Stop_Channel(TIM_CHANNEL_1);
           // bldc_motor_PWM_Stop_Channel(TIM_CHANNEL_2);
            bldc_motor_PWM_Config_Channel(bldc.speed_pulse, TIM_CHANNEL_3);
            HAL_GPIO_WritePin(PWM1EN_GPIO_Port, PWM1EN_Pin, GPIO_PIN_RESET); // U floating
            HAL_GPIO_WritePin(PWM2EN_GPIO_Port, PWM2EN_Pin, GPIO_PIN_SET); // V-
            HAL_GPIO_WritePin(PWM3EN_GPIO_Port, PWM3EN_Pin, GPIO_PIN_SET); // W+
            break;
    }

    bldc.step_number++;
    if (bldc.step_number > 6) {
        bldc.step_number = 1;
    }
}

void bldc_motor_PWM_Stop_Channel(uint32_t channel)
{
    HAL_TIM_SET_COMPARE(bldc.tim_pwm, channel, 0);
    HAL_TIM_PWM_Stop(bldc.tim_pwm, channel);
}

void bldc_motor_set_speed(uint32_t speed)
{

	if(speed > BLDC_MOTOR_MAX_SPEED)
	{
		bldc.speed_pulse = BLDC_MOTOR_MAX_SPEED;
	}
	else
	{
		bldc.speed_pulse = speed;
	}

	//bldc.dir = dir;

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
}

void bldc_motor_PWM_Config_Channel(uint32_t pulse, uint32_t channel)
{
    bldc.sConfigOC.OCMode = TIM_OCMODE_PWM1;
    bldc.sConfigOC.Pulse = pulse;
    HAL_TIM_PWM_ConfigChannel(bldc.tim_pwm, &bldc.sConfigOC, channel);

    HAL_TIM_PWM_Start(bldc.tim_pwm, channel);
	HAL_TIMEx_PWMN_Start(bldc.tim_pwm, channel);
}

void bldc_motor_OC_Config_Channel(uint32_t mode, uint32_t channel)
{
    bldc.sConfigOC.OCMode = mode;
    HAL_TIM_OC_ConfigChannel(bldc.tim_pwm, &bldc.sConfigOC, channel);

    HAL_TIM_OC_Stop(bldc.tim_pwm, channel);
	HAL_TIMEx_OCN_Start(bldc.tim_pwm, channel);
}




