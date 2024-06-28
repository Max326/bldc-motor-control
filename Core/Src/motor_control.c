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
//void Delay(volatile uint32_t delay) {
//    while (delay--) {
//        __asm("nop");
//    }
//}
//
//
//void Spin_Motor(void) {
//    uint16_t speed = 50; // Example speed (duty cycle)
//    while (1) {
//        // Commutation state 1: A+ B- Cx
//        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
//        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
//        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, speed);
//        HAL_Delay(1);
//
//        // Commutation state 2: A+ Bx C-
//        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
//        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed);
//        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
//        HAL_Delay(1);
//
//        // Commutation state 3: Ax B+ C-
//        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
//        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed);
//        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, speed);
//        HAL_Delay(1);
//
//        // Commutation state 4: A- B+ Cx
//        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
//        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed);
//        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
//        HAL_Delay(1);
//
//        // Commutation state 5: A- Bx C+
//        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
//        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
//        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, speed);
//        HAL_Delay(1);
//
//        // Commutation state 6: Ax B- C+
//        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
//        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed);
//        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, speed);
//        HAL_Delay(1);
//    }
//}
//
//
//void Motor_Stop(void)
//{
//    // Stop PWM signals
//    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
//    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
//    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
//}
//
//void Motor_SetSpeed(uint16_t speed)
//{
//    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
//    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed);
//    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, speed);
//}


// from tutorial

struct BLDC_Motor bldc;

#define ARR_TIM3_VALUE			100
#define BLDC_MOTOR_MAX_SPEED	100


//void Motor_Start(void)
//{
//    // Enabling PWM
//    HAL_GPIO_WritePin(PWM1EN_GPIO_Port, PWM1EN_Pin, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(PWM2EN_GPIO_Port, PWM2EN_Pin, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(PWM3EN_GPIO_Port, PWM3EN_Pin, GPIO_PIN_SET);
//
//    // Initialize BLDC motor control parameters
//    bldc.step_number = 1;
//    bldc.speed_pulse = 0;  // Example initial speed (duty cycle)
//    bldc.dir = 1;  // Set direction to CW
//
////    bldc.tim_pwm = &htim1;
//	bldc.tim_pwm = _tim_pwm;
//	bldc.tim_com = _tim_com;
//
//    // Start PWM signals
//    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
//    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
//    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
//
//    //fucking around
////    HAL_GPIO_WritePin(PH1_GPIO_Port, PH1_Pin, GPIO_PIN_SET);
////	HAL_GPIO_WritePin(PH2_GPIO_Port, PH2_Pin, GPIO_PIN_SET);
////	HAL_GPIO_WritePin(PWM3EN_GPIO_Port, PWM3EN_Pin, GPIO_PIN_SET);
//}

void bldc_motor_init(TIM_HandleTypeDef *_tim_pwm, TIM_HandleTypeDef *_tim_com)
{
	bldc.tim_pwm = _tim_pwm;
	bldc.tim_com = _tim_com;

	bldc.step_number = 1;
	bldc.speed_pulse = 0;
	bldc.dir = CW;

	bldc_motor_Config_Channel_Init();

	__HAL_TIM_SET_AUTORELOAD(bldc.tim_com, ARR_TIM3_VALUE);

	HAL_TIM_Base_Start(bldc.tim_com);
	HAL_TIMEx_ConfigCommutationEvent_IT(bldc.tim_pwm, TIM_TS_ITR2, TIM_COMMUTATION_TRGI);


	// ???
//	bldc_motor_PWM_Config_Channel(bldc.speed_pulse, TIM_CHANNEL_1);
//	bldc_motor_PWM_Config_Channel(bldc.speed_pulse, TIM_CHANNEL_4);
//	bldc_motor_PWM_Config_Channel(bldc.speed_pulse, TIM_CHANNEL_3);

}



void bldc_motor_set_speed(uint32_t speed, direction dir)
{
	if(speed > BLDC_MOTOR_MAX_SPEED)
	{
		bldc.speed_pulse = BLDC_MOTOR_MAX_SPEED;
	}
	else
	{
		bldc.speed_pulse = speed;
	}

	bldc.dir = dir;
}



//void Spin_Motor(void) {
//    while (1) {
//        bldc_motor_six_step_algorithm();
////        HAL_Delay(2); // Adjust the delay for the desired motor speed
//
//    }
//}
//
//void Motor_Stop(void)
//{
//    // Stop PWM signals
//    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
//    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
//    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
//}

//void Motor_SetSpeed(uint16_t speed)
//{
//    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
//    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed);
//    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, speed);
//}


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


//            bldc_motor_PWM_Config_Channel(bldc.speed_pulse, TIM_CHANNEL_1);
            bldc_motor_OC_Config_Channel(TIM_OCMODE_FORCED_ACTIVE, TIM_CHANNEL_1);
            bldc_motor_OC_Config_Channel(TIM_OCMODE_FORCED_ACTIVE, TIM_CHANNEL_4);
            bldc_motor_OC_Config_Channel(TIM_OCMODE_FORCED_INACTIVE, TIM_CHANNEL_3);
            break;
        case 2:
        	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, SET);
			HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, RESET);
			HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, RESET);

			HAL_GPIO_WritePin(PWM1EN_GPIO_Port, PWM1EN_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(PWM2EN_GPIO_Port, PWM2EN_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(PWM3EN_GPIO_Port, PWM3EN_Pin, GPIO_PIN_SET);

//			bldc_motor_PWM_Config_Channel(bldc.speed_pulse, TIM_CHANNEL_1);
			bldc_motor_OC_Config_Channel(TIM_OCMODE_FORCED_ACTIVE, TIM_CHANNEL_1);
			bldc_motor_OC_Config_Channel(TIM_OCMODE_FORCED_INACTIVE, TIM_CHANNEL_4);
			bldc_motor_OC_Config_Channel(TIM_OCMODE_FORCED_INACTIVE, TIM_CHANNEL_3);
//            bldc_motor_PWM_Config_Channel(bldc.speed_pulse, TIM_CHANNEL_1);
//            bldc_motor_OC_Config_Channel(TIM_OCMODE_FORCED_INACTIVE, TIM_CHANNEL_4);
//            bldc_motor_OC_Config_Channel(TIM_OCMODE_FORCED_ACTIVE, TIM_CHANNEL_3);
            break;
        case 3:
        	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, RESET);
			HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, SET);
			HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, RESET);

			HAL_GPIO_WritePin(PWM1EN_GPIO_Port, PWM1EN_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(PWM2EN_GPIO_Port, PWM2EN_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(PWM3EN_GPIO_Port, PWM3EN_Pin, GPIO_PIN_SET);


			bldc_motor_OC_Config_Channel(TIM_OCMODE_FORCED_ACTIVE, TIM_CHANNEL_1);
//			bldc_motor_PWM_Config_Channel(bldc.speed_pulse, TIM_CHANNEL_4);
			bldc_motor_OC_Config_Channel(TIM_OCMODE_FORCED_INACTIVE, TIM_CHANNEL_4);
			bldc_motor_OC_Config_Channel(TIM_OCMODE_FORCED_ACTIVE, TIM_CHANNEL_3);
//            bldc_motor_OC_Config_Channel(TIM_OCMODE_FORCED_INACTIVE, TIM_CHANNEL_1);
//            bldc_motor_PWM_Config_Channel(bldc.speed_pulse, TIM_CHANNEL_4);
//            bldc_motor_OC_Config_Channel(TIM_OCMODE_FORCED_ACTIVE, TIM_CHANNEL_3);
            break;
        case 4:
        	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, RESET);
			HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, SET);
			HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, SET);

			HAL_GPIO_WritePin(PWM1EN_GPIO_Port, PWM1EN_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(PWM2EN_GPIO_Port, PWM2EN_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(PWM3EN_GPIO_Port, PWM3EN_Pin, GPIO_PIN_RESET);

			bldc_motor_OC_Config_Channel(TIM_OCMODE_FORCED_INACTIVE, TIM_CHANNEL_1);
			bldc_motor_OC_Config_Channel(TIM_OCMODE_FORCED_INACTIVE, TIM_CHANNEL_4);
			bldc_motor_OC_Config_Channel(TIM_OCMODE_FORCED_ACTIVE, TIM_CHANNEL_3);

//            bldc_motor_OC_Config_Channel(TIM_OCMODE_FORCED_ACTIVE, TIM_CHANNEL_1);
//            bldc_motor_PWM_Config_Channel(bldc.speed_pulse, TIM_CHANNEL_4);
//            bldc_motor_OC_Config_Channel(TIM_OCMODE_FORCED_INACTIVE, TIM_CHANNEL_3);
            break;
        case 5:
        	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, RESET);
			HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, SET);
			HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, SET);

			HAL_GPIO_WritePin(PWM1EN_GPIO_Port, PWM1EN_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(PWM2EN_GPIO_Port, PWM2EN_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(PWM3EN_GPIO_Port, PWM3EN_Pin, GPIO_PIN_SET);


			bldc_motor_OC_Config_Channel(TIM_OCMODE_FORCED_INACTIVE, TIM_CHANNEL_1);
			bldc_motor_OC_Config_Channel(TIM_OCMODE_FORCED_ACTIVE, TIM_CHANNEL_4);
			bldc_motor_OC_Config_Channel(TIM_OCMODE_FORCED_ACTIVE, TIM_CHANNEL_3);
//            bldc_motor_OC_Config_Channel(TIM_OCMODE_FORCED_ACTIVE, TIM_CHANNEL_1);
//            bldc_motor_OC_Config_Channel(TIM_OCMODE_FORCED_INACTIVE, TIM_CHANNEL_4);
//            bldc_motor_PWM_Config_Channel(bldc.speed_pulse, TIM_CHANNEL_3);
            break;
        case 6:
        	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, RESET);
			HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, RESET);
			HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, SET);

			HAL_GPIO_WritePin(PWM1EN_GPIO_Port, PWM1EN_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(PWM2EN_GPIO_Port, PWM2EN_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(PWM3EN_GPIO_Port, PWM3EN_Pin, GPIO_PIN_SET);

			bldc_motor_OC_Config_Channel(TIM_OCMODE_FORCED_INACTIVE, TIM_CHANNEL_1);
			bldc_motor_OC_Config_Channel(TIM_OCMODE_FORCED_ACTIVE, TIM_CHANNEL_4);
			bldc_motor_OC_Config_Channel(TIM_OCMODE_FORCED_INACTIVE, TIM_CHANNEL_3);

//            bldc_motor_OC_Config_Channel(TIM_OCMODE_FORCED_INACTIVE, TIM_CHANNEL_1);
//            bldc_motor_OC_Config_Channel(TIM_OCMODE_FORCED_ACTIVE, TIM_CHANNEL_4);
//            bldc_motor_PWM_Config_Channel(bldc.speed_pulse, TIM_CHANNEL_3);
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

//void bldc_motor_six_step_algorithm(void)
//{
//	switch (bldc.step_number)
//	{
//		case 1:
//			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1, bldc.speed_pulse);
//			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
//			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
//			break;
//		case 2:
//			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
//			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, bldc.speed_pulse);
//			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
//			break;
//		case 3:
//			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
//			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
//			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, bldc.speed_pulse);
//			break;
//		case 4:
//			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, bldc.speed_pulse);
//			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
//			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
//			break;
//		case 5:
//			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
//			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, bldc.speed_pulse);
//			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
//			break;
//		case 6:
//			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
//			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
//			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, bldc.speed_pulse);
//			break;
//	}
//
//	if (bldc.dir == 1)  // CW direction
//	{
//		bldc.step_number++;
//		if (bldc.step_number > 6)
//			bldc.step_number = 1;
//	}
//	else if (bldc.dir == 0)  // CCW direction
//	{
//		bldc.step_number--;
//		if (bldc.step_number < 1)
//			bldc.step_number = 6;
//	}
//}

