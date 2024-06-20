/*
 * motor_control.c
 *
 *  Created on: Jun 19, 2024
 *      Author: max
 */

#include "motor_control.h"
//#include "adc.h"
//#include "tim.h"
//#include "usart.h"
//#include "gpio.h"
//#include "stm32f4xx_hal_adc.h"
//#include "stm32f4xx_hal_tim.h"
//#include "stm32f4xx_hal_usart.h"
#include "stm32f4xx_hal_gpio.h"


//void Motor_Init(void)
//{
//    MX_GPIO_Init();
//    MX_ADC1_Init();
//    MX_TIM1_Init();
//    MX_USART2_UART_Init();
//}

void Motor_Start(void)
{
    // Start PWM signals
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
}

void Delay(volatile uint32_t delay) {
    while (delay--) {
        __asm("nop");
    }
}

//void Spin_Motor(void) {
//    uint16_t speed = 2250; // Example initial speed (duty cycle)
//    while (1) {
//        // Commutation state 1
//        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
//        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
//        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
//        Delay(50000);
//
//        // Commutation state 2
//        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
//        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
//        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, speed);
//        Delay(50000);
//
//        // Commutation state 3
//        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
//        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed);
//        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, speed);
//        Delay(50000);
//
//        // Commutation state 4
//        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
//        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed);
//        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
//        Delay(50000);
//
//        // Commutation state 5
//        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
//        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
//        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, speed);
//        Delay(50000);
//
//        // Commutation state 6
//        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
//        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
//        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, speed);
//        Delay(50000);
//    }
//}

void Spin_Motor(void) {
    uint16_t speed = 50; // Example speed (duty cycle)
    while (1) {
        // Commutation state 1: A+ B- Cx
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, speed);
        HAL_Delay(1);

        // Commutation state 2: A+ Bx C-
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
        HAL_Delay(1);

        // Commutation state 3: Ax B+ C-
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, speed);
        HAL_Delay(1);

        // Commutation state 4: A- B+ Cx
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
        HAL_Delay(1);

        // Commutation state 5: A- Bx C+
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, speed);
        HAL_Delay(1);

        // Commutation state 6: Ax B- C+
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, speed);
        HAL_Delay(1);
    }
}

//void Spin_Motor(void) {
//    uint16_t speed = 100-1; // Example initial speed (duty cycle)
//    while (1) {
//        // Commutation state 1: A+ B- Cx
//        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
//        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
//        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
//        HAL_Delay(1);
//
//        // Commutation state 2: A+ Bx C-
//        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
//        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
//        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, speed);
//        HAL_Delay(1);
//
//        // Commutation state 3: Ax B+ C-
//        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
//        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed);
//        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, speed);
//        HAL_Delay(1);
//
//        // Commutation state 4: Ax B+ Cx
//        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
//        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed);
//        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
//        HAL_Delay(1);
//
//        // Commutation state 5: A- B+ Cx
//        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
//        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed);
//        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, speed);
//        HAL_Delay(1);
//
//        // Commutation state 6: A- Bx C+
//        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
//        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
//        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, speed);
//        HAL_Delay(1);
//    }
//}


void Motor_Stop(void)
{
    // Stop PWM signals
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
}

void Motor_SetSpeed(uint16_t speed)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, speed);
}


