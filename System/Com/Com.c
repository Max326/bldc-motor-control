#include "Com.h"

#include "usart.h"
#include "stdio.h"
#include <stdarg.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "MotorControl/MotorControl.h"

static void ComStartReceive();
static void Log(const char* format, ...);

#define LOG_BUFFER_SIZE 128

static uint8_t sChar;

void ComTask() {
	ComStartReceive();

	while(1) {
		float speed = MotorGetSpeed();
		float torque = MotorGetTorque();
		float direction = MotorGetDirection();
		bool potMode = MotorGetPotModeState();

		Log("speed = %.2f, torque = %.2f, direction = %d, potMode = %d\n",
				speed, torque, (int)direction, (int)potMode);

		osDelay(1000);
	}
}

void ComRxCpltCallback() {
	switch(sChar) {
		case '=': { // +
			MotorIncreaseSpeed();
		} break;
		case '-': { // -
			MotorDecreaseSpeed();
		} break;
		case 'd': {
			MotorChangeDirection();
		} break;
		case '[': {
			MotorDecreaseTorque();
		} break;
		case ']': {
			MotorIncreaseTorque();
		} break;
		case 'p': {
			MotorPotModeToggle();
		} break;
	}

	ComStartReceive();
}

static void ComStartReceive() {
	HAL_UART_Receive_DMA(&huart6, (uint8_t *)&sChar, 1);
}

static void Log(const char* format, ...) {
    static char logBuffer[LOG_BUFFER_SIZE];
    va_list args;
    va_start(args, format);

    vsnprintf(logBuffer, LOG_BUFFER_SIZE, format, args);

    va_end(args);

    HAL_UART_Transmit_DMA(&huart6, (uint8_t*)logBuffer, strlen(logBuffer));
}
