#include "Pot.h"

#include "adc.h"
#include "stdio.h"

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

static void PotInit();

static uint16_t sPotentiometerRaw;
static uint16_t sPotentiometerValue;

void PotTask() {
	PotInit();

	while(1) {
		HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&sPotentiometerRaw, 1);

		osDelay(10);
	}
}

uint16_t PotGetValue() {
	return sPotentiometerValue;
}


void AdcConvCpltCallback() {
	sPotentiometerValue = sPotentiometerRaw;
}

static void PotInit() {
	HAL_StatusTypeDef status = HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&sPotentiometerRaw, 1);

	if (status == HAL_OK) {
		printf("Potentiometer OK\n");
	}
	else {
		printf("Potentiometer ERROR\n");
	}
}
