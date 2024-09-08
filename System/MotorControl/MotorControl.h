#pragma once

#include "stdio.h"
#include <stdbool.h>

void MotorControlTask();
void AdcConvCpltCallback();

void MotorIncreaseSpeed();
void MotorDecreaseSpeed();
void MotorChangeDirection();
void MotorIncreaseTorque();
void MotorDecreaseTorque();
void MotorPotModeToggle();

float MotorGetSpeed();
float MotorGetTorque();
float MotorGetDirection();
bool MotorGetPotModeState();
