// manual_control.c

#include "manual_control.h"
#include "main.h"
#include "pid.h"

// Constants
#define FULL_PULSE_BASE_MOTOR 4148
#define FULL_PULSE_TILT_MOTOR 800
#define PWM_MAX_VALUE 65535

// Base motor control functions
void baseMotorClockwise(double sp) {
    HAL_GPIO_WritePin(motorA1_GPIO_Port, motorA1_Pin, 0);
    HAL_GPIO_WritePin(motorB1_GPIO_Port, motorB1_Pin, 1);
    int speed = (int)(sp * PWM_MAX_VALUE);

    TIM2->CCR1 = speed;
}

void baseMotorCounterclockwise(double sp) {
    HAL_GPIO_WritePin(motorA1_GPIO_Port, motorA1_Pin, 1);
    HAL_GPIO_WritePin(motorB1_GPIO_Port, motorB1_Pin, 0);
    int speed = (int)(sp * PWM_MAX_VALUE);

    TIM2->CCR1 = speed;
}

// Tilt motor control functions
void tiltMotorClockwise(double sp) {
    HAL_GPIO_WritePin(motorA2_GPIO_Port, motorA2_Pin, 1);
    HAL_GPIO_WritePin(motorB2_GPIO_Port, motorB2_Pin, 0);
    int speed = (int)(sp * PWM_MAX_VALUE);

    TIM2->CCR2 = speed;
}

void tiltMotorCounterclockwise(double sp) {
    HAL_GPIO_WritePin(motorA2_GPIO_Port, motorA2_Pin, 0);
    HAL_GPIO_WritePin(motorB2_GPIO_Port, motorB2_Pin, 1);
    int speed = (int)(sp * PWM_MAX_VALUE);

    TIM2->CCR2 = speed;
}

// Function to stop all motors
void stopAllMotors() {
    HAL_GPIO_WritePin(motorA1_GPIO_Port, motorA1_Pin, 0);
    HAL_GPIO_WritePin(motorB1_GPIO_Port, motorB1_Pin, 0);

    HAL_GPIO_WritePin(motorA2_GPIO_Port, motorA2_Pin, 0);
    HAL_GPIO_WritePin(motorB2_GPIO_Port, motorB2_Pin, 0);
}

void runMotors(PID_TypeDef *basePID, PID_TypeDef *tiltPID, int baseSetPoint, int tiltSetPoint){
	int baseCurPos = TIM4->CNT;
	int tiltCurPos = TIM3->CNT;

	int baseClockwiseDir = (baseSetPoint - baseCurPos + FULL_PULSE_BASE_MOTOR) % FULL_PULSE_BASE_MOTOR;
	int baseCounterclockwiseDir = (baseCurPos - baseSetPoint + FULL_PULSE_BASE_MOTOR) % FULL_PULSE_BASE_MOTOR;

	int tiltClockwiseDir = (tiltSetPoint - tiltCurPos + FULL_PULSE_TILT_MOTOR) % FULL_PULSE_TILT_MOTOR;
	int tiltCounterclockwiseDir = (tiltCurPos - tiltSetPoint + FULL_PULSE_TILT_MOTOR) % FULL_PULSE_TILT_MOTOR;

	//Base Motor Dir Set
	if(baseClockwiseDir <= baseCounterclockwiseDir){
		HAL_GPIO_WritePin(motorA1_GPIO_Port, motorA1_Pin, 0);
		HAL_GPIO_WritePin(motorB1_GPIO_Port, motorB1_Pin, 1);
	}else{
		HAL_GPIO_WritePin(motorA1_GPIO_Port, motorA1_Pin, 1);
		HAL_GPIO_WritePin(motorB1_GPIO_Port, motorB1_Pin, 0);
	}

	//Tilt Motor Dir Set
	if(tiltClockwiseDir <= tiltCounterclockwiseDir){
		HAL_GPIO_WritePin(motorA2_GPIO_Port, motorA2_Pin, 1);
		HAL_GPIO_WritePin(motorB2_GPIO_Port, motorB2_Pin, 0);
	}else{
		HAL_GPIO_WritePin(motorA2_GPIO_Port, motorA2_Pin, 0);
		HAL_GPIO_WritePin(motorB2_GPIO_Port, motorB2_Pin, 1);
	}

	*basePID->MyInput = baseCurPos;
	*basePID->MySetpoint = baseSetPoint;
	PID_Compute(basePID, FULL_PULSE_BASE_MOTOR);

	*tiltPID->MyInput = tiltCurPos;
	*tiltPID->MySetpoint = tiltSetPoint;
	PID_Compute(tiltPID, FULL_PULSE_TILT_MOTOR);

	TIM2->CCR1 = (*basePID->MyOutput + 0.04)*PWM_MAX_VALUE;
	TIM2->CCR2 = (*tiltPID->MyOutput + 0.02)*PWM_MAX_VALUE;
}


