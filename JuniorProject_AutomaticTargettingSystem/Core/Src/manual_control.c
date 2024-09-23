// manual_control.c

#include "manual_control.h"
#include "main.h"
#include "pid.h"
#include "math.h"

// Constants
#define FULL_DIM_X 640
#define FULL_DIM_Y 400
#define PWM_MAX_VALUE 65535

void runMotors(PID_TypeDef *basePID, PID_TypeDef *tiltPID, int base_proj_point, int tilt_proj_point, int base_set_point, int tilt_set_point){

	*basePID->MyInput = base_proj_point;
	*basePID->MySetpoint = base_set_point;
	PID_Compute(basePID, FULL_DIM_X);

	*tiltPID->MyInput = tilt_proj_point;
	*tiltPID->MySetpoint = tilt_set_point;
	PID_Compute(tiltPID, FULL_DIM_Y);

	//Base Motor Dir Set
	if(*basePID->MyOutput <= 0){
		HAL_GPIO_WritePin(motorA1_GPIO_Port, motorA1_Pin, 0);
		HAL_GPIO_WritePin(motorB1_GPIO_Port, motorB1_Pin, 1);
	}else{
		HAL_GPIO_WritePin(motorA1_GPIO_Port, motorA1_Pin, 1);
		HAL_GPIO_WritePin(motorB1_GPIO_Port, motorB1_Pin, 0);
	}

	//Tilt Motor Dir Set
	if(*tiltPID->MyOutput <= 0){
		HAL_GPIO_WritePin(motorA2_GPIO_Port, motorA2_Pin, 1);
		HAL_GPIO_WritePin(motorB2_GPIO_Port, motorB2_Pin, 0);
	}else{
		HAL_GPIO_WritePin(motorA2_GPIO_Port, motorA2_Pin, 0);
		HAL_GPIO_WritePin(motorB2_GPIO_Port, motorB2_Pin, 1);
	}

	//Speed set
	TIM2->CCR1 = (fabs(*basePID->MyOutput) + 0.01)*PWM_MAX_VALUE;
	TIM2->CCR2 = (fabs(*tiltPID->MyOutput))*PWM_MAX_VALUE;
}


