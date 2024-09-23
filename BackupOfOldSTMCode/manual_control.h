/*
 * manual_control.h
 *
 *  Created on: Jul 10, 2024
 *      Author: sandr
 */

#ifndef INC_MANUAL_CONTROL_H_
#define INC_MANUAL_CONTROL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include "pid.h"

void baseMotorClockwise(double sp);
void baseMotorCounterclockwise(double sp);
void tiltMotorClockwise(double sp);
void tiltMotorCounterclockwise(double sp);
void stopAllMotors();

void runMotors(PID_TypeDef *basePID, PID_TypeDef *tiltPID, int baseSetPoint, int tiltSetPoint);

#ifdef __cplusplus
}
#endif

#endif /* INC_MANUAL_CONTROL_H_ */
