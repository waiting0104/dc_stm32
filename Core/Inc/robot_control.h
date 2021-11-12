/*
 * robot_control.h
 *
 *  Created on: Mar 31, 2021
 *      Author: YUWEI, YEH
 */

#ifndef INC_ROBOT_CONTROL_H_
#define INC_ROBOT_CONTROL_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include <math.h>
#include "Driver_Layer/drv_motor.h"
#include "robot_param.h"
#include "tim.h"

/* Declare the structure of mobile robot*/
typedef struct _Robot{
	Motor				motor_L;
	Motor				motor_R;
	TIM_HandleTypeDef*	INT_TIM;

}Robot;

void Robot_init(Robot* robot);
// control with the velocity, angular velocity of robot
void Robot_ControlwithRobotVelocity(Robot* robot, ENCODER_STATE velocity, ENCODER_STATE angular_velocity);
#endif /* INC_ROBOT_CONTROL_H_ */
