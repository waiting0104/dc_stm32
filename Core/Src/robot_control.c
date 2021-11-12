/*
 * robot_control.c
 *
 *  Created on: Mar 31, 2021
 *      Author: YUWEI, YEH
 */

#include "robot_control.h"

void Robot_init(Robot* robot){
	// Pin setting for motor_L
	robot->motor_L.INA_port = GPIOB;
	robot->motor_L.INA_pin = GPIO_PIN_8;
	robot->motor_L.INB_port = GPIOB;
	robot->motor_L.INB_pin = GPIO_PIN_9;
	robot->motor_L.PWM_TIM = &htim3;
	robot->motor_L.PWM_channel = TIM_CHANNEL_4;
	// Pin setting for encoder_L
	robot->motor_L.ENC_TIM = &htim5;
	robot->motor_L.direction = 1;
	robot->motor_L.wheel_size = 0.0475;
	Drv_Motor_Init(&robot->motor_L);

	// Pin setting for motor_R
	robot->motor_R.INA_port = GPIOC;
	robot->motor_R.INA_pin = GPIO_PIN_6;
	robot->motor_R.INB_port = GPIOC;
	robot->motor_R.INB_pin = GPIO_PIN_5;
	robot->motor_R.PWM_TIM = &htim3;
	robot->motor_R.PWM_channel = TIM_CHANNEL_3;
	// Pin setting for encoder_R
	robot->motor_R.ENC_TIM = &htim1;
	robot->motor_R.direction = -1;
	robot->motor_R.wheel_size = 0.0475;
	Drv_Motor_Init(&robot->motor_R);

	// Setting for internal interrupt timer
	robot->INT_TIM = &htim6;
	HAL_TIM_Base_Start_IT(robot->INT_TIM);
}

void Robot_ControlwithRobotVelocity(Robot* robot, ENCODER_STATE velocity, ENCODER_STATE angular_velocity){
	ENCODER_STATE velocity_L, velocity_R;
	velocity_R = (velocity + 0.5*DISTANCE_BETWEEN_WHEEL*angular_velocity)/robot->motor_R.wheel_size;
	velocity_L = (velocity - 0.5*DISTANCE_BETWEEN_WHEEL*angular_velocity)/robot->motor_L.wheel_size;
	Drv_Motor_ControlwithWheelVelocity(&robot->motor_R, velocity_R);
	Drv_Motor_ControlwithWheelVelocity(&robot->motor_L, velocity_L);
}
