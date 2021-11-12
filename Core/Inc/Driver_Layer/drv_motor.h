/*
 * drv_motor.h
 *
 *  Created on: Mar 26, 2021
 *      Author: YUWEI, YEH
 */

#ifndef INC_DRIVER_LAYER_DRV_MOTOR_H_
#define INC_DRIVER_LAYER_DRV_MOTOR_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "robot_param.h"
#include <math.h>

/* Data type for motor and the encoder on motor */
typedef struct _Motor{
	// For motor
	GPIO_TypeDef		*INA_port;
	uint16_t			INA_pin;
	GPIO_TypeDef		*INB_port;
	uint16_t			INB_pin;
	TIM_HandleTypeDef 	*PWM_TIM;
	uint32_t			PWM_channel;
	MOTOR_STATE 		duty;
	MOTOR_STATE 		duty_old;

	// For encoder
	TIM_HandleTypeDef	*ENC_TIM;
	STATE				direction;
	uint32_t 			ENC_position;
	ENCODER_STATE 		velocity;
	ENCODER_STATE 		velocity_old;
	ENCODER_STATE 		error_velocity;
	ENCODER_STATE 		error_velocity_old;
	ENCODER_STATE		position;
	ENCODER_STATE		wheel_size; // radius(mm)
}Motor;


/* Motor Function*/
void Drv_Motor_Init(Motor* motor);
void Drv_Motor_ControlwithWheelVelocity(Motor* motor, ENCODER_STATE REF_velocity); // control with the velocity of two wheel
void Drv_Motor_ReadVelocity(Motor* motor);
void Drv_Motor_ReadPosition(Motor* motor);

#endif /* INC_DRIVER_LAYER_DRV_MOTOR_H_ */
