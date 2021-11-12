/*
 * drv_motor.c
 *
 *  Created on: Mar 26, 2021
 *      Author: YUWEI, YEH
 */
#include "Driver_layer/drv_motor.h"

void Drv_Motor_Init(Motor* motor){
	// initialize motor
	motor->duty = 0;
	motor->duty_old = 0;

	// initialize encoder
	HAL_TIM_Encoder_Start(motor->ENC_TIM, TIM_CHANNEL_ALL);
	motor->ENC_position = ENC_COUNTER_HALF;
	motor->ENC_TIM->Instance->CNT = ENC_COUNTER_HALF;
	motor->velocity = 0;
	motor->velocity_old = 0;
	motor->error_velocity = 0;
	motor->error_velocity_old = 0;
	motor->position = 0;
}

void Drv_Motor_ControlwithWheelVelocity(Motor* motor, ENCODER_STATE REF_velocity){ // control with the velocity of two wheel
	motor->error_velocity = REF_velocity - motor->velocity;
	motor->duty = (int)(motor->duty_old + 309.409*motor->error_velocity - 296.8012*motor->error_velocity_old);
	if(motor->duty > 0){ // if(a>0)? true condition: false condition
		HAL_GPIO_WritePin(motor->INA_port, motor->INA_pin, (motor->direction>0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(motor->INB_port, motor->INB_pin, (motor->direction>0) ? GPIO_PIN_RESET : GPIO_PIN_SET);
		if(motor->duty > 2250){
			motor->duty = 2250;
		}
		__HAL_TIM_SET_COMPARE(motor->PWM_TIM, motor->PWM_channel, motor->duty);
	}
	else if(motor->duty < 0){
		HAL_GPIO_WritePin(motor->INA_port, motor->INA_pin, (motor->direction>0) ? GPIO_PIN_RESET : GPIO_PIN_SET);
		HAL_GPIO_WritePin(motor->INB_port, motor->INB_pin, (motor->direction>0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
		if(motor->duty < -2250){
			motor->duty = -2250;
		}
		__HAL_TIM_SET_COMPARE(motor->PWM_TIM, motor->PWM_channel, -1*motor->duty);
	}
	HAL_TIM_PWM_Start(motor->PWM_TIM,  motor->PWM_channel);
	motor->duty_old = motor->duty;
	motor->error_velocity_old = motor->error_velocity;
}

void Drv_Motor_ReadVelocity(Motor* motor){
	motor->ENC_position = motor->ENC_TIM->Instance->CNT;
	motor->ENC_TIM->Instance->CNT = ENC_COUNTER_HALF;
	// distinguish the direction of rotation
	int Enc_Pos_diff = motor->ENC_position - ENC_COUNTER_HALF;
	if(Enc_Pos_diff > 0){
		// CNT2*M_PI/(delta_t*(enc_resolution*4)(one turn)*gear ratio (rad/s)
		motor->velocity = motor->direction*(double)Enc_Pos_diff*M_TWOPI/(INT_TIM_DT*ENC_RESOLUTION*4.0f*GEAR_RATIO);
	}
	else if(Enc_Pos_diff < 0){
		motor->velocity = motor->direction*(double)Enc_Pos_diff*M_TWOPI/(INT_TIM_DT*ENC_RESOLUTION*4.0f*GEAR_RATIO);
	}
	else{
		motor->velocity = 0.0;
	}
	Drv_Motor_ReadPosition(motor);
//	motor->velocity_old = motor->velocity;
}

void Drv_Motor_ReadPosition(Motor* motor){
	motor->position += motor->wheel_size*motor->velocity*INT_TIM_DT;
}
