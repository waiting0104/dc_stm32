/*
 * robot_param.h
 *
 *  Created on: Mar 26, 2021
 *      Author: YUWEI, YEH
 */

#ifndef INC_ROBOT_PARAM_H_
#define INC_ROBOT_PARAM_H_

// data type define
#define STATE				int
#define MOTOR_STATE			volatile int
#define MOTOR_STATE_LONG  	volatile long int
#define ENCODER_STATE		volatile double
#define COMM_STATE  		uint8_t

// define several parameter
#define ENC_COUNTER_HALF (65536/2U)
#define INT_TIM_DT 0.001 // (second)
#define GEAR_RATIO 35.9
#define ENC_RESOLUTION 512.0
#define DISTANCE_BETWEEN_WHEEL 0.4 //(m)
#define FRICTION_DUTY_L 200U //(200/2250 V)
#define FRICTION_DUTY_R 200U //(200/2250 V)

#endif /* INC_ROBOT_PARAM_H_ */
