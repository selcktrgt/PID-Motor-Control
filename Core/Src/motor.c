/*
 * motor.c
 *
 *  Created on: Mar 22, 2025
 *      Author: SELCUK
 */

#include <mpu6050.h>
#include <main.h>
#include <motor.h>
#include <stdio.h>
#include <math.h>
#include <pid.h>

extern TIM_HandleTypeDef htim1;

double roll_error;
double target_angle=0;
double roll_control_sig;

double calculate_motor_powers(float angle_roll, double dt){


roll_error=target_angle-angle_roll;



return 0;

}
