/*
 * pid.c
 *
 *  Created on: Feb 22, 2025
 *      Author: SELCUK
 */

#include <main.h>
#include <math.h>
#include <stdio.h>
#include <mpu6050.h>

int ki=10;
int kp=1;
int kd=50;

double last_error=0;
double pid_i, pid_d,pid_p;
double control_sig;

extern TIM_HandleTypeDef htim1;






double get_control_signal(float angle_roll, double error_roll,double dt)
{


  pid_p=error_roll;
  pid_i+=error_roll*dt;
  pid_d=(error_roll-last_error)/dt;
  control_sig= (kp*error_roll) + (ki*pid_i) + (kd*pid_d);
  last_error=error_roll;

  return control_sig;
}








