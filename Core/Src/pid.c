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
#include <stdlib.h>

int ki=0.0005;
int kp=1;
int kd=300;

double last_error=0;
double pid_i=0, pid_d=0,pid_p=0;
double control_sig;

extern TIM_HandleTypeDef htim1;

float setpoint = 0.0;  // Hedef açı = 0 derece
float error = 0.0;
float prev_error = 0.0;

extern double dt;
extern double dt1;
extern double last_time;
double D_ALPHA = 0.2;
double filtered_d = 0;
double MAX_I = 200;

double get_control_signal(float angle_roll)
{


  if(angle_roll <=7){

     error = (setpoint-angle_roll);

     pid_i+=error*dt;

     if (pid_i > MAX_I)
     {
       pid_i = MAX_I;
     };
     if (pid_i < -MAX_I){
       pid_i = -MAX_I;
     }

     pid_d = (error - prev_error) / dt;
     filtered_d = (1.0 - D_ALPHA) * filtered_d + D_ALPHA * pid_d;
     control_sig= (kp*error) + (ki*pid_i) + (kd*filtered_d) + 91;
  }



  if(angle_roll > 7){

       error = (setpoint-angle_roll);

       pid_i+=error*dt;
       pid_d=(error-prev_error)/dt;
       filtered_d = (1.0 - D_ALPHA) * filtered_d + D_ALPHA * pid_d;
       control_sig= (kp*error) + (ki*pid_i) + (kd*pid_d) + (91.0-(angle_roll*(5.0/40.0)));
    }







   prev_error=error;

   return control_sig;
}








