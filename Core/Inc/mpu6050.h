/*
 * mpu6050.h
 *
 *  Created on: Jan 24, 2025
 *      Author: SELCUK
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#define MPU6050_ADDR  0xD0

#define SMPLRT_DIV_REG  0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG  0x1C
#define ACCEL_X_OUT_H_REG 0x3B
#define TEMP_OUT_H_REG  0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG  0x6B
#define WHO_AM_I_REG  0x75



void mpu6050_init();
void gyro_signals();
void accel_signals();
void calibrate_gyro();
void kalman_1d(float, float, float,  float);
void Get_Angle();


#endif /* INC_MPU6050_H_ */
