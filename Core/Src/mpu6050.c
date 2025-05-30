/*
 * mpu6050.c
 *
 *  Created on: Jan 24, 2025
 *      Author: SELCUK
 */

#include <mpu6050.h>
#include <main.h>
#include <stdio.h>
#include <math.h>
extern I2C_HandleTypeDef hi2c1;



int16_t Accel_X_RAW;
int16_t Accel_Y_RAW;
int16_t Accel_Z_RAW;

int16_t Gyro_X_RAW;
int16_t Gyro_Y_RAW;
int16_t Gyro_Z_RAW;
uint8_t check;

float Ax;
float Ay;
float Az;


float Gx;
float Gy;
float Gz;


float KalmanAngleRoll=0;
float KalmanUncertaintyAngleRoll = 2*2;
float KalmanAnglePitch=0;
float KalmanUncertaintyAnglePitch = 2*2;


float Kalman1DOutput[] = {0,0};


float RateRoll, RatePitch, RateYaw;
float AngleRoll, AnglePitch;
float RateCalibrationNumber;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;

extern double dt;
uint32_t time1;
void mpu6050_init()
{

  HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 1000);

  if(check==104){

      //wake up
      uint8_t data=0;
      HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &data, 1, 1000);

      //smpl_rate 1KHZ
      data=0x07;
      HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &data, 1, 1000);

      //accel config FSEL=0 =>2G
      data=0x00;
      HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &data, 1, 1000);

      //gyro config FSEL=0 =>250
      data=0x00;
      HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &data, 1, 1000);



  }

}

void gyro_signals()
{
  //DLPF CONFIG for vibrations
  //10HZ
     time1=HAL_GetTick();
     uint8_t data= 0x05;
     HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1A, 1, &data, 1, 1000);

  //read GYRO

     uint8_t Rec_Data1[6];
     HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data1, 6, 1000);
     Gyro_X_RAW=(int16_t)(Rec_Data1[0] << 8 | Rec_Data1[1]);
     Gyro_Y_RAW=(int16_t)(Rec_Data1[2] << 8 | Rec_Data1[3]);
     Gyro_Z_RAW=(int16_t)(Rec_Data1[4] << 8 | Rec_Data1[5]);

     RateRoll = Gyro_X_RAW / 131.0;
     RatePitch = Gyro_Y_RAW / 131.0;
     RateYaw = Gyro_Z_RAW / 131.0;

     RateRoll -= RateCalibrationRoll;
     RatePitch -= RateCalibrationPitch;
     RateYaw -= RateCalibrationYaw;

}


void accel_signals()
{

      uint8_t Rec_Data[6];

      //read 6 bytes starting from accel_xout_h register

      HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_X_OUT_H_REG, 1, Rec_Data, 6, 1000);


      Accel_X_RAW=(int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
      Accel_Y_RAW=(int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
      Accel_Z_RAW=(int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

      Ax = Accel_X_RAW / 16384.0;
      Ay = Accel_Y_RAW / 16384.0;
      Az = Accel_Z_RAW / 16384.0;

      AngleRoll = atan(Ay/sqrt(Ax*Ax + Az*Az))*1/(3.142/180);
      AnglePitch = atan(Ax/sqrt(Ay*Ay + Az*Az))*1/(3.142/180);

      dt=HAL_GetTickFreq()-time1;

}

void calibrate_gyro()
{

     for(RateCalibrationNumber=0; RateCalibrationNumber<2000;RateCalibrationNumber++){

       gyro_signals();
       RateCalibrationRoll += RateRoll;
       RateCalibrationPitch += RatePitch;
       RateCalibrationYaw += RateYaw;
       HAL_Delay(1);
     }

     RateCalibrationRoll /= 2000;
     RateCalibrationPitch /= 2000;
     RateCalibrationYaw /= 2000;






}

void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement)
{
    KalmanState=KalmanState + (0.004*KalmanInput);
    KalmanUncertainty = KalmanUncertainty + ( 0.004 * 0.004 * 4 * 4);
    float KalmanGain = KalmanUncertainty * 1/((1*KalmanUncertainty) + (3 * 3));
    KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
    KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;

    Kalman1DOutput[0] = KalmanState;
    Kalman1DOutput[1] = KalmanUncertainty;
}

void Get_Angle()
{


    kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
    KalmanAngleRoll=Kalman1DOutput[0];
    kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
    KalmanAnglePitch=Kalman1DOutput[0]; //2 derece sapma var

}
