#ifndef __mpu_H
#define __mpu_H
#include "i2c.h"
#include "stdio.h"
#include "string.h"
#include "math.h"
#include "usart.h"
#define MPUADDRESS 0xD0

 
#define MPU6050_I2C      		hi2c1
#define MPU6050_ADDR     		0xD0   
#define I2C_TimeOut  			100
 
/*MPU6050内部寄存器地址*/
#define MPU_SAMPLE_RATE_REG		0X19	//采样频率分频器
#define MPU_GYRO_CFG_REG		0X1B	//陀螺仪配置寄存器
#define MPU_ACCEL_CFG_REG		0X1C	//加速度计配置寄存器
#define MPU_ACCEL_XOUTH_REG		0X3B	//加速度值,X轴高8位寄存器
#define MPU_TEMP_OUTH_REG		0X41	//温度值高八位寄存器
#define MPU_GYRO_XOUTH_REG		0X43	//陀螺仪值,X轴高8位寄存器
#define MPU_PWR_MGMT1_REG		0X6B	//电源管理寄存器1
#define MPU_DEVICE_ID_REG		0X75
void init_mpu(void);
void MPU_Get_RAW_Accelerometer(void);
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az);
#endif
