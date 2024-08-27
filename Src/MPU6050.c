#include "mpu6050.h"
const float alpha = 0.95238;
void init_mpu(){
uint8_t check;
	uint8_t Data;
 
	// check device ID WHO_AM_I
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU_DEVICE_ID_REG, 1, &check, 1, I2C_TimeOut);
	// 0x68 will be returned by the sensor if everything goes well
	if (check == 104) 
	{
			// power management register 0X6B we should write all 0's to wake the sensor up
			Data = 0;
			HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU_PWR_MGMT1_REG, 1, &Data, 1, I2C_TimeOut);
 
			// Set DATA RATE of 1KHz by writing SMPLRT_DIV register
			Data = 0x07;
			HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU_SAMPLE_RATE_REG, 1, &Data, 1, I2C_TimeOut);
 
			// Set accelerometer configuration in ACCEL_CONFIG Register
			// XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 ->   2g
			Data = 0x00;
			HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU_ACCEL_CFG_REG, 1, &Data, 1, I2C_TimeOut);
 
			// Set Gyroscopic configuration in GYRO_CONFIG Register
			// XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 ->   250  /s
			Data = 0x00;
			HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU_GYRO_CFG_REG, 1, &Data, 1, I2C_TimeOut);
	
	}
}
float read_tepm(){
	uint8_t temo[2];
    HAL_I2C_Mem_Read(&hi2c1,MPUADDRESS,0x41,1,temo,2,1000);
	short t = (uint16_t)temo[0]<<8|temo[1];
	float ans = 36.53+((double)t)/340;
	return ans;
}

void MPU_Get_RAW_Accelerometer()
{
	uint8_t buf[6];  
	HAL_I2C_Mem_Read(&hi2c1,MPUADDRESS, MPU_ACCEL_XOUTH_REG, 1, buf, 6, 1000);
		int16_t ax,ay,az;
		ax=((int16_t)(buf[0]<<8)|buf[1]);  
		ay=((int16_t)(buf[2]<<8)|buf[3]);  
		az=((int16_t)(buf[4]<<8)|buf[5]);
	    float roll_a = atan2(ay,az)/3.141593f*180.0f;
		float pitch_a = -atan2(ax,az)/3.141593f*180.0f;
		char data[50];
//		sprintf(data,"roll_a  =  %f\n",roll_a);
//		HAL_UART_Transmit(&huart3,(uint8_t*)data,strlen(data),1000);
//		sprintf(data,"pitch_a =  %f\n",pitch_a);
       HAL_UART_Transmit(&huart3,(uint8_t*)data,strlen(data),1000);
     int16_t x,y,z;
	 HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU_GYRO_XOUTH_REG, 1, buf, 6, I2C_TimeOut);

		x=((int16_t)(buf[0]<<8)|buf[1]);  
		y=((int16_t)(buf[2]<<8)|buf[3]);  
		z=((int16_t)(buf[4]<<8)|buf[5]);
//		sprintf(data,"x  =  %f\n",x/131.0);
//		sprintf(data,"y  =  %f\n",y/131.0);
//		sprintf(data,"z  =  %f\n",z/131.0);
		IMUupdate(ax,ay,az,x,y,z);
	 	
}

void MPU_Get_RAW_Gyroscope()
{
	uint8_t buf[6];  
	int16_t x,y,z;
	 HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU_GYRO_XOUTH_REG, 1, buf, 6, I2C_TimeOut);

		x=((int16_t)(buf[0]<<8)|buf[1]);  
		y=((int16_t)(buf[2]<<8)|buf[3]);  
		z=((int16_t)(buf[4]<<8)|buf[5]);
        char data[50];
		sprintf(data,"x  =  %f\n",x/131.0);
		sprintf(data,"y  =  %f\n",y/131.0);
		sprintf(data,"z  =  %f\n",z/131.0);
		
}

#define Kp 100.0f                        // 比例增益支配率收敛到加速度计/磁强计
#define Ki 0.002f                // 积分增益支配率的陀螺仪偏见的衔接
#define halfT 0.001f                // 采样周期的一半
 
float q0 = 1, q1 = 0, q2 = 0, q3 = 0;          // 四元数的元素，代表估计方向
float exInt = 0, eyInt = 0, ezInt = 0;        // 按比例缩小积分误差
 
float Yaw,Pitch,Roll;  //偏航角，俯仰角，翻滚角

void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az)
{
        float norm;
        float vx, vy, vz;
        float ex, ey, ez;  
 
        // 测量正常化
        norm = sqrt(ax*ax + ay*ay + az*az);      
        ax = ax / norm;                   //单位化
        ay = ay / norm;
        az = az / norm;      
 
        // 估计方向的重力
        vx = 2*(q1*q3 - q0*q2);
        vy = 2*(q0*q1 + q2*q3);
        vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
 
        // 错误的领域和方向传感器测量参考方向之间的交叉乘积的总和
        ex = (ay*vz - az*vy);
        ey = (az*vx - ax*vz);
        ez = (ax*vy - ay*vx);
 
        // 积分误差比例积分增益
        exInt = exInt + ex*Ki;
        eyInt = eyInt + ey*Ki;
        ezInt = ezInt + ez*Ki;
 
        // 调整后的陀螺仪测量
        gx = gx + Kp*ex + exInt;
        gy = gy + Kp*ey + eyInt;
        gz = gz + Kp*ez + ezInt;
 
        // 整合四元数率和正常化
        q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
        q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
        q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
        q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  
 
        // 正常化四元
        norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
        q0 = q0 / norm;
        q1 = q1 / norm;
        q2 = q2 / norm;
        q3 = q3 / norm;
 
        Pitch  = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3; // pitch ,转换为度数
        Roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; // rollv
        Yaw = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;                //此处没有价值，注掉
		char pry[200];
		sprintf(pry,"pitch %f roll %f yaw %f \n",Pitch,Roll,Yaw);
		HAL_UART_Transmit(&huart3,(uint8_t*)pry,strlen(pry),1000);
}
