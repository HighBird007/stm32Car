#include "hmc5883l.h"
#include "usart.h"
#include "math.h"
#include "spi.h"
#include "m25q.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
uint8_t xyz[6];
int xoffset = 0, yoffset = 0;

void hmc5883l_init(){
		int initcountread =0 ;
	    char initcount[50];

	
		uint8_t data = 0x70;
	
		HAL_I2C_Mem_Write(&hi2c1,HMC5883L_ADDRESS,CONFIGURATION_A, 1, &data, 1,1000); //配置寄存器A
		
		data = 0x20;
	
		HAL_I2C_Mem_Write(&hi2c1,HMC5883L_ADDRESS,CONFIGURATION_B, 1, &data, 1,1000); //配置寄存器B
	
		data = 0x0;
	
		HAL_I2C_Mem_Write(&hi2c1,HMC5883L_ADDRESS,HMC5883L_MODE, 1, &data, 1,1000); //配置模式寄存器，连续模式


	
	 initcountread =  readcount();
	sprintf(initcount,"initcount %d \n",initcountread);
	HAL_UART_Transmit(&huart3,(uint8_t*)initcount,strlen(initcount),1000);	
	savecount(initcountread==0?1:0);	
		HAL_Delay(100);
	
	
			
	 
}
void hmc5883l_rawread(float *GaX, float *GaY){
		
		uint8_t data[6];
		
	  HAL_I2C_Mem_Read(&hi2c1,HMC5883L_ADDRESS, 0x03,1,data,6 , 1000); //连续读取
	
		int16_t dxra,dyra;
	
		dxra = (data[0] << 8) | data[1]; 
		*GaX = (float)dxra /1090;
	
		dyra = (data[4] << 8) | data[5]; 
		*GaY = (float)dyra /1090 ;

		
/*	dzra = (data[2] << 8) | data[3];	
		float GaZ = (float)dzra /1090 ;		
	*/
	
}



void hmc5883l_selftest(float *Xoffest,float *Yoffest,float *Kx,float *Ky){
	
		uint8_t i=0 ;
	
		float GaX,GaY,GaXmax=0,GaXmin=0,GaYmax=0,GaYmin=0;
		char count[50];
		while(i != 100)
	{
		hmc5883l_rawread(&GaX, &GaY);
	
		GaXmax = GaXmax < GaX? GaX:GaXmax;
	
		GaXmin = GaXmin > GaX? GaX:GaXmin;
	
		GaYmax = GaYmax < GaY? GaY:GaYmax;
	
		GaYmin = GaYmin > GaY? GaY:GaYmin;
			
		HAL_Delay(100);
		
		i++;
		//sprintf(count,"count %d %.1f %.1f \n",i,GaX,GaY);
		//HAL_UART_Transmit(&huart3,(uint8_t*)count,strlen(count),1000);
				
	}
	
	*Xoffest = (GaXmax+GaXmin)/2;
	*Yoffest = (GaYmax+GaYmin)/2;
	*Kx = 2/(GaXmax-GaXmin);
	*Ky = 2/(GaXmax-GaXmin);
	
}


int16_t hmc5883l_read(float Xoffest,float Yoffest,float Kx,float Ky){
	
		float rawGaX,rawGaY;
	
		int16_t Magangle;
	
		hmc5883l_rawread(&rawGaX,&rawGaY);
	
		float GaX = (rawGaX - Xoffest) * Kx;
	
		float GaY = (rawGaY - Yoffest) * Ky;
			
			if((GaX > 0)&&(GaY > 0)) Magangle = atan(GaY/GaX)*57.0;
			else if((GaX > 0)&&(GaY < 0)) Magangle = 360+atan(GaY/GaX)*57.0;
			else if((GaX == 0)&&(GaY > 0)) Magangle = 90;
			else if((GaX == 0)&&(GaY < 0)) Magangle = 270;
			else if(GaX < 0) Magangle = 180+atan(GaY/GaX)*57.0;
			
			return Magangle;
		
}
