#ifndef __hmc_H
#define __hmc_H
#include "usart.h"
#include "stdio.h"
#include "string.h"
#include "i2c.h"
#define HMC5883L_ADDRESS 0x3C
#define CONFIGURATION_A  0x00
#define CONFIGURATION_B  0x01
#define HMC5883L_MODE  0x02
void hmc5883l_init();
void hmc5883l_rawread(float *GaX, float *GaY);
void hmc5883l_selftest(float *Xoffest,float *Yoffest,float *Kx,float *Ky);
int16_t hmc5883l_read(float Xoffest,float Yoffest,float Kx,float Ky);
#endif
