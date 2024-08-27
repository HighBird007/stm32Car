#include "m25q.h"
uint8_t writeEnableCmd[] = {0x06};
uint8_t eraseEnableCmd[] = {0x20,0x00,0x00,0x00};
#define use userDevice();
#define unuse unuserDevice();
void userDevice(){
HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,0);
}
void unuserDevice(){
HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,1);
}
void eraseEnable(){
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,0);;
	HAL_SPI_Transmit(&hspi2,eraseEnableCmd,4,10000);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,1);;
    HAL_Delay(100);
}
void writeEnable(){
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,0);;
	HAL_SPI_Transmit(&hspi2,writeEnableCmd,1,10000);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,1);;
	HAL_Delay(100);
}
void savecount(uint8_t count){
    writeEnable();
    eraseEnable(); // 这里擦除后，所有的数据会变为 0xFF
    writeEnable();
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,0);
    uint8_t saveit[] = {0x02,0x00,0x00,0x00, count}; // 将 0x25 替换为 count
    HAL_SPI_Transmit(&hspi2, saveit, 5, 10000);
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,1);
    HAL_Delay(100);
}

uint8_t readcount(){
	uint8_t w25data[1];
	uint8_t readEnableCmd[]={ 0x03,0x00,0x00,0x00};
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,0);;
	HAL_SPI_Transmit(&hspi2,readEnableCmd,4,10000);
	HAL_SPI_Receive(&hspi2,w25data,1,10000);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,1);;
		uint8_t count = w25data[0];
	return count;
}