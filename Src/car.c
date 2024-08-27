#include "car.h"
#include "tim.h"
#include "hmc5883l.h"
//pwm 频类不能太快 否则失效 其次比较不能低于400否则跑不动
int status = 0;
int magangle = 0;
PIDController p;
void car_init(){
 	 p.Kp = 0.5f;             // 设置比例增益
    p.Ki = 0.1f;             // 设置积分增益
    p.Kd = 0.01f;            // 设置微分增益

    p.tau = 0.02f;           // 设置微分低鿚滤波器时间常数

    p.limMin = -10.0f;       // 设置输出朿小忿
    p.limMax = 10.0f;        // 设置输出朿大忿

    p.limMinInt = -5.0f;     // 设置积分器最小忿
    p.limMaxInt = 5.0f;      // 设置积分器最大忿

    p.T = 0.2f;              // 设置采样时间（秒＿
	PIDController_Init(&p);	
}
void Turn_Forward(void){
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,1);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,0);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,1);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,0);
}
void Turn_left(void){
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,0);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,0);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,1);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,0);
}
void Turn_right(void){
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,1);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,0);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,0);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,0);
}
void Turn_back(void){
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,0);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,1);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,0);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,1);
}
void shut(){
	    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,0);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,0);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,0);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,0);
}
void Car_Forward(void){
    Enable_left();
    Enable_right();
	Turn_Forward();
}
void Car_Backward(void){
    Enable_left();
    Enable_right();
	Turn_back();
}
void Car_TurnLeft(void){
    Enable_left();
    Enable_right();
	Turn_left();

}
void Car_TurnRight(void){
    Enable_right();
	Turn_right();
}
void Car_Stop(void){
    Enable_left();
	shut();
	status = 0;
}
void Enable_left(void){
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
}
void Enable_right(void){
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
}
void Set_Speed(speed){
	switch(speed){
		case low:
		__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,400);
		__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,400);
		break;
		case medium:
		__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,600);
		__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,600);
		break;
		case fast:
		__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,800);
		__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,800);
		break;
		case max:
		__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,999);
		__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,999);
		break;
	}
}
void PWM_Turn( float num){
	//num > 0 则左轮子 速度快  反之 右轮子速度快
	if(num>0){
		//右转
if(num>=8){
			__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,400+num*20);
		__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,0);
	return ;
		}
	__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,400+num*20);
		__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,400);
		
	}else if(num<0){
		num = __fabs(num);
		if(num>=8){
			__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,400+num*20);
		__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,0);
			return ;
		}
		
	__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,0);
		__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,400+num*20);
	}
}
