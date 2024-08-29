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

    p.T = 0.02f;              // 设置采样时间（秒＿
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
	Turn_left();

}
void Car_TurnRight(void){
    Enable_right();
	Turn_right();
}
void Car_Stop(void){
    Enable_left();
	Enable_right();
	shut();
	status = 0;
}
void Enable_left(void){
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
}
void Enable_right(void){
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
}

//左轮TIM_CHANNEL_2 右TIM_CHANNEL_1
void PWM_Turn(float num) {
    // 基础速度，确保车辆在直线行驶时两轮速度相同
    const int baseSpeed = 400;
    const int maxSpeedAdjustment = 160; // 最大调整幅度

    // num > 0 表示左转，num < 0 表示右转
    if (num > 0) {
		 
        int leftSpeed = baseSpeed + (int)(num * 20);
        int rightSpeed = baseSpeed - (int)(num * 20);

        if (rightSpeed < 0) rightSpeed = 0;  // 确保速度不为负
        if (leftSpeed > baseSpeed + maxSpeedAdjustment) leftSpeed = baseSpeed + maxSpeedAdjustment;

        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, leftSpeed);
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, rightSpeed);
        // 左转，右轮加速，左轮减速
       
    } else if (num < 0) {
        // 右转，左轮加速，右轮减速
		num = -num; // 取绝对值
        int leftSpeed = baseSpeed - (int)(num * 20);
        int rightSpeed = baseSpeed + (int)(num * 20);

        if (leftSpeed < 0) leftSpeed = 0;  // 确保速度不为负
        if (rightSpeed > baseSpeed + maxSpeedAdjustment) rightSpeed = baseSpeed + maxSpeedAdjustment;

        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, leftSpeed);
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, rightSpeed);
    } else {
        // num == 0，保持直线行驶
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, baseSpeed);
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, baseSpeed);
    }
}


