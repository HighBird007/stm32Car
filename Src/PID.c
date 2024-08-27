#include "PID.h"
#include "usart.h"
#include "stdio.h"
#include "string.h"
void PIDController_Init(PIDController *pid) {

	/* Clear controller variables */
	pid->integrator = 0.0f;
	pid->prevError  = 0.0f;

	pid->differentiator  = 0.0f;
	pid->prevMeasurement = 0.0f;

	pid->out = 0.0f;

}

float PIDController_Update(PIDController *pid, float setpoint, float measurement) {

    /*
    * Error signal
    */
    float error = setpoint - measurement;

    // 调试：打印错误信号
//    char debug[100];
//    sprintf(debug, "Error: %f \n", error);
//    HAL_UART_Transmit(&huart3, (uint8_t*)debug, strlen(debug), 1000);

    /*
    * Proportional
    */
    float proportional = pid->Kp * error;

    // 调试：打印比例项
//    sprintf(debug, "Proportional: %f \n", proportional);
//    HAL_UART_Transmit(&huart3, (uint8_t*)debug, strlen(debug), 1000);

    /*
    * Integral
    */
    pid->integrator = pid->integrator + 0.5f * pid->Ki * pid->T * (error + pid->prevError);

    /* Anti-wind-up via integrator clamping */
    if (pid->integrator > pid->limMaxInt) {
        pid->integrator = pid->limMaxInt;
    } else if (pid->integrator < pid->limMinInt) {
        pid->integrator = pid->limMinInt;
    }

    // 调试：打印积分项
//    sprintf(debug, "Integrator: %f \n", pid->integrator);
//    HAL_UART_Transmit(&huart3, (uint8_t*)debug, strlen(debug), 1000);

    /*
    * Derivative (band-limited differentiator)
    */
    pid->differentiator = -(2.0f * pid->Kd * (measurement - pid->prevMeasurement)  /* Note: derivative on measurement, therefore minus sign in front of equation! */
                        + (2.0f * pid->tau - pid->T) * pid->differentiator)
                        / (2.0f * pid->tau + pid->T);

    // 调试：打印微分项
//    sprintf(debug, "Differentiator: %f \n", pid->differentiator);
//    HAL_UART_Transmit(&huart3, (uint8_t*)debug, strlen(debug), 1000);

    /*
    * Compute output and apply limits
    */
    pid->out = proportional + pid->integrator + pid->differentiator;

    if (pid->out > pid->limMax) {
        pid->out = pid->limMax;
    } else if (pid->out < pid->limMin) {
        pid->out = pid->limMin;
    }

    // 调试：打印最终输出
//    sprintf(debug, "PID Output: %f \n", pid->out);
//    HAL_UART_Transmit(&huart3, (uint8_t*)debug, strlen(debug), 1000);

    /* Store error and measurement for later use */
    pid->prevError       = error;
    pid->prevMeasurement = measurement;

    /* Return controller output */
    return pid->out;
}
