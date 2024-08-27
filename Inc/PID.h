#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

typedef struct {

    /* 控制器增益 */
    float Kp;
    float Ki;
    float Kd;

    /* 微分低通滤波器时间常数 */
    float tau;

    /* 输出限幅 */
    float limMin;
    float limMax;
    
    /* 积分器限幅 */
    float limMinInt;
    float limMaxInt;

    /* 采样时间（以秒为单位） */
    float T;

    /* 控制器“记忆”变量 */
    float integrator;            /* 积分器 */
    float prevError;             /* 积分器所需的前一误差 */
    float differentiator;        /* 微分器 */
    float prevMeasurement;       /* 微分器所需的前一测量值 */

    /* 控制器输出 */
    float out;

} PIDController;

void  PIDController_Init(PIDController *pid);
float PIDController_Update(PIDController *pid, float setpoint, float measurement);

#endif
