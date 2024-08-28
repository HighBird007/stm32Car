#include "PID.h"
#include "usart.h"
#include "stdio.h"
#include "string.h"
#include "math.h"
void PIDController_Init(PIDController *pid) {

	/* Clear controller variables */
	pid->integrator = 0.0f;
	pid->prevError  = 0.0f;

	pid->differentiator  = 0.0f;
	pid->prevMeasurement = 0.0f;

	pid->out = 0.0f;

}
//num>0 则说明 当前角度小于预订方向角   <0则说明当前角度大于预定方向角
//角度差问题
//判断应该顺时针还是逆时针转 
float getTurnRaw(int point , int raw){
	int delta = point - raw;
	if (delta > 180) {
    delta -= 360;
    } else if (delta < -180) {
    delta += 360;
    }
	return delta;
	
}
float PIDController_Update(PIDController *pid, float setpoint, float measurement) {

    /*
    * Error signal
    */
	//error > 0  表明小车此时转向时顺时针 应该转动多少  反之
    float error = getTurnRaw(setpoint,measurement);


    /*
    * Proportional
    */
    float proportional = pid->Kp * error;


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


    /*
    * Derivative (band-limited differentiator)
    */
    pid->differentiator = -(2.0f * pid->Kd * (measurement - pid->prevMeasurement)  /* Note: derivative on measurement, therefore minus sign in front of equation! */
                        + (2.0f * pid->tau - pid->T) * pid->differentiator)
                        / (2.0f * pid->tau + pid->T);



    /*
    * Compute output and apply limits
    */
    pid->out = proportional + pid->integrator + pid->differentiator;

    if (pid->out > pid->limMax) {
        pid->out = pid->limMax;
    } else if (pid->out < pid->limMin) {
        pid->out = pid->limMin;
    }



    /* Store error and measurement for later use */
    pid->prevError       = error;
    pid->prevMeasurement = measurement;

    /* Return controller output */
    return pid->out;
}
