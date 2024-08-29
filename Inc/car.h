#ifndef car__h
#define car__h
#include "main.h"
#include "PID.h"
void car_init();
void Car_Forward(void);
void Car_Backward(void);
void Car_TurnLeft(void);
void Car_TurnRight(void);
void Car_Stop(void);
void Enable_right(void);
void Enable_left(void);
void Turn_Forward(void);
extern int status ;
extern int magangle ;
extern PIDController p;
void PWM_Turn( float num);
#endif