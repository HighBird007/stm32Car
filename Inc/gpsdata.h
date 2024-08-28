#ifndef gpsdata_H
#define gpsdata_H
#include "stdio.h"
#include "math.h"
#include "usart.h"
typedef struct{
 double latitude;
 double longitude;
}point;
extern point points[];
extern uint8_t gpsdata[];
void init_GPS();
int gowhere();
double calculateBearing(point start, point end);
#endif
