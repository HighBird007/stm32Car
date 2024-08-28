#ifndef gpsdata_H
#define gpsdata_H
#include "stdio.h"
#include "math.h"
#include "usart.h"
typedef struct{
	double longitude;
 double latitude;
 
}point;
void init_GPS();
double calculateBearing(point start, point end);
void updateCurrentPosition();

extern point points[];
extern uint8_t gpsdata[];

extern double longitude  ;
extern double latitude  ;

extern int positionPointTag ;
extern point currentLoAndLa;
#endif
