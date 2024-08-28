#include "gpsdata.h"
uint8_t gpsdata[100];
#define PI 3.14159265358979323846
#define DEG_TO_RAD (PI / 180.0)
#define RAD_TO_DEG (180.0 / PI)
point points[] = {
     {31.899564371925493,118.90083978747536},
     {31.899431399302994,118.90084577433903},
     {31.89932344538577,118.90075678213088},
};
void init_GPSDATA(){
	HAL_UART_Receive_DMA(&huart2,gpsdata,100);
}
double calculateBearing(point start, point end) {
    // 将经纬度转换为弧度
    double lat1 = start.latitude * DEG_TO_RAD;
    double lat2 = end.latitude * DEG_TO_RAD;
    double dLon = (end.longitude - start.longitude) * DEG_TO_RAD;

    // 计算方向角
    double y = sin(dLon) * cos(lat2);
    double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
    double bearing = atan2(y, x) * RAD_TO_DEG;

    // 归一化为 0 到 360 度
    if (bearing < 0) {
        bearing += 360.0;
    }

    return bearing;
}