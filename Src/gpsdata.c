#include "gpsdata.h"
#include "usart.h"
#include "car.h"
uint8_t gpsdata[100];
#define PI 3.14159265358979323846
#define DEG_TO_RAD (PI / 180.0)
#define RAD_TO_DEG (180.0 / PI)
 double longitude  = 0 ;
 double latitude = 0 ;
 int positionPointTag = 0;
 point currentLoAndLa;
point points[] = {
//      {31.901427,118.899738},
//     {31.901395, 118.899714},
//     {31.901388, 118.899736},
//     {31.901458, 118.899888}
	{31.901447,118.899768},{31.901454,118.899793},{31.901442,118.899800},{31.901433,118.899776}
};
void init_GPS(){
	HAL_UART_Receive_DMA(&huart2,gpsdata,100);
}
#define EARTH_RADIUS 6371000.0 // 地球半径，单位：米
#define ALLOWABLE_DISTANCE 0.5 // 允许的误差范围，单位：米

// 计算两点之间的距离，返回单位：米
double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
    double dLat = (lat2 - lat1) * PI / 180.0;
    double dLon = (lon2 - lon1) * PI / 180.0;

    // 转换为弧度
    lat1 = lat1 * PI / 180.0;
    lat2 = lat2 * PI / 180.0;

    // Haversine 公式
    double a = sin(dLat / 2) * sin(dLat / 2) +
               cos(lat1) * cos(lat2) *
               sin(dLon / 2) * sin(dLon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    double distance = EARTH_RADIUS * c;
    
    return distance;
}

double calculateBearing(point start, point end) {
    // 将经纬度转换为弧度
    double lat1 = start.longitude * DEG_TO_RAD;
    double lat2 = end.longitude * DEG_TO_RAD;
    double dLon = (end.latitude - start.latitude) * DEG_TO_RAD;

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
void updateCurrentPosition() {
    latitude = ((int32_t)gpsdata[33] << 24) | ((int32_t)gpsdata[32] << 16) | ((int32_t)gpsdata[31] << 8) | gpsdata[30];
    longitude = ((int32_t)gpsdata[37] << 24) | ((int32_t)gpsdata[36] << 16) | ((int32_t)gpsdata[35] << 8) | gpsdata[34];
    longitude = longitude * 1e-7;
    latitude = latitude * 1e-7;
    currentLoAndLa.longitude = longitude;
    currentLoAndLa.latitude = latitude;

    // 计算当前位置与目标点之间的距离
    double distance = calculateDistance(currentLoAndLa.latitude, currentLoAndLa.longitude, 
                                        points[positionPointTag].latitude, points[positionPointTag].longitude);
	char testdata[200];
    sprintf(testdata," %f %d%f,%f  aim %f,%f  num %d \n",currentm,magangle,currentLoAndLa.longitude,currentLoAndLa.latitude,points[positionPointTag].longitude,points[positionPointTag].latitude,positionPointTag);
			liuxinusart(&huart3,testdata);
    // 如果距离小于允许范围，则切换到下一个目标点
    if (distance <= ALLOWABLE_DISTANCE) {
        positionPointTag++;
        if (positionPointTag == sizeof(points) / sizeof(points[0])) 
            positionPointTag = 0;
    }
}