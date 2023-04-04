/*
 * @Author: zzttqu zzttqu@gmail.com
 * @Date: 2023-03-28 11:55:46
 * @LastEditors: zzttqu zzttqu@gmail.com
 * @LastEditTime: 2023-04-04 14:14:46
 * @FilePath: /roscar/src/serial_demo/include/myData.hpp
 * @Description: 
 * 一个大学生的毕业设计
 */
#ifndef myData_H
#define myData_H
#include "KalmanFilter.hpp"

typedef struct _AGV_Vel_
{
    float X;
    float Y;
    double Yaw;
} AGV_Vel;

// 这个是在世界坐标系下的坐标，不是车自身坐标系下的
typedef struct _AGV_Pos_
{
    double X;
    double Y;
    double Yaw;
} AGV_Pos;

typedef struct _Motor_Parameter_
{
  float target_speed;
  short preloader;
  short direction_Target;
  short encoder;
  short direction_Now;
  short voltage;
  short current;
  short temperature;
} Motor_Parameter;
#endif