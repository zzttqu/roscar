#ifndef Serial_My_H
#define Serial_My_H

#include <ros/ros.h>
#include <sstream>
#include "serial/serial.h"
#include <string>
#include <iostream>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <signal.h>
#include "myData.hpp"
#include "KalmanFilter.hpp"
using namespace std;

#define HEADER 'S'
#define TAIL 'E'
#define PI 3.14159
#define active_code 'I'
#define deactive_code 'S'
#define wheel_r_mm 150.0
#define encoder_num 1000.0
#define dt 0.1

extern Motor_Parameter MOTOR_Parameters[4];
extern unsigned char STM32_Settings[16];
extern AGV_Vel agv_nav_vel;
// 车辆参数
extern int wheel_center_x ;
extern int wheel_center_y;
extern serial::Serial se;
extern char port[] ;

static void Speed_Trans(AGV_Vel agv_vel);
static AGV_Vel Encoder_Trans();
void MySigintHandler(int sig);
class STM32_Serial
{
private:
    unsigned char Recieve_Buffer[32];
    // 发送实际上是12个字节
    unsigned char Send_Buffer[16];
    ros::NodeHandle n;
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    ros::Publisher pub_odom;
    ros::Subscriber velocityCMD;
    AGV_Vel agv_encoder_vel;
    AGV_Pos agv_pos;
    double posx = 0, posy = 0, dx = 0, dy = 0, dz = 0;

    void Read_Data(uint8_t resBuff[], int buff_size);
    void Recieve_Speed_Trans();
    void STM32_Set();
    void STM32_Stop();
    void Send_Speed_Trans();

public:
    STM32_Serial(ros::NodeHandle &node);
    int Serial_Init(char port[]);
    int Serial_Close(char port[]);
    int Send_Speed_Msg();
    bool Get_Data();
    void Publish_Odom();
    void Subsribe_cmd_vel();
    void V_CallBack(const geometry_msgs::Twist::ConstPtr &msg);
};
#endif