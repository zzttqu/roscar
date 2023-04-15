/*
 * @Author: zzttqu zzttqu@gmail.com
 * @Date: 2023-03-28 11:55:46
 * @LastEditors: zzttqu zzttqu@gmail.com
 * @LastEditTime: 2023-03-29 15:58:22
 * @FilePath: /roscar/src/serial_demo/include/serial_demo.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
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
    AGV_Vel Recieve_Speed_Trans(uint8_t Recieve_Buffer[]);
    void STM32_Set();
    void STM32_Stop();
    void Send_Speed_Trans();

public:
    STM32_Serial(ros::NodeHandle &node);
    int Serial_Init(string port,int baudrate);
    int Serial_Close();
    int Send_Speed_Msg();
    bool Get_Data();
    void Publish_Odom();
    void Subsribe_cmd_vel();
    void V_CallBack(const geometry_msgs::Twist::ConstPtr &msg);
};
#endif