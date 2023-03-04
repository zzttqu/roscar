#include <ros/ros.h>
#include "serial/serial.h"
#include <std_msgs/String.h>
#include <string>
#include <iostream>
#include <sstream>
#include "myData.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
using namespace std;

#define HEADER 'S'
#define TAIL 'E'

// 发送和接受串口消息
char serialReqData[64];
uint8_t data3[10];
Data_Transer serial_Res_Data;

// uint8_t=unsigned char等价关系

class STM32_Serial
{
private:
    serial::Serial se;
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    AGV_Vel agv_vel;
    AGV_Pos agv_pos;

public:
    // 串口初始化
    void Serial_Init()
    {
        try
        {

            se.setPort("/dev/ttyS1");
            se.setBaudrate(9600);
            se.setTimeout(to);
            se.open();
        }
        catch (serial::PortNotOpenedException &e)
        {
            ROS_INFO_STREAM("打开串口失败");
        }
    }

    int Read_Data(uint8_t resBuff[], int buff_size)
    {
        if (se.available())
        {
            std_msgs::String res;
            resBuff[buff_size];
            try
            {
                se.read(resBuff, se.available());
            }
            catch (serial::IOException &e)
            {
                ROS_ERROR_STREAM("数据接收失败");
            }
        }
    }

    float Odom_Trans(uint8_t Data_High, uint8_t Data_Low)
    {
        float data_return;
        short transition_16;
        transition_16 = 0;
        transition_16 |= Data_High << 8;
        transition_16 |= Data_Low;
        data_return = (transition_16 / 1000) + (transition_16 % 1000) * 0.001; // 转化为float类型
        return data_return;
    }

    bool Get_Data()
    {
        short transition_16 = 0;                                 // 中间变量
        uint8_t i = 0, check = 0, error = 1, Receive_Data_Pr[1]; // 临时变量，保存下位机数据
        static int count = 0;                                    // 静态变量，用于计数
        Read_Data(Receive_Data_Pr, sizeof(Receive_Data_Pr));     // 通过串口读取下位机发送过来的数据
        serial_Res_Data.buffer[count] = Receive_Data_Pr[0];
        if (Receive_Data_Pr[0] == HEADER || count > 0) // 确保数组第一个数据为FRAME_HEADER
            count++;
        else
            count = 0;
        if (count == 8) // 验证数据包的长度
        {
            serial_Res_Data.data.Data_Header = serial_Res_Data.buffer[0];
            serial_Res_Data.data.Data_Tail = serial_Res_Data.buffer[7];
            count = 0;                                  // 为串口数据重新填入数组做准备
            if (serial_Res_Data.data.Data_Tail == TAIL) // 验证数据包的帧尾
            {
                agv_vel.X = Odom_Trans(serial_Res_Data.buffer[1], serial_Res_Data.buffer[2]); // 获取运动底盘X方向速度
                agv_vel.Y = Odom_Trans(serial_Res_Data.buffer[3], serial_Res_Data.buffer[4]); // 获取运动底盘Y方向速度
                agv_vel.Z = Odom_Trans(serial_Res_Data.buffer[5], serial_Res_Data.buffer[6]); // 获取运动底盘Z方向速度
                return true;
            }
        }
        return false;
    }

    void Publish_Odom()
    {
        // 定义tf 对象
        static tf::TransformBroadcaster odom_broadcaster;
        // 定义tf发布时需要的类型消息
        geometry_msgs::TransformStamped odom_trans;
        ros::NodeHandle n;
        static ros::Publisher pub_odom = n.advertise<nav_msgs::Odometry>("/odom", 1000);
        // 把Z轴转角转换为四元数进行表达
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(agv_pos.Z);
        nav_msgs::Odometry odom; // 实例化里程计话题数据
        ros::Time current_time=ros::Time::now();
        // 发布坐标变换父子坐标系
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";
        // 填充获取的数据
        odom_trans.transform.translation.x = posy; // x坐标
        odom_trans.transform.translation.y = posx; // y坐标
        odom_trans.transform.translation.z = 0;    // z坐标
        odom_trans.transform.rotation = odom_quat; // 偏航角
        // 发布tf坐标变换
        odom_broadcaster.sendTransform(odom_trans);

        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = "odom";         // 里程计TF父坐标
        odom.pose.pose.position.x = agv_pos.X; // 位置
        odom.pose.pose.position.y = agv_pos.Y;
        odom.pose.pose.position.z = 0;
        odom.pose.pose.orientation = odom_quat; // 姿态，通过Z轴转角转换的四元数
        odom.child_frame_id = "base_footprint"; // 里程计TF子坐标
        odom.twist.twist.linear.x = agv_vel.X;  // X方向速度
        odom.twist.twist.linear.y = agv_vel.Y;  // Y方向速度
        odom.twist.twist.angular.z = agv_vel.Z; // 绕Z轴角速度

        pub_odom.publish(odom); // Pub odometer topic //发布里程计话题
    }

    void Control()
    {
        static ros::Time _Last_Time = ros::Time::now();
        while (ros::ok())
        {
            ros::Time _Now = ros::Time::now();
            double Sampling_Time = (_Now - _Last_Time).toSec(); // 获取时间间隔，用于积分速度获得位移(里程)
            if (true == Get_Data())                             // 通过串口读取并校验下位机发送过来的数据，然后数据转换为国际单位
            {
                agv_pos.X += (agv_vel.X * cos(agv_pos.Z) - agv_vel.Y * sin(agv_pos.Z)) * Sampling_Time; // 计算X方向的位移，单位：m
                agv_pos.Y += (agv_vel.X * sin(agv_pos.Z) + agv_vel.Y * cos(agv_pos.Z)) * Sampling_Time; // 计算Y方向的位移，单位：m
                agv_pos.Z += agv_vel.Z * Sampling_Time;                                                 // 绕Z轴的角位移，单位：rad
                Publish_Odom();                                                                         // 发布里程计话题

                _Last_Time = _Now; // 记录时间，用于计算时间间隔
            }

            ros::spinOnce(); // 循环等待回调函数
        }
    }
};

int main(int argc, char *argv[])
{
    STM32_Serial stm32_Serial;
    ros::init(argc, argv, "serial_port");
    ros::NodeHandle n;
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    ros::Rate rate(10); // 一秒执行十次
    serial::Serial se;
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    try
    {

        se.setPort("/dev/ttyS1");
        se.setBaudrate(9600);
        se.setTimeout(to);
        se.open();
    }
    catch (serial::PortNotOpenedException &e)
    {
        ROS_INFO_STREAM("打开串口失败");
        return -1;
    }

    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
    se.close();
    return 0;
}
