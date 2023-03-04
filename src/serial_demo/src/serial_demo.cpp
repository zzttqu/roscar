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
#define PI 3.14159

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
    float posx = 0, posy = 0, dx = 0, dy = 0, dz = 0;

    void Read_Data(uint8_t resBuff[], int buff_size)
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

    void Odom_Trans(uint8_t data[])
    {
        for (size_t i = 2; i < 14; i++)
        {
            if (i < 6)
            {
                serial_Res_Data.data.X_speed.byte[(i - 2) % 4] = serial_Res_Data.buffer[i];
            }
            else if (5 < i && i < 10)
            {
                serial_Res_Data.data.Y_speed.byte[(i - 2) % 4] = serial_Res_Data.buffer[i];
            }
            else if (9 < i && i < 14)
            {
                serial_Res_Data.data.Z_speed.byte[(i - 2) % 4] = serial_Res_Data.buffer[i];
            }
        }
    }

public:
    // 串口初始化
    void Serial_Init(char port[])
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
    
    //获取串口数据并转化
    bool Get_Data()
    {
        uint8_t i = 0, check = 0, error = 1, Receive_Data_Pr[1]; // 临时变量，保存下位机数据
        static int count = 0;                                    // 静态变量，用于计数
        Read_Data(Receive_Data_Pr, sizeof(Receive_Data_Pr));     // 通过串口读取下位机发送过来的数据
        serial_Res_Data.buffer[count] = Receive_Data_Pr[0];
        if (Receive_Data_Pr[0] == HEADER || count > 0) // 确保数组第一个数据为FRAME_HEADER
            count++;
        else
            count = 0;
        if (count == 15) // 验证数据包的长度
        {
            serial_Res_Data.data.Data_Header = serial_Res_Data.buffer[0];
            serial_Res_Data.data.Data_Tail = serial_Res_Data.buffer[14];
            count = 0;                                  // 为串口数据重新填入数组做准备
            if (serial_Res_Data.data.Data_Tail == TAIL) // 验证数据包的帧尾
            {
                agv_vel.X = serial_Res_Data.data.X_speed.f_data / 1000; // 获取运动底盘X方向速度,并除以1000换算为m/s
                agv_vel.Y = serial_Res_Data.data.Y_speed.f_data / 1000; // 获取运动底盘Y方向速度
                agv_vel.Z = serial_Res_Data.data.Z_speed.f_data / 1000; // 获取运动底盘Z方向速度
                ROS_INFO("agv速度为x=%.3f y=%.3f z=%.3f", agv_vel.X, agv_vel.Y, agv_vel.Z);
                return true;
            }
        }
        return false;
    }
    //发布odom数据
    void Publish_Odom()
    {
        // 定义tf 对象
        static tf::TransformBroadcaster odom_broadcaster;
        // 定义tf发布时需要的类型消息
        geometry_msgs::TransformStamped odom_trans;
        ros::NodeHandle n;
        static ros::Publisher pub_odom = n.advertise<nav_msgs::Odometry>("/odom", 1000);
        nav_msgs::Odometry odom; // 实例化里程计话题数据

        ros::Time current_time = ros::Time::now();

        double dt = 0.05; // 循环获取STM32速度
        // 这里是AGV本身坐标系下的
        dx = agv_vel.X * dt;
        dy = agv_vel.Y * dt;
        dz = agv_vel.Z * dt;
        // 转化到世界坐标系下
        agv_pos.Yaw += dz;
        // 根据角度简化防止超过2pai
        agv_pos.Yaw = (agv_pos.Yaw > PI) ? (agv_pos.Yaw - 2 * PI) : ((agv_pos.Yaw < -PI) ? (agv_pos.Yaw + 2 * PI) : agv_pos.Yaw);
        // 世界坐标系下
        agv_pos.X -= dx * cos(agv_pos.Yaw) + dy * sin(agv_pos.Yaw);
        agv_pos.Y += dx * sin(agv_pos.Yaw) - dy * cos(agv_pos.Yaw);
        // 把Z轴转角转换为四元数进行表达
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(agv_pos.Yaw);

        // 发布坐标变换父子坐标系
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";
        // 填充获取的数据
        odom_trans.transform.translation.x = agv_pos.X; // x坐标
        odom_trans.transform.translation.y = agv_pos.Y; // y坐标
        odom_trans.transform.translation.z = 0;         // z坐标
        odom_trans.transform.rotation = odom_quat;      // 偏航角
        // 发布tf坐标变换
        odom_broadcaster.sendTransform(odom_trans);

        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";         // 里程计TF父坐标
        odom.pose.pose.position.x = agv_pos.X; // 位置
        odom.pose.pose.position.y = agv_pos.Y;
        odom.pose.pose.position.z = 0;
        odom.pose.pose.orientation = odom_quat; // 姿态，通过Z轴转角转换的四元数

        odom.child_frame_id = "base_link";      // 里程计TF子坐标
        odom.twist.twist.linear.x = agv_vel.X;  // X方向速度
        odom.twist.twist.linear.y = agv_vel.Y;  // Y方向速度
        odom.twist.twist.angular.z = agv_vel.Z; // 绕Z轴角速度

        pub_odom.publish(odom); // Pub odometer topic //发布里程计话题
        ros::spinOnce();
    }
};

int main(int argc, char *argv[])
{
    STM32_Serial stm32_Serial;
    ros::init(argc, argv, "serial_port");
    ros::NodeHandle n;
    ros::Rate rate(100);
    char port[]="/dev/ttyS1";
    ROS_INFO("serial node is running");
    try
    {
        stm32_Serial.Serial_Init(port);
    }
    catch (serial::PortNotOpenedException &e)
    {
        ROS_INFO_STREAM("打开串口失败");
        return -1;
    }

    while (ros::ok())
    {
        if (stm32_Serial.Get_Data())
        {
            stm32_Serial.Publish_Odom();
        }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
