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
#define active_code 'I'
#define deactive_code 'S'

// sudo chmod 666 /dev/ttyUSB0
//  发送和接受串口消息
Data_Reciever serial_Res_Data;
Data_Sender serial_Send_Data;
unsigned char STM32_Settings[16] = {'A'};
// uint8_t=unsigned char等价关系
// # TODO 完成设置代码
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
        uint8_t ccbuff[64];
        if (se.available())
        {
            try
            {
                se.read(ccbuff, se.available());
                for (size_t i = 0; i < 64; i++)
                {
                    if (ccbuff[i] == HEADER)
                    {
                        for (size_t j = 0; j < 16; j++)
                        {
                            resBuff[j] = ccbuff[i + j];
                        }
                    }
                }
                // ROS_INFO_STREAM(ccbuff);
            }
            catch (serial::IOException &e)
            {
                ROS_ERROR_STREAM("数据接收失败 "<<e.what());
            }
        }
        memset(ccbuff, 0x00, 64);
    }

    void Recieve_Speed_Trans(uint8_t data[])
    {
        for (size_t i = 2; i < 14; i++)
        {
            if (i < 6)
            {
                serial_Res_Data.X_speed.byte[(i - 2) % 4] = serial_Res_Data.buffer[i];
            }
            else if (5 < i && i < 10)
            {
                serial_Res_Data.Y_speed.byte[(i - 2) % 4] = serial_Res_Data.buffer[i];
            }
            else if (9 < i && i < 14)
            {
                serial_Res_Data.Z_speed.byte[(i - 2) % 4] = serial_Res_Data.buffer[i];
            }
        }
    }
    // STM32设置
    void STM32_Set()
    {
        STM32_Settings[1] = 'I';
        se.write(STM32_Settings, sizeof(STM32_Settings));
    }

    void Speed_Trans()
    {
        serial_Send_Data.Data_Header = 'S';
        serial_Send_Data.Data_Tail = 'E';
        memset(serial_Send_Data.buffer, 0x00, sizeof(serial_Send_Data.buffer));

        // 导航的速度数据切分后传入
        // 赋值到buffer中进行传输，四bit为一个float
        serial_Send_Data.buffer[0] = serial_Send_Data.Data_Header;
        for (size_t i = 2; i < 14; i++)
        {
            if (i < 6)
            {
                serial_Send_Data.buffer[i] = serial_Send_Data.X_speed.byte[(i - 2) % 4];
            }
            else if (5 < i && i < 10)
            {
                serial_Send_Data.buffer[i] = serial_Send_Data.Y_speed.byte[(i - 2) % 4];
            }
            else if (9 < i && i < 14)
            {
                serial_Send_Data.buffer[i] = serial_Send_Data.Z_speed.byte[(i - 2) % 4];
            }
        }
        for (size_t i = 0; i < 14; i++)
        {
            serial_Send_Data.buffer[14]=serial_Send_Data.buffer[i]^serial_Send_Data.buffer[14];
        }
        serial_Send_Data.buffer[15]=serial_Send_Data.Data_Tail;
    }

public:
    // 串口初始化
    int Serial_Init(char port[])
    {
        try
        {
            se.setPort(port);
            se.setBaudrate(9600);
            se.setTimeout(to);
            se.open();
            STM32_Set();
            ROS_INFO("打开串口成功");
            return 1;
        }
        catch (serial::IOException &e)
        {
            ROS_ERROR("打开串口失败");
            return -1;
        }
    }

    // 发送速度信息
    int Send_Speed_Msg()
    {
        try
        {
            if(se.isOpen()){
                se.write(serial_Send_Data.buffer,sizeof(serial_Send_Data));
            }
            else{
               ROS_ERROR("串口未打开，无法发送信息"); 
            }  
            return 1;
        }
        catch(const serial::IOException e)
        {
            ROS_ERROR(e.what());
            return -1;
        }
    }

    // 获取串口数据并转化
    bool Get_Data()
    {
        static int count = 0;                                              // 静态变量，用于计数
        Read_Data(serial_Res_Data.buffer, sizeof(serial_Res_Data.buffer)); // 通过串口读取下位机发送过来的数据
        uint8_t CRC = 0x00;
        if (serial_Res_Data.buffer[0] == HEADER && serial_Res_Data.buffer[15] == TAIL) // 验证数据包的帧尾
        {
            for (size_t i = 0; i < 14; i++)
            {
                CRC = serial_Res_Data.buffer[i] ^ CRC;
            }
            ROS_INFO_STREAM("校验码为"<<CRC);
            ROS_INFO_STREAM(""<<serial_Res_Data.buffer);
            // if (CRC == serial_Res_Data.buffer[14])
            // {
            agv_vel.X = serial_Res_Data.X_speed.f_data / 1000; // 获取运动底盘X方向速度,并除以1000换算为m/s
            agv_vel.Y = serial_Res_Data.Y_speed.f_data / 1000; // 获取运动底盘Y方向速度
            agv_vel.Z = serial_Res_Data.Z_speed.f_data / 1000; // 获取运动底盘Z方向速度
            ROS_INFO("agv速度为x=%.3f y=%.3f z=%.3f", agv_vel.X, agv_vel.Y, agv_vel.Z);
            se.flush();
            CRC = 0x00;
            return true;
            // }
        }
        return false;
    }
    // 发布odom数据
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

    setlocale(LC_ALL, "");
    setlocale(LC_CTYPE, "zh_CN.utf8");
    STM32_Serial stm32_Serial;
    ros::init(argc, argv, "serial_port");
    ros::NodeHandle n;
    ros::Rate rate(10);
    char port[] = "/dev/ttyUSB0";
    ROS_INFO("serial node is running");

    if (stm32_Serial.Serial_Init(port) == -1)
    {
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
