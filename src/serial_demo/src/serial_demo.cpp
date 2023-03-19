#include <ros/ros.h>
#include "serial/serial.h"
#include <string>
#include <iostream>
#include "myData.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
using namespace std;

#define HEADER 'S'
#define TAIL 'E'
#define PI 3.14159
#define active_code 'I'
#define deactive_code 'S'
#define wheel_r_mm 150
#define encoder_num 1000
// sudo chmod 666 /dev/ttyUSB0
//  发送和接受串口消息
Motor_Parameter MOTOR_Parameters[4];
unsigned char STM32_Settings[16] = {'A'};
AGV_Vel agv_nav_vel;
// 车辆参数
int wheel_center_x = 250;
int wheel_center_y = 250;
// uint8_t=unsigned char等价关系
// # TODO 完成设置代码

static void Speed_Trans(AGV_Vel agv_vel)
{
    // 运动学解算出四个轮子的线速度
    MOTOR_Parameters[0].target = agv_vel.X + agv_vel.Y - agv_vel.Yaw * (wheel_center_x + wheel_center_y);
    MOTOR_Parameters[1].target = -agv_vel.X + agv_vel.Y - agv_vel.Yaw * (wheel_center_x + wheel_center_y);
    MOTOR_Parameters[2].target = agv_vel.X + agv_vel.Y + agv_vel.Yaw * (wheel_center_x + wheel_center_y);
    MOTOR_Parameters[3].target = -agv_vel.X + agv_vel.Y + agv_vel.Yaw * (wheel_center_x + wheel_center_y);
    for (uint8_t i = 0; i < 4; i++)
    {
        // 要先变为角速度值，再转化为preloader数值
        MOTOR_Parameters[i].preloader.i_data = (uint8_t)abs(PI * wheel_r_mm * 1000 / MOTOR_Parameters[i].target - 1);
        // 转向判断
        MOTOR_Parameters[i].direction_Target = (MOTOR_Parameters[i].target > 0) ? 1 : -1;
    }
    ROS_INFO_STREAM("A电机preloader" << MOTOR_Parameters[0].preloader.i_data << "B电机preloader" << MOTOR_Parameters[1].preloader.i_data << "C电机preloader" << MOTOR_Parameters[2].preloader.i_data << "D电机preloader" << MOTOR_Parameters[3].preloader.i_data);
};
// 正向运动学解算
static AGV_Vel Encoder_Trans()
{
    AGV_Vel agv_vel;
    agv_vel.X = (PI * wheel_r_mm / encoder_num *
                 (MOTOR_Parameters[0].encoder.i_data + MOTOR_Parameters[1].encoder.i_data + MOTOR_Parameters[2].encoder.i_data + MOTOR_Parameters[3].encoder.i_data) / 4);
    agv_vel.Y = (PI * wheel_r_mm / encoder_num *
                 (MOTOR_Parameters[0].encoder.i_data - MOTOR_Parameters[1].encoder.i_data + MOTOR_Parameters[2].encoder.i_data - MOTOR_Parameters[3].encoder.i_data) / 4);
    agv_vel.Yaw = (PI * wheel_r_mm / encoder_num *
                   (-MOTOR_Parameters[0].encoder.i_data - MOTOR_Parameters[1].encoder.i_data + MOTOR_Parameters[2].encoder.i_data + MOTOR_Parameters[3].encoder.i_data) / 4 / (wheel_center_x + wheel_center_y));
    return agv_vel;
}
class STM32_Serial
{
private:
    unsigned char Recieve_Buffer[32];
    // 发送实际上是12个字节
    unsigned char Send_Buffer[16];
    serial::Serial se;
    ros::NodeHandle n;
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    ros::Publisher pub_odom;
    ros::Subscriber velocityCMD;
    AGV_Vel agv_encoder_vel;
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
                        uint8_t CRC = 0x00;
                        for (size_t j = 0; j < 16; j++)
                        {
                            resBuff[j] = ccbuff[i + j];
                        }
                        for (size_t i = 0; i < 10; i++)
                        {
                            CRC = resBuff[i] ^ CRC;
                        }
                        if (CRC == resBuff[10])
                        {

                            return;
                        }
                        else
                        {
                            ROS_WARN_STREAM("接受数据校验未通过" << CRC);
                            memset(resBuff, 0x00, buff_size);
                        }
                    }
                }
                // ROS_INFO_STREAM(ccbuff);
            }
            catch (serial::IOException &e)
            {
                ROS_ERROR_STREAM("数据接收失败 " << e.what());
            }
        }
        memset(ccbuff, 0x00, 64);
    }

    void Recieve_Speed_Trans()
    {
        for (size_t i = 2; i < 10; i++)
        {
            if (i < 4)
            {
                MOTOR_Parameters[0].encoder.byte[(i - 2) % 2] = Recieve_Buffer[i];
            }
            else if (3 < i && i < 6)
            {
                MOTOR_Parameters[1].encoder.byte[(i - 2) % 2] = Recieve_Buffer[i];
            }
            else if (5 < i && i < 8)
            {
                MOTOR_Parameters[2].encoder.byte[(i - 2) % 2] = Recieve_Buffer[i];
            }
            else if (7 < i && i < 10)
            {
                MOTOR_Parameters[3].encoder.byte[(i - 2) % 2] = Recieve_Buffer[i];
            }
        }
    }
    // STM32设置
    void STM32_Set()
    {
        // 开启速度回传
        STM32_Settings[1] = 'I';
        // 电机启动
        STM32_Settings[2] = 'I';
        se.write(STM32_Settings, sizeof(STM32_Settings));
        ROS_INFO_STREAM("设置成功" << STM32_Settings);
    }
    void STM32_Stop()
    {
        // 关闭速度回传
        STM32_Settings[1] = 'S';
        // 电机停止
        STM32_Settings[2] = 'S';
        se.write(STM32_Settings, sizeof(STM32_Settings));
        ROS_INFO_STREAM("设置成功" << STM32_Settings);
    }

    void Send_Speed_Trans()
    {
        memset(Send_Buffer, 0x00, sizeof(Send_Buffer));
        // 导航的速度数据切分后传入
        // 赋值到buffer中进行传输，四bit为一个float
        Speed_Trans(agv_nav_vel);
        Send_Buffer[0] = HEADER;
        for (size_t i = 2; i < 10; i++)
        {
            if (i < 4)
            {
                Send_Buffer[i] = MOTOR_Parameters[0].preloader.byte[(i - 2) % 2];
            }
            else if (3 < i && i < 6)
            {
                Send_Buffer[i] = MOTOR_Parameters[1].preloader.byte[(i - 2) % 2];
            }
            else if (5 < i && i < 8)
            {
                Send_Buffer[i] = MOTOR_Parameters[2].preloader.byte[(i - 2) % 2];
            }
            else if (7 < i && i < 10)
            {
                Send_Buffer[i] = MOTOR_Parameters[3].preloader.byte[(i - 2) % 2];
            }
            else if (9 < i && i < 14)
            {
                Send_Buffer[i] = MOTOR_Parameters[(i - 2) % 4].direction_Target;
            }
        }
        for (size_t i = 0; i < 14; i++)
        {
            Send_Buffer[14] = Send_Buffer[i] ^ Send_Buffer[14];
        }
        Send_Buffer[11] = TAIL;
    }

public:
    STM32_Serial(ros::NodeHandle& node);
    // 串口初始化
    int Serial_Init(char port[])
    {
        try
        {
            se.setPort(port);
            se.setBaudrate(38400);
            se.setTimeout(to);
            se.open();
            STM32_Set();
            ROS_INFO("打开串口成功");
            return 1;
        }
        catch (serial::IOException &e)
        {
            ROS_ERROR_STREAM("打开串口失败" << e.what());
            return -1;
        }
    }
    int Serial_Close(char port[])
    {
        try
        {
            STM32_Stop();
            se.close();
            ROS_INFO("关闭串口成功");
            return 1;
        }
        catch (serial::IOException &e)
        {
            ROS_ERROR("关闭串口失败");
            return -1;
        }
    }

    // 发送速度信息
    int Send_Speed_Msg()
    {
        try
        {
            if (se.isOpen())
            {
                se.write(Send_Buffer, sizeof(Send_Buffer));
            }
            else
            {
                ROS_ERROR("串口未打开，无法发送信息");
            }
            return 1;
        }
        catch (const serial::IOException e)
        {
            ROS_ERROR("串口错误，无法发送信息");
            return -1;
        }
    }

    // 获取串口数据并转化
    bool Get_Data()
    {
        Read_Data(Recieve_Buffer, sizeof(Recieve_Buffer)); // 通过串口读取下位机发送过来的数据
        Recieve_Speed_Trans();
        if (Recieve_Buffer[0] == HEADER && Recieve_Buffer[11] == TAIL) // 验证数据包的帧尾
        {

            agv_encoder_vel = Encoder_Trans();
            // ROS_INFO("agv速度为x=%.3f y=%.3f z=%.3f", agv_encoder_vel.X, agv_encoder_vel.Y, agv_encoder_vel.Yaw);
            se.flush();
            return true;
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
        pub_odom = n.advertise<nav_msgs::Odometry>("/odom", 1000);
        nav_msgs::Odometry odom; // 实例化里程计话题数据

        ros::Time current_time = ros::Time::now();

        double dt = 0.05; // 循环获取STM32速度
        // 这里是AGV本身坐标系下的
        dx = agv_encoder_vel.X * dt;
        dy = agv_encoder_vel.Y * dt;
        dz = agv_encoder_vel.Yaw * dt;
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

        odom.child_frame_id = "base_link";                // 里程计TF子坐标
        odom.twist.twist.linear.x = agv_encoder_vel.X;    // X方向速度
        odom.twist.twist.linear.y = agv_encoder_vel.Y;    // Y方向速度
        odom.twist.twist.angular.z = agv_encoder_vel.Yaw; // 绕Z轴角速度
        pub_odom.publish(odom);                           // Pub odometer topic //发布里程计话题
        ros::spinOnce();
    }
    // 接收导航传入的速度数据
    void Subsribe_cmd_vel()
    {
        velocityCMD = n.subscribe<geometry_msgs::Twist>("/cmd_vel", 1000, boost::bind(&STM32_Serial::V_CallBack, this, _1));
    }
    void V_CallBack(const geometry_msgs::Twist::ConstPtr &msg);
};
void STM32_Serial::V_CallBack(const geometry_msgs::Twist::ConstPtr &msg)
{
    // 换算为mm/s
    float X = msg.get()->linear.x * 1000;
    float Y = msg.get()->linear.y * 1000;
    double Yaw = msg.get()->angular.z; // 这个是rad/s
    agv_nav_vel = {X, Y, Yaw};
    Send_Speed_Trans();
    Send_Speed_Msg();
    ROS_INFO_STREAM("X速度为" << agv_nav_vel.X << "Y速度为" << agv_nav_vel.Y << "Z转动速度为" << agv_nav_vel.Yaw);
}
STM32_Serial::STM32_Serial(ros::NodeHandle& node)
{
    n = node;
}
int main(int argc, char *argv[])
{
    int count = 0;
    setlocale(LC_ALL, "");
    setlocale(LC_CTYPE, "zh_CN.utf8");
    ros::init(argc, argv, "serial_port");
    ros::NodeHandle n;
    ros::Rate rate(200);
    STM32_Serial stm32_Serial(n);
    stm32_Serial.Subsribe_cmd_vel();
    char port[] = "/dev/ttyUSB0";
    ROS_INFO("serial node is running");
    //ros::Subscriber velocityCMD = n.subscribe<geometry_msgs::Twist>("/cmd_vel", 1000, boost::bind(&STM32_Serial::V_CallBack, &stm32_Serial, _1));
    if (stm32_Serial.Serial_Init(port) == -1)
    {
        return -1;
    }
    while (ros::ok() && count < 5000)
    {
        if (stm32_Serial.Get_Data())
        {
            stm32_Serial.Publish_Odom();
        }
        ros::spinOnce();
        rate.sleep();
        count++;
    }
    stm32_Serial.Serial_Close(port);
    return 0;
}
