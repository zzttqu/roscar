#include <ros/ros.h>
#include "serial/serial.h"
#include <std_msgs/String.h>
#include <string>
#include <iostream>
#include <sstream>
#include "myData.h"
using namespace std;

#define HEADER 'S'
#define TAIL 'E'

// 发送和接受串口消息
char serialReqData[64];
uint8_t data3[10];
Data_Transer data_tr;
char serialResData[64];
// uint8_t=unsigned char等价关系

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

class STM32_Serial
{
private:
    serial::Serial se;
    serial::Timeout to = serial::Timeout::simpleTimeout(100);

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

    int read_Data(uint8_t resBuff[],int buff_size)
    {
        if (se.available())
        {
            std_msgs::String res;
            ostringstream oss;
            size_t n = se.available();
            resBuff[buff_size];
            uint8_t *reqBuff = reinterpret_cast<uint8_t *>(serialReqData);
            try
            {
                se.read(resBuff, se.available());
                strcpy(serialResData, reinterpret_cast<char *>(resBuff));
                res.data = serialResData;
            }
            catch (serial::IOException &e)
            {
                ROS_ERROR_STREAM("数据接收失败");
            }
            ROS_INFO_STREAM("接收到的数据为：" << serialResData); // 相当于C++的count
            
        }
    }
};

bool Get_Sensor_Data_New(STM32_Serial stm32)
{
    short transition_16 = 0;                                     // 中间变量
    uint8_t i = 0, check = 0, error = 1, Receive_Data_Pr[1];     // 临时变量，保存下位机数据
    static int count;                                            // 静态变量，用于计数
    stm32.read_Data(Receive_Data_Pr, sizeof(Receive_Data_Pr)); // 通过串口读取下位机发送过来的数据

    Receive_Data.rx[count] = Receive_Data_Pr[0];    // 串口数据填入数组
    Receive_Data.Frame_Header = Receive_Data.rx[0]; // 数据的第一位是帧头0X7B
    Receive_Data.Frame_Tail = Receive_Data.rx[23];  // 数据的最后一位是帧尾0X7D

    if (Receive_Data_Pr[0] == FRAME_HEADER || count > 0) // 确保数组第一个数据为FRAME_HEADER
        count++;
    else
        count = 0;
    if (count == 24) // Verify the length of the packet //验证数据包的长度
    {
        count = 0;                                 // 为串口数据重新填入数组做准备
        if (Receive_Data.Frame_Tail == FRAME_TAIL) // 验证数据包的帧尾
        {
            check = Check_Sum(22, READ_DATA_CHECK); // BCC校验通过或者两组数据包交错

            if (check == Receive_Data.rx[22])
            {
                error = 0; // 异或位校验成功
            }
            if (error == 0)
            {
                /*//Check receive_data.rx for debugging use //查看Receive_Data.rx，调试使用
                ROS_INFO("%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x",
                Receive_Data.rx[0],Receive_Data.rx[1],Receive_Data.rx[2],Receive_Data.rx[3],Receive_Data.rx[4],Receive_Data.rx[5],Receive_Data.rx[6],Receive_Data.rx[7],
                Receive_Data.rx[8],Receive_Data.rx[9],Receive_Data.rx[10],Receive_Data.rx[11],Receive_Data.rx[12],Receive_Data.rx[13],Receive_Data.rx[14],Receive_Data.rx[15],
                Receive_Data.rx[16],Receive_Data.rx[17],Receive_Data.rx[18],Receive_Data.rx[19],Receive_Data.rx[20],Receive_Data.rx[21],Receive_Data.rx[22],Receive_Data.rx[23]);
                */

                Receive_Data.Flag_Stop = Receive_Data.rx[1];                      // set aside //预留位
                Robot_Vel.X = Odom_Trans(Receive_Data.rx[2], Receive_Data.rx[3]); // 获取运动底盘X方向速度
                Robot_Vel.Y = Odom_Trans(Receive_Data.rx[4], Receive_Data.rx[5]); // 获取运动底盘Y方向速度
                Robot_Vel.Z = Odom_Trans(Receive_Data.rx[6], Receive_Data.rx[7]); // 获取运动底盘Z方向速度

                return true;
            }
        }
    }
    return false;
}

int main(int argc, char *argv[])
{
    STM32_Serial stm32_Serial;
    ros::init(argc, argv, "serial_port");
    ros::NodeHandle n;
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
        if (se.available())
        {
            std_msgs::String res;
            ostringstream oss;
            size_t n = se.available();
            uint8_t resBuff[64];
            uint8_t *reqBuff = reinterpret_cast<uint8_t *>(serialReqData);
            try
            {
                se.read(resBuff, se.available());
                strcpy(serialResData, reinterpret_cast<char *>(resBuff));
                res.data = serialResData;
            }
            catch (serial::IOException &e)
            {
                ROS_ERROR_STREAM("数据接收失败");
            }
            ROS_INFO_STREAM("接收到的数据为：" << serialResData); // 相当于C++的count
            try
            {
                se.write(reqBuff, sizeof(reqBuff));
            }
            catch (serial::IOException &e)
            {
                ROS_ERROR_STREAM("数据发送失败");
            }
            ROS_INFO_STREAM("发送到的数据为：" << reqBuff);
        }
        ros::spinOnce();
        rate.sleep();
    }
    se.close();
    return 0;
}
