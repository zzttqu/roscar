#include "serial_node.hpp"

int main(int argc, char *argv[])
{
    int count = 0;
    setlocale(LC_ALL, "");
    setlocale(LC_CTYPE, "zh_CN.utf8");
    ros::init(argc, argv, "serial_port");
    ros::NodeHandle n;
    ros::NodeHandle n_private("~");
    ros::Rate rate(10);
    STM32_Serial stm32_Serial(n);
    stm32_Serial.Subsribe_cmd_vel();
    signal(SIGINT, MySigintHandler);
    ROS_INFO("serial node is running");
    //获取stm32串口参数
    string port = n_private.param<std::string>("serial_port", "ttyUSB0");
    int baudrate = n_private.param<int>("serial_baudrate", 38400);
    if (stm32_Serial.Serial_Init(port, baudrate) == -1)
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
        count++;
    }
    stm32_Serial.Serial_Close();
    return 0;
}
