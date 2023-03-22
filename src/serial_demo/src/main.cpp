#include "serial_demo.hpp"

int main(int argc, char *argv[])
{
    int count = 0;
    setlocale(LC_ALL, "");
    setlocale(LC_CTYPE, "zh_CN.utf8");
    ros::init(argc, argv, "serial_port");
    ros::NodeHandle n;
    ros::Rate rate(10);
    STM32_Serial stm32_Serial(n);
    stm32_Serial.Subsribe_cmd_vel();
    signal(SIGINT, MySigintHandler);
    ROS_INFO("serial node is running");
    // ros::Subscriber velocityCMD = n.subscribe<geometry_msgs::Twist>("/cmd_vel", 1000, boost::bind(&STM32_Serial::V_CallBack, &stm32_Serial, _1));
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
        count++;
    }
    stm32_Serial.Serial_Close(port);
    return 0;
}
