#include "ros/ros.h"
#include "std_msgs/String.h"
#include "serial/serial.h"
#include <sstream>

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"erGouZi");
    ros::NodeHandle nh;
    ros::Publisher pub=nh.advertise<std_msgs::String>("fang",10);
    std_msgs::String msg;
    ros::Rate rate(10);//一秒执行十次
    int count=0;
    while (ros::ok)
    {
        std::stringstream ss;
        ss<<"hello---->"<<count;
        count++;
        msg.data=ss.str();
        pub.publish(msg);
        ROS_INFO("发布的数据为%s",ss.str().c_str());
        rate.sleep();
    }
    
    return 0;
}
