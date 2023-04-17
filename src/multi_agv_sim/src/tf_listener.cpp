#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"
int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "tf_listener");
    ROS_INFO("坐标变换已启动");
    ros::NodeHandle n;
    tf2_ros::TransformBroadcaster broadcaster;
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);
    ros::Rate rate(50);
    float y = 0.25;
    while (ros::ok())
    {
        
        geometry_msgs::TransformStamped tf1;
        try{
           geometry_msgs::TransformStamped tf2;
           tf2=buffer.lookupTransform("agv0/map","agv1/map",ros::Time(0));
        }
        catch(const std::exception& r){
            ROS_INFO_STREAM(r.what());
        }
        rate.sleep();
    }

    return 0;
}