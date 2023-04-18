#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Twist.h"
#include "tf2/LinearMath/Quaternion.h"
int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "tf_listener");
    ROS_ERROR("坐标变换已启动");
    ros::NodeHandle n;
    ros::Publisher agv_1_vel = n.advertise<geometry_msgs::Twist>("agv_1/cmd_vel", 10);
    tf2_ros::TransformBroadcaster broadcaster;
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);
    ros::Rate rate(5);
    while (ros::ok())
    {

        try
        {
            geometry_msgs::TransformStamped tf2;
            tf2 = buffer.lookupTransform("agv_1/base_link", "agv_0/base_link", ros::Time(0));
            geometry_msgs::Twist vel_msg;
            double angular = 4*atan2(tf2.transform.translation.y, tf2.transform.translation.x);
            double vel = 0.5*sqrt(pow(tf2.transform.translation.x, 2) + pow(tf2.transform.translation.y, 2));
            if (vel < 1)
            {
                vel = 0;
                angular = 0;
            }

            ROS_WARN_STREAM(tf2.transform);
            ROS_WARN_STREAM(angular << " " << vel);
            vel_msg.angular.z = angular;
            vel_msg.linear.x = vel;
            agv_1_vel.publish(vel_msg);
        }
        catch (const std::exception &r)
        {
            ROS_ERROR_STREAM(r.what());
        }
        rate.sleep();
    }

    return 0;
}