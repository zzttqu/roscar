#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Twist.h"
#include "tf2/LinearMath/Quaternion.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "tf_listener");
    ROS_ERROR("坐标变换订阅已启动");
    ros::NodeHandle n;
    ros::Publisher agv_1_vel = n.advertise<geometry_msgs::Twist>("agv_1/cmd_vel", 10);

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient("agv_1/move_base",true);
    move_base_msgs::MoveBaseGoal goal;

    tf2_ros::TransformBroadcaster broadcaster;
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);
    ros::Rate rate(5);
    goal.target_pose.header.frame_id="map";
    while (ros::ok())
    {

        try
        {
            geometry_msgs::TransformStamped tf2;
            tf2 = buffer.lookupTransform("map", "agv_1/ass_pos", ros::Time(0));
            geometry_msgs::Twist vel_msg;
            double angular = 4*atan2(tf2.transform.translation.y, tf2.transform.translation.x);
            double vel = 1*sqrt(pow(tf2.transform.translation.x, 2) + pow(tf2.transform.translation.y, 2));
            if (vel < 0.01)
            {
                vel = 0;
                angular = 0;
            }
            goal.target_pose.pose.position.x=tf2.transform.translation.x;
            goal.target_pose.pose.position.y=tf2.transform.translation.y;
            goal.target_pose.pose.orientation.z=tf2.transform.rotation.z;
            goal.target_pose.pose.orientation.w=tf2.transform.rotation.w;
            goal.target_pose.header.stamp=ros::Time::now();
            // ROS_WARN_STREAM(tf2.transform);
            // ROS_WARN_STREAM(angular << " " << vel);
            vel_msg.angular.z = angular;
            vel_msg.linear.x = vel;
            MoveBaseClient.sendGoal(goal);
            // agv_1_vel.publish(vel_msg);
        }
        catch (const std::exception &r)
        {
            ROS_ERROR_STREAM(r.what());
        }
        rate.sleep();
    }

    return 0;
}