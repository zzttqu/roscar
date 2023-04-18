#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"
#include <nav_msgs/Odometry.h>
//负责将agv1转化到agv0的地图下
void agv_pos_tf(const nav_msgs::Odometry::ConstPtr &msg)
{
    static tf2_ros::TransformBroadcaster broadcaster;
    geometry_msgs::TransformStamped tfs;
    // tfs.header.seq=100;
    tfs.header.frame_id = "agv_0/map";
    tfs.header.stamp = ros::Time::now();

    tfs.child_frame_id = "agv_1/odom";
    tfs.transform.translation.x = msg->pose.pose.position.x;
    tfs.transform.translation.y = msg->pose.pose.position.y;
    tfs.transform.translation.z = msg->pose.pose.position.x;

    tf2::Quaternion qtn;
    qtn.setRPY(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    tfs.transform.rotation.x = qtn.getX();
    tfs.transform.rotation.y = qtn.getY();
    tfs.transform.rotation.z = qtn.getZ();
    tfs.transform.rotation.w = qtn.getW();
    broadcaster.sendTransform(tfs);
    // ROS_ERROR_STREAM(tfs.transform);
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "agv_tf");
    ROS_ERROR("坐标变换已启动");
    ros::NodeHandle n;

    ros::Rate rate(10);
    float y = 0.25;
    ros::Subscriber sub = n.subscribe<nav_msgs::Odometry>("agv_1/odom",1000, agv_pos_tf);
    // while (ros::ok())
    // {

    //     rate.sleep();
    // }
    ros::spin();
    return 0;
}