#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"
#include <nav_msgs/Odometry.h>
//负责生成AGV1旁边的组合AGV2位置
void agv_pos_tf()
{
    static tf2_ros::TransformBroadcaster broadcaster;
    geometry_msgs::TransformStamped tfs;
    // tfs.header.seq=100;
    tfs.header.frame_id = "agv_0/base_link";
    tfs.header.stamp = ros::Time::now();

    tfs.child_frame_id = "agv_1/ass_pos";
    tfs.transform.translation.x = 0;
    tfs.transform.translation.y = -0.3;
    tfs.transform.translation.z = 0;

    tf2::Quaternion qtn;
    qtn.setRPY(0,0,0);
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
    ROS_ERROR("坐标变换广播已启动");
    ros::NodeHandle n;

    ros::Rate rate(10);
    float y = 0.25;
    while (ros::ok())
    {
        agv_pos_tf();
        rate.sleep();
    }
    ros::spin();
    return 0;
}