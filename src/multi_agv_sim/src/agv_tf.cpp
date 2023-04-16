#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"
int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "agv_tf");
    ROS_INFO("坐标变换已启动");
    ros::NodeHandle n;
    tf2_ros::TransformBroadcaster broadcaster;
    ros::Rate rate(50);
    float y = 0.25;
    while (ros::ok())
    {
        
        geometry_msgs::TransformStamped tfs;
        // tfs.header.seq=100;
        tfs.header.frame_id = "base_link";
        tfs.header.stamp = ros::Time::now();

        tfs.child_frame_id = "multi_center";
        tfs.transform.translation.x = 0.0;
        tfs.transform.translation.y = y;
        tfs.transform.translation.z = 0.0;

        tf2::Quaternion qtn;
        qtn.setRPY(0, 0, 0);
        tfs.transform.rotation.x = qtn.getX();
        tfs.transform.rotation.y = qtn.getY();
        tfs.transform.rotation.z = qtn.getZ();
        tfs.transform.rotation.w = qtn.getW();
        broadcaster.sendTransform(tfs);
        rate.sleep();
    }

    return 0;
}