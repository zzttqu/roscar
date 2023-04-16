#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"
int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    setlocale(LC_CTYPE, "zh_CN.utf8");
    ros::init(argc, argv, "agv_tf");
    ros::NodeHandle n;
    static tf2_ros::TransformBroadcaster broadcaster;
    geometry_msgs::TransformStamped tfs;
    tfs.header.frame_id="base_link";
    tfs.header.stamp=ros::Time::now();
    tfs.child_frame_id="multi_center";
    tfs.transform.translation.x=0.0;
    tfs.transform.translation.y=0.0;
    tfs.transform.translation.z=0.0;

    tf2::Quaternion qtn;
    qtn.setRPY(0,0,0);
    tfs.transform.rotation.x=qtn.getX();
    
    return 0;
}