#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>
// 负责生成AGV1旁边的组合AGV2位置
void agv_pos_tf(int num)
{
    static tf2_ros::StaticTransformBroadcaster broadcaster;
    geometry_msgs::TransformStamped tfs[num];
    tf2::Quaternion qtn;
    for (size_t i = 0; i < num; i++)
    {
        tfs[i].header.seq = 100;
        tfs[i].header.frame_id = "agv_ass/base_link";
        tfs[i].header.stamp = ros::Time::now();
        char agv_id[32];
        snprintf(agv_id, 32, "agv_ass/base_link/agv_%ld", i);
        tfs[i].child_frame_id = agv_id;
        tfs[i].transform.translation.x = 0;
        tfs[i].transform.translation.y = 0.5 * pow(-1, i);
        tfs[i].transform.translation.z = 0;

        qtn.setRPY(0, 0, 0);
        tfs[i].transform.rotation.x = qtn.getX();
        tfs[i].transform.rotation.y = qtn.getY();
        tfs[i].transform.rotation.z = qtn.getZ();
        tfs[i].transform.rotation.w = qtn.getW();
    }
    for (size_t i = 0; i < num; i++)
    {
        broadcaster.sendTransform(tfs[i]);
    }
}
int core_status_flag;
void core_status(const std_msgs::Int32::ConstPtr &core)
{
    core_status_flag = core->data;
}
int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "agv_tf");
    ROS_INFO("组合坐标变换广播已启动");
    ros::NodeHandle n;
    ros::Subscriber agv_status_sub = n.subscribe<std_msgs::Int32>("/core_status", 1, &core_status);
    ros::Rate rate(10);
    // // 如果没消息就一直监听，有消息了就往下走
    // while (ros::ok() && core_status_flag == 0)
    // {
    //     ROS_WARN("静态坐标变换节点等待中央节点就绪");
    //     ros::spinOnce();
    //     rate.sleep();
    // }

    int num = 2;
    n.getParam("num", num);
    agv_pos_tf(num);

    ros::spin();
    return 0;
}