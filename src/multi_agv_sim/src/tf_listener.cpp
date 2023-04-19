#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Twist.h"
#include "tf2/LinearMath/Quaternion.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "actionlib/client/simple_goal_state.h"
using namespace actionlib;
void subscribe(){

}
// 0是未抵达状态，1是已经抵达，-1是无法抵达
int assamble_status = 0;
int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "tf_listener");
    ROS_ERROR("agv2控制节点已启动");
    ros::NodeHandle n;
    ros::Publisher agv_1_vel = n.advertise<geometry_msgs::Twist>("agv_1/cmd_vel", 10);
    //ros::Subscriber agv_0_vel=n.subscribe<geometry_msgs::Twist>("agv_10cmd_vel", 10,subscribe);

    SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient("agv_1/move_base", true);
    move_base_msgs::MoveBaseGoal goal;

    tf2_ros::TransformBroadcaster broadcaster;
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);
    ros::Rate rate(5);
    ros::Duration(1).sleep();
    ROS_ERROR("agv2控制节点已正式启动");
    goal.target_pose.header.frame_id = "map";
    try
    {
        // 初始状态下先过去
        geometry_msgs::TransformStamped map_asspos = buffer.lookupTransform("map", "agv_1/ass_pos", ros::Time(0));
        goal.target_pose.pose.position.x = map_asspos.transform.translation.x;
        goal.target_pose.pose.position.y = map_asspos.transform.translation.y;
        goal.target_pose.pose.orientation.z = map_asspos.transform.rotation.z;
        goal.target_pose.pose.orientation.w = map_asspos.transform.rotation.w;
        goal.target_pose.header.stamp = ros::Time::now();

        MoveBaseClient.sendGoal(goal);
        // 30s后如果没获得结果就算超时
        MoveBaseClient.waitForResult(ros::Duration(5));
        if (MoveBaseClient.getState() == SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("AGV2已到达预定位置");
            assamble_status = 1;
        }
        else if (SimpleClientGoalState::ACTIVE)
        {
            ROS_INFO("AGV2正在前往预定位置");
            assamble_status = 0;
        }
        else if (SimpleClientGoalState::PENDING)
        {
            MoveBaseClient.sendGoal(goal);
            ROS_WARN("AGV2未收到坐标信息，已重新发送");
        }
        else
        {
            assamble_status = -1;
            ROS_ERROR("AGV2无法到达预定位置");
        }
    }
    catch (const std::exception &e)
    {
        ROS_ERROR_STREAM(e.what());
    }
    ROS_ERROR("准备进入循环");
    while (ros::ok())
    {
        try
        {
            // 未设定目标
            if (assamble_status == -1)
            {
                geometry_msgs::TransformStamped map_asspos = buffer.lookupTransform("map", "agv_1/ass_pos", ros::Time(0));
                goal.target_pose.pose.position.x = map_asspos.transform.translation.x;
                goal.target_pose.pose.position.y = map_asspos.transform.translation.y;
                goal.target_pose.pose.orientation.z = map_asspos.transform.rotation.z;
                goal.target_pose.pose.orientation.w = map_asspos.transform.rotation.w;
                goal.target_pose.header.stamp = ros::Time::now();
                // ROS_WARN_STREAM(tf2.transform);
                // ROS_WARN_STREAM(angular << " " << vel);

                MoveBaseClient.sendGoal(goal);

                MoveBaseClient.waitForResult(ros::Duration(10));

                if (MoveBaseClient.getState() == SimpleClientGoalState::SUCCEEDED)
                {
                    ROS_INFO("AGV2已到达预定位置");
                    assamble_status = 1;
                }
                else if (SimpleClientGoalState::ACTIVE)
                {
                    ROS_WARN("AGV2还未抵达预定位置");
                    assamble_status = 0;
                }
                else if (SimpleClientGoalState::PENDING)
                {
                    MoveBaseClient.sendGoal(goal);
                    ROS_WARN("AGV2未收到坐标信息，已重新发送");
                }
                else
                {
                    ROS_ERROR("AGV2无法到达预定位置");
                    assamble_status = -1;
                }
            }
            // 正在前往
            if (assamble_status == 0)
            {
                geometry_msgs::TransformStamped map_asspos = buffer.lookupTransform("map", "agv_1/ass_pos", ros::Time(0));
                // 这里可以判断一下距离还有多远，或者目标点是否有变化
                //  double a[3];
                //  if((goal.target_pose.pose.position.x-map_asspos.transform.translation.x)>0.1){

                // }
                goal.target_pose.pose.position.x = map_asspos.transform.translation.x;
                goal.target_pose.pose.position.y = map_asspos.transform.translation.y;
                goal.target_pose.pose.orientation.z = map_asspos.transform.rotation.z;
                goal.target_pose.pose.orientation.w = map_asspos.transform.rotation.w;
                goal.target_pose.header.stamp = ros::Time::now();
                // MoveBaseClient.waitForResult(ros::Duration(10));
                SimpleClientGoalState Goalstate = MoveBaseClient.getState();
                if (MoveBaseClient.getState() == SimpleClientGoalState::SUCCEEDED)
                {
                    ROS_INFO("AGV2已到达预定位置");
                    assamble_status = 1;
                }
                else if (SimpleClientGoalState::ACTIVE)
                {
                    ROS_WARN("AGV2正在前往预定位置");
                    assamble_status = 0;
                }
                else
                {
                    ROS_ERROR("AGV2无法到达预定位置");
                    assamble_status = -1;
                }
            }

            // 进入锁定状态
            if (assamble_status == 1)
            {
                geometry_msgs::TransformStamped agv2_asspos = buffer.lookupTransform("agv_1/base_link", "agv_1/ass_pos", ros::Time(0));
                geometry_msgs::Twist vel_msg;
                double angular = agv2_asspos.transform.rotation.z;

                double vel = 1 * sqrt(pow(agv2_asspos.transform.translation.x, 2) + pow(agv2_asspos.transform.translation.y, 2));
                vel_msg.linear.x = vel;
                if (vel < 0.04 && angular < 0.01)
                {
                    vel_msg.linear.x = 0;
                    vel_msg.linear.y = 0;
                    vel_msg.angular.z = 0;
                }
                else
                {
                    vel_msg.linear.x = 5 * agv2_asspos.transform.translation.x;
                    vel_msg.linear.y = 5 * agv2_asspos.transform.translation.y;
                    vel_msg.angular.z = 8 * agv2_asspos.transform.rotation.z;
                }
                agv_1_vel.publish(vel_msg);
            }
        }
        catch (const std::exception &r)
        {
            ROS_ERROR_STREAM(r.what());
        }
        rate.sleep();
    }

    return 0;
}