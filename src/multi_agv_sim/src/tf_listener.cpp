#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Twist.h"
#include <std_msgs/Int32.h>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "actionlib/client/simple_goal_state.h"
#include "multi_agv_sim/agv_status.h"
using namespace multi_agv_sim;
using namespace actionlib;
using namespace std;
int core_status_flag;
void core_status(const std_msgs::Int32::ConstPtr &core)
{
    core_status_flag = core->data;
}
// 0是未抵达状态，1是已经抵达，-1是无法抵达，2是已经锁定，订阅center并使用坐标变换
int assamble_status = -1;

void nav_to_pos(
    int agv_id,
    SimpleActionClient<move_base_msgs::MoveBaseAction> &MoveBaseClient,
    move_base_msgs::MoveBaseGoal &goal,
    tf2_ros::Buffer &buffer, string goal_link)
{
    try
    {
        geometry_msgs::TransformStamped nav_goal_tf = buffer.lookupTransform("agv_0/map", goal_link, ros::Time(0));
        goal.target_pose.pose.position.x = nav_goal_tf.transform.translation.x;
        goal.target_pose.pose.position.y = nav_goal_tf.transform.translation.y;
        goal.target_pose.pose.orientation.z = nav_goal_tf.transform.rotation.z;
        goal.target_pose.pose.orientation.w = nav_goal_tf.transform.rotation.w;
        goal.target_pose.header.stamp = ros::Time::now();
        MoveBaseClient.sendGoal(goal);
        // 5s后如果没获得结果就算超时
        MoveBaseClient.waitForResult(ros::Duration(5));
        if (MoveBaseClient.getState() == SimpleClientGoalState::SUCCEEDED)
        {
            // ROS_INFO_STREAM(agv_id << "号AGV已到达预定位置");
            assamble_status = 1;
        }
        else if (SimpleClientGoalState::ACTIVE)
        {
            ROS_INFO_STREAM(agv_id << "号AGV正在前往预定位置");
            assamble_status = 0;
        }
        else if (SimpleClientGoalState::PENDING)
        {
            MoveBaseClient.sendGoal(goal);
            ROS_WARN_STREAM(agv_id << "号AGV未收到坐标信息,已重新发送");
        }
        else
        {
            assamble_status = -1;
            ROS_ERROR_STREAM(agv_id << "号AGV无法到达预定位置");
            MoveBaseClient.cancelAllGoals();
        }
    }
    catch (const std::exception &e)
    {
        ROS_ERROR_STREAM(agv_id << "号AGV无法到达预定位置" << e.what());
        MoveBaseClient.cancelAllGoals();
    }
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "tf_listener");
    ROS_INFO("agv控制节点已启动");
    ros::NodeHandle n;
    // 发布当前AGV状态
    agv_status this_agv_status;
    move_base_msgs::MoveBaseGoal goal;
    // 发布自身状态
    ros::Publisher move_status = n.advertise<agv_status>("/agv_status", 10);
    int agv_id = n.getNamespace().back() - '0'; // 取出agv的id
    this_agv_status.header.frame_id = n.getNamespace();
    this_agv_status.agv_id = agv_id;
    ros::Publisher agv_vel = n.advertise<geometry_msgs::Twist>("cmd_vel", 100);
    // ros::Subscriber agv_0_vel = n.subscribe<geometry_msgs::Twist>("agv_0/cmd_vel", 1000, subscribe);
    SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient("move_base", true);
    // 订阅中央节点的状态
    ros::Subscriber agv_status_sub = n.subscribe<std_msgs::Int32>("/core_status", 10, &core_status);

    tf2_ros::TransformBroadcaster broadcaster;
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);
    ros::Rate rate(10);

    goal.target_pose.header.frame_id = "agv_0/map";
    geometry_msgs::TransformStamped nav_goal_tf;

    stringstream ass_pos;
    ass_pos << "agv_ass/base_link/agv_" << agv_id;
    stringstream agv_pos;
    agv_pos << "agv_" << agv_id << "/base_link";

    // try
    // {
    //     nav_to_pos(agv_id, MoveBaseClient, goal, buffer, ass_pos.str());
    // }
    // catch (const std::exception &e)
    // {
    //     ROS_ERROR_STREAM("导航点发布失败" << e.what() << '\n');
    //     MoveBaseClient.cancelAllGoals();
    //     ros::Duration(0.5).sleep();
    // }

    // this_agv_status.agv_move_status = assamble_status;
    // this_agv_status.header.stamp = ros::Time::now();
    // move_status.publish(this_agv_status);
    // ROS_ERROR("准备进入循环");
    ros::Duration(1).sleep();
    ROS_INFO("agv控制节点已正式启动");
    int ass_pos_count = 0;
    int count = 0;
    while (ros::ok())
    {
        ros::spinOnce();
        if (core_status_flag != 1)
        {
            ROS_WARN("agv%d控制节点未检测中央节点就绪", agv_id);
            rate.sleep();
            continue;
        }
        else if (core_status_flag == 1 && count < 2)
        {
            count++;
            ROS_INFO("agv%d控制节点已连接中央节点", agv_id);
        }
        // 未设定目标
        if (assamble_status == -1)
        {
            try
            {
                nav_to_pos(agv_id, MoveBaseClient, goal, buffer, ass_pos.str());
            }
            catch (const std::exception &e)
            {
                ROS_ERROR_STREAM("导航点发布失败" << e.what() << '\n');
                MoveBaseClient.cancelAllGoals();
                ros::Duration(0.5).sleep();
            }
        }
        // 正在前往
        if (assamble_status == 0)
        {
            // MoveBaseClient.waitForResult(ros::Duration(10));
            if (MoveBaseClient.getState() == SimpleClientGoalState::SUCCEEDED)
            {
                assamble_status = 1;
            }
            else if (SimpleClientGoalState::ACTIVE)
            {
                // ROS_INFO_STREAM(agv_id << "正在前往预定位置");
                assamble_status = 0;
            }
            else if (SimpleClientGoalState::PENDING)
            {
                MoveBaseClient.sendGoal(goal);
                ROS_WARN_STREAM(agv_id << "号AGV未收到坐标信息，已重新发送");
            }
            else
            {
                assamble_status = -1;
                ROS_ERROR_STREAM(agv_id << "无法到达预定位置");
            }
            // ROS_WARN_STREAM(agv_id << "号AGV已发送状态信息");
        }

        // 进入锁定状态
        // 迅速对齐，稳定后在判定为锁定成功
        if (assamble_status == 1 && ass_pos_count < 3)
        {
            try
            {
                // 这里改成了固定的
                geometry_msgs::TransformStamped agv2_asspos = buffer.lookupTransform(agv_pos.str(), ass_pos.str(), ros::Time(0));
                geometry_msgs::Twist vel_msg;
                // 需要把四元数转化出来
                double angular = tf2::getYaw(agv2_asspos.transform.rotation);
                double vel = 1 * sqrt(pow(agv2_asspos.transform.translation.x, 2) + pow(agv2_asspos.transform.translation.y, 2));
                vel_msg.linear.x = vel;
                if (vel < 0.04 && angular < 0.01)
                {
                    vel_msg.linear.x = 0;
                    vel_msg.linear.y = 0;
                    vel_msg.angular.z = 0;
                    ass_pos_count++;
                }
                else
                {
                    vel_msg.linear.x = 1 * agv2_asspos.transform.translation.x;
                    vel_msg.linear.y = 1 * agv2_asspos.transform.translation.y;
                    vel_msg.angular.z = 4 * tf2::getYaw(agv2_asspos.transform.rotation);
                    agv_vel.publish(vel_msg);
                }
            }
            catch (const std::exception &e)
            {
                ROS_ERROR_STREAM("迅速对齐失败" << e.what() << '\n');
                ros::Duration(0.5).sleep();
            }
        }
        if (assamble_status == 1 && ass_pos_count >= 3)
        {
            geometry_msgs::Twist vel_msg;
            vel_msg.linear.x = 0;
            vel_msg.linear.y = 0;
            vel_msg.angular.z = 0;
            agv_vel.publish(vel_msg);
            // 锁定成功
            assamble_status = 2;
            ROS_INFO_STREAM(agv_id << "号AGV已到达预定位置");
        }
        // 锁定后，主要发布坐标变换，毕竟子车可以承担一点坐标变换的工作量
        if (assamble_status == 2)
        {
            assamble_status = 2;
        }
        try
        {
            this_agv_status.center2agv_ass_tf = buffer.lookupTransform("agv_ass/base_link", ass_pos.str(), ros::Time(0));
            // 找到实际坐标和生成坐标的误差
            this_agv_status.error_tf = buffer.lookupTransform(ass_pos.str(), agv_pos.str(), ros::Time(0));
            auto x = this_agv_status.center2agv_ass_tf.transform.translation.x;
            auto y = this_agv_status.center2agv_ass_tf.transform.translation.y;
            this_agv_status.center_radius = sqrt(x * x + y * y);
            this_agv_status.center2agv_tf = buffer.lookupTransform("agv_ass/base_link", agv_pos.str(), ros::Time(0));
        }
        catch (const std::exception &e)
        {
            ROS_ERROR_STREAM("未找到坐标变换" << e.what());
        }
        this_agv_status.agv_move_status = assamble_status;
        this_agv_status.header.stamp = ros::Time::now();
        move_status.publish(this_agv_status);
        // ROS_INFO_STREAM(this_agv_status.agv_id<<"  "<<this_agv_status.agv_move_status);
        rate.sleep();
    }

    return 0;
}