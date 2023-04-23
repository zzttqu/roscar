#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <std_msgs/Int32.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include "multi_agv_sim/agv_status.h"
#include "tf2/utils.h"
#include <math.h>
using namespace multi_agv_sim;
using namespace std;
using namespace geometry_msgs;

#define eps 1e-10
struct Agv_Status
{
    int agv_id;
    int agv_move_status;
    int agv_warning;
    TransformStamped center2agv_tf;
    TransformStamped center2agv_ass_tf;
    TransformStamped error_tf;
    int agv_motor_status;
    double center_radius;
};
// 将收到的状态放入数组中这里是限制agv数量的参数！！！

Agv_Status all_agvs_status[32];
int agv_ass_status, center_set_flag;
// 这个是速度啦
double center_x, center_y, center_yaw;

void center_init_pos_set(const PoseWithCovarianceStamped::ConstPtr &initpose,
                         TransformStamped &center, tf2_ros::TransformBroadcaster &broadcaster)
{
    if (center_set_flag != 0)
    {
        ROS_WARN("您已经设置过了初始位置，请不要再设置");
        return;
    }
    // 发布手动标记的初始化组合位置
    center_set_flag = 1;
    center.header.frame_id = "agv_0/map";
    center.child_frame_id = "agv_ass/base_link";
    center.transform.rotation = initpose->pose.pose.orientation;
    center.transform.translation.x = initpose->pose.pose.position.x;
    center.transform.translation.y = initpose->pose.pose.position.y;
    center.header.stamp = ros::Time::now();
    broadcaster.sendTransform(center);
}

// 接收各个AGV状态
void agv_status_callback(const agv_status::ConstPtr &status, int num)
{
    // ROS_WARN("中央管理节点已收到状态信息");
    agv_ass_status = 0;
    // 按照agv的id给每一个属性赋值
    all_agvs_status[status->agv_id].agv_move_status = status->agv_move_status;
    all_agvs_status[status->agv_id].center2agv_tf = status->center2agv_tf;
    all_agvs_status[status->agv_id].center2agv_ass_tf = status->center2agv_ass_tf;
    all_agvs_status[status->agv_id].error_tf = status->error_tf;
    all_agvs_status[status->agv_id].center_radius = status->center_radius;
    // ROS_INFO_STREAM("现在agv"<<status->agv_id<<"状态" << status->agv_move_status);
    // 只有agv_Status为2倍才是全部锁定了
    for (size_t i = 0; i < num; i++)
    {
        agv_ass_status += all_agvs_status[i].agv_move_status;
    }

    // for (const auto& x:all_agvs_status)
    // {
    //     agv_ass_status += x.agv_move_status;
    // }

    // ROS_INFO_STREAM("现在agv状态" << agv_ass_status);
}
// 接收定位数据
void center_vel_callback(const Twist::ConstPtr &vel_msg)
{

    // 获取中心速度，解算由别的函数干
    center_x = vel_msg.get()->linear.x;
    center_y = vel_msg.get()->linear.y;
    center_yaw = vel_msg.get()->angular.z;
    ROS_INFO_STREAM("中心x速度为" << center_x << " y速度为" << center_y << " z速度为" << center_yaw);
}

void center_cal(string agv_link_names[],
                int num,
                TransformStamped &center,
                tf2_ros::TransformBroadcaster &broadcaster,
                tf2_ros::Buffer &buffer)
{
    double x, y, rz, rx, ry, rw = 0;
    // 也就是已经设置了中心初始坐标
    if (center_set_flag != 2)
    {
        return;
    }

    try
    {
        for (size_t i = 0; i < num; i++)
        {
            TransformStamped map_agv_pos = buffer.lookupTransform("agv_0/map", agv_link_names[i], ros::Time(0));
            x += map_agv_pos.transform.translation.x;
            y += map_agv_pos.transform.translation.y;
            rz += map_agv_pos.transform.rotation.z;
            rw += map_agv_pos.transform.rotation.w;
            ry += map_agv_pos.transform.rotation.y;
            rx += map_agv_pos.transform.rotation.x;
        }
        tf2::Quaternion qtn(rx / num, ry / num, rz / num, rw / num);
        qtn.normalize();
        center.header.frame_id = "agv_0/map";
        center.child_frame_id = "agv_ass/base_link";
        center.transform.rotation.w = qtn.getW();
        center.transform.rotation.x = qtn.getX();
        center.transform.rotation.y = qtn.getY();
        center.transform.rotation.z = qtn.getZ();
        center.transform.translation.x = x / num;
        center.transform.translation.y = y / num;
        center.transform.translation.z = 0;
        center.header.stamp = ros::Time::now();
        broadcaster.sendTransform(center);
        // ROS_WARN("中心坐标计算成功");
    }
    catch (const std::exception &e)
    {
        ROS_ERROR_STREAM("中心坐标计算失败" << e.what() << '\n');
        buffer.clear();
        ros::Duration(0.5).sleep();
    }
    return;
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "cal_center");
    ROS_INFO("agv中心计算节点已启动");
    ros::NodeHandle n;
    int num = 2;
    n.getParam("num", num);
    TransformStamped center;
    tf2_ros::TransformBroadcaster broadcaster;
    tf2_ros::Buffer buffer;
    // 不listen会直接导致超时！！！！
    tf2_ros::TransformListener listener(buffer);
    Twist agv_cmd_vel[num];
    // 订阅中心导航节点
    ros::Subscriber center_vel_topic = n.subscribe<Twist>("agv_ass/cmd_vel", 1, center_vel_callback);

    // 订阅2Dpos初始中心位置
    ros::Subscriber center_init_pos = n.subscribe<PoseWithCovarianceStamped>("/initialpose", 10, boost::bind(center_init_pos_set, _1, boost::ref(center), boost::ref(broadcaster)));

    // 发布中央控制初始化完成消息

    ros::Publisher core_node_status = n.advertise<std_msgs::Int32>("/core_status", 10);

    // 发布各个车的速度信息
    ros::Publisher agvs_vel_topics[num];
    ros::Subscriber agv_status_sub = n.subscribe<agv_status>("/agv_status", 10, boost::bind(agv_status_callback, _1, num));
    stringstream ss;
    ros::Rate rate(10);
    string agv_link_names[num];
    string agv_move_names[num];
    for (size_t i = 0; i < num; i++)
    {
        // 组合各个agv的baselink字符串
        ss << "agv_" << i << "/base_link";
        agv_link_names[i] = ss.str();
        ss.str(string());
        // 接收各个AGV是否到位的消息

        ss.str(string());
        // 做好发布各个agv速度信息的准备
        ss << "agv_" << i << "/cmd_vel";
        agvs_vel_topics[i] = n.advertise<Twist>(ss.str(), 100);
        ss.str(string());
    }

    // ROS_WARN("准备进入循环");

    // ros::Duration(0.5).sleep();
    // 循环次数
    int count = 0;
    while (ros::ok())
    {
        // 处理订阅回调函数
        count++;
        ros::spinOnce();
        if (center_set_flag == 0 && count >= 10)
        {
            center_set_flag = 2;
            ROS_INFO_STREAM("未检测到初始化组合位置，将自动计算组合位置");
            // 计算各车中点
            center_cal(agv_link_names, num, center, broadcaster, buffer);
            std_msgs::Int32 core_status;
            core_status.data = 1;
            // 表示core节点还活着
            core_node_status.publish(core_status);
        }
        else if (center_set_flag == 1 && count < 10)
        {
            ROS_INFO_STREAM("检测到初始化组合位置");
            std_msgs::Int32 core_status;
            core_status.data = 1;
            // 表示core节点还活着
            core_node_status.publish(core_status);
        }
        else if (center_set_flag == 0)
        {
            rate.sleep();
            continue;
        }

        // 全部锁定后就可以启动中心导航了
        if (agv_ass_status == (2 * num))
        {
            // 查询各AGV相对绝对地图的位置然后计算中点
            center_cal(agv_link_names, num, center, broadcaster, buffer);
            for (size_t i = 0; i < num; i++)
            {
                // 就不用时间戳了。。。太长了有点难看，直接就是tf了
                Transform center2agv_tf = all_agvs_status[i].center2agv_tf.transform;
                Transform error_tf = all_agvs_status[i].error_tf.transform;
                // ROS_INFO_STREAM(ass_agv_pos);
                // 因为是从中心指向agv，所以向量要取反
                // 实际状态下
                double real_x = center2agv_tf.translation.x;
                double real_y = center2agv_tf.translation.y;
                // 生成p
                double error_yaw = tf2::getYaw(error_tf.rotation);
                double error_x = error_tf.translation.x;
                double error_y = error_tf.translation.y;

                // 求出角速度，角度不能直接相加的哦，顺时针和逆时针不一样，我也不知道为啥
                double w = center_yaw - 1 * error_yaw;
                // ROS_INFO_STREAM(i <<" "<< error_yaw);
                // 理想状态假定

                // 求出线速度
                double linerx = real_y * w + error_x * 1;
                double linery = real_x * w + error_y * 1;
                // 线速度正交分解
                agv_cmd_vel[i].linear.x = center_x - linerx;
                agv_cmd_vel[i].linear.y = center_y - linery;
                agv_cmd_vel[i].angular.z = w;
            }
            // 解算完毕后发布到各个AGV上
            for (size_t i = 0; i < num; i++)
            {
                agvs_vel_topics[i].publish(agv_cmd_vel[i]);
                // ROS_INFO_STREAM(agv_cmd_vel[i]);
            }
            // 就绪就发个消息
            std_msgs::Int32 core_status;
            core_status.data = 1;
            // 表示core节点还活着
            core_node_status.publish(core_status);
            // 这里应该加一个允许接收目标点的flag，但是现在不需要
        }
        else
        {
            // 开始的时候虽然不计算了但是还要发布，记得更新时间戳
            center.header.stamp = ros::Time::now();
            broadcaster.sendTransform(center);
            std_msgs::Int32 core_status;
            core_status.data = 1;
            // 表示core节点还活着
            core_node_status.publish(core_status);
        }

        rate.sleep();
    }

    return 0;
}
