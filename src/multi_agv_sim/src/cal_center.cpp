#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <std_msgs/Int32.h>
#include <tf2_ros/transform_listener.h>
#include "geometry_msgs/Twist.h"
#include "multi_agv_sim/agv_status.h"
using namespace multi_agv_sim;
using namespace std;
using namespace geometry_msgs;
// 将收到的状态放入数组中这里是限制agv数量的参数！！！

struct Agv_Status
{
    int agv_id;
    int agv_move_status;
    int agv_warning;
    TransformStamped center2agv_tf;
    int agv_motor_status;
};
// 只有agv_Status为2倍才是全部锁定了
Agv_Status all_agvs_status[32];
int agv_ass_status;
double center_x, center_y, center_yaw;
// 接收各个AGV状态
void agv_status_callback(const agv_status::ConstPtr &status, int num)
{
    // ROS_WARN("中央管理节点已收到状态信息");
    agv_ass_status = 0;
    //按照agv的id给每一个属性赋值
    all_agvs_status[status->agv_id].agv_move_status = status->agv_move_status;
    all_agvs_status[status->agv_id].center2agv_tf = status->center2agv_tf;
    // ROS_INFO_STREAM("现在agv"<<status->agv_id<<"状态" << status->agv_move_status);
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
// 接收定位数据并进行运动解算
void center_vel_callback(const geometry_msgs::Twist::ConstPtr &vel_msg)
{
    // // 求得各车相对中心坐标的位置
    // geometry_msgs::TransformStamped map_agv_pos[num];
    // for (size_t i = 0; i < num; i++)
    // {
    //     map_agv_pos[i] = buffer.lookupTransform("agv_ass/base_link", agv_link_names[i], ros::Time(0));
    // }
    // buffer.lookupTransform("agv_ass/base_link", "agv_ass/base_link/agv_0", ros::Time(0));
    // 获取中心速度，解算由别的函数干
    center_x = vel_msg.get()->linear.x;
    center_y = vel_msg.get()->linear.y;
    center_yaw = vel_msg.get()->angular.z;
    ROS_INFO_STREAM("x速度为" << center_x << "x速度为" << center_y << "x速度为" << center_yaw);
}

void center_cal(string agv_link_names[],
                int num,
                geometry_msgs::TransformStamped map_agv_pos[],
                geometry_msgs::TransformStamped &center,
                tf2_ros::TransformBroadcaster &broadcaster,
                tf2_ros::Buffer &buffer)
{
    double x, y, yaw;
    try
    {
        for (size_t i = 0; i < num; i++)
        {
            map_agv_pos[i] = buffer.lookupTransform("agv_0/map", agv_link_names[i], ros::Time(0));
            x += map_agv_pos[i].transform.translation.x;
            y += map_agv_pos[i].transform.translation.y;
            yaw += map_agv_pos[i].transform.rotation.z;
        }
        tf2::Quaternion qtn;
        qtn.setRPY(0, 0, yaw / num);
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
        x = 0;
        y = 0;
        yaw = 0;
    }
    catch (const std::exception &e)
    {
        ROS_ERROR_STREAM("中心坐标计算失败" << e.what() << '\n');
        buffer.clear();
        ros::Duration(0.5).sleep();
    }
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "cal_center");
    ROS_WARN("agv中心计算节点已启动");
    ros::NodeHandle n;
    int num = 2;
    n.getParam("num", num);
    geometry_msgs::TransformStamped map_agv_pos[num];
    geometry_msgs::TransformStamped center;
    tf2_ros::TransformBroadcaster broadcaster;
    tf2_ros::Buffer buffer;
    // 不listen会直接导致超时！！！！
    tf2_ros::TransformListener listener(buffer);
    geometry_msgs::Twist agv_cmd_vel[num];
    // 订阅中心导航节点
    ros::Subscriber center_vel_topic = n.subscribe<geometry_msgs::Twist>("agv_ass/cmd_vel", 10, center_vel_callback);

    // 发布各个车的速度信息
    ros::Publisher agvs_vel_topics[num];
    ros::Subscriber agv_status_sub = n.subscribe<agv_status>("/agv_status", 10, boost::bind(&agv_status_callback, _1, num));
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
        agvs_vel_topics[i] = n.advertise<geometry_msgs::Twist>(ss.str(), 100);
        ss.str(string());
    }

    // boost::bind(agv_status_callback, _1, i);
    // ROS_WARN("准备进入循环");
    // 计算各车中点
    ros::Duration(0.5).sleep();
    center_cal(agv_link_names, num, map_agv_pos, center, broadcaster, buffer);
    while (ros::ok())
    {
        // 处理订阅回调函数
        ros::spinOnce();

        // 全部锁定后就可以启动中心导航了
        if (agv_ass_status == (2 * num))
        {
            // 查询各AGV相对绝对地图的位置然后计算中点
            center_cal(agv_link_names, num, map_agv_pos, center, broadcaster, buffer);
            geometry_msgs::TransformStamped ass_agv_pos;
            for (size_t i = 0; i < num; i++)
            {
                // 运动解算有问题
                ass_agv_pos = all_agvs_status[i].center2agv_tf;
                // ROS_INFO_STREAM(ass_agv_pos);
                agv_cmd_vel[i].linear.x = center_x - center_yaw * ass_agv_pos.transform.translation.y;
                agv_cmd_vel[i].linear.y = center_y - center_yaw * ass_agv_pos.transform.translation.x;
                agv_cmd_vel[i].angular.z = center_yaw;
            }
            // 解算完毕后发布到各个AGV上
            for (size_t i = 0; i < num; i++)
            {
                agvs_vel_topics[i].publish(agv_cmd_vel[i]);
                // ROS_INFO_STREAM(agv_cmd_vel[i]);
            }

            // 这里应该加一个允许接收目标点的flag，但是现在不需要
        }
        else
        {
            // 开始的时候虽然不计算了但是还要发布，记得更新时间戳
            center.header.stamp = ros::Time::now();
            broadcaster.sendTransform(center);
        }

        rate.sleep();
    }

    return 0;
}
