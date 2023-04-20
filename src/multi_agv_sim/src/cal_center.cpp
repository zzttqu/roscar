#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <std_msgs/Int32.h>
#include <tf2_ros/transform_listener.h>
using namespace std;
// 将收到的状态放入数组中
int agvs_status[32];
// 只有agv_Status为2倍才是全部锁定了
int agv_Status;
stringstream ss;
void agv_status_callback(const std_msgs::Int32::ConstPtr &status, int i)
{
    agv_Status = 0;
    agvs_status[i] = status->data;
    for (auto x : agvs_status)
    {
        agv_Status += x;
    }
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
        center.header.stamp = ros::Time::now();
        center.child_frame_id = "agv_ass/base_link";
        center.header.seq = 10;
        center.transform.rotation.w = qtn.getW();
        center.transform.rotation.x = qtn.getX();
        center.transform.rotation.y = qtn.getY();
        center.transform.rotation.z = qtn.getZ();
        center.transform.translation.x = x / num;
        center.transform.translation.y = y / num;
        center.transform.translation.z = 0;
        broadcaster.sendTransform(center);
        ROS_WARN("中心坐标计算成功");
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
    ros::Subscriber agvs_status_sub[num];
    geometry_msgs::TransformStamped center;
    tf2_ros::TransformBroadcaster broadcaster;
    tf2_ros::Buffer buffer;
    // 不listen会直接导致超时！！！！
    tf2_ros::TransformListener listener(buffer);
    ros::Rate rate(10);
    string agv_link_names[num];
    string agv_move_names[num];
    for (size_t i = 0; i < num; i++)
    {
        ss << "agv_" << i << "/base_link";
        agv_link_names[i] = ss.str();
        ss.str(string());
        ss << "agv_" << i << "/move_status";
        agvs_status_sub[i] = n.subscribe<std_msgs::Int32>(ss.str(), 10, boost::bind(agv_status_callback, _1, i));
        ss.str(string());
    }

    // ROS_WARN("准备进入循环");
    // 计算各车中点
    ros::Duration(0.5).sleep();
    center_cal(agv_link_names, num, map_agv_pos, center, broadcaster, buffer);
    while (ros::ok())
    {
        // 之后虽然不计算了但是还要发布，记得更新时间戳
        center.header.stamp = ros::Time::now();
        broadcaster.sendTransform(center);

        // 全部锁定后就可以启动中心导航了
        if (agv_Status == (2 * num))
        {
            // 这里应该加一个允许接收目标点的flag，但是现在不需要
        }

        // 查询各AGV相对绝对地图的位置

        rate.sleep();
    }
    ros::spin();
    return 0;
}
