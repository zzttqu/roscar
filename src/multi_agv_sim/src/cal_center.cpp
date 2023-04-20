#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "cal_center");
    ROS_WARN("agv中心计算节点已启动");
    ros::NodeHandle n;
    tf2_ros::TransformBroadcaster broadcaster;
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);
    int num = 2;
    n.getParam("num", num);
    geometry_msgs::TransformStamped map_agv_pos[num];
    geometry_msgs::TransformStamped center;
    ros::Rate rate(10);
    char agv_name[num][32];
    for (size_t i = 0; i < num; i++)
    {
        sprintf(agv_name[i], "agv_%ld/base_link", i);
    }
    double x, y, yaw;
    // ROS_WARN("准备进入循环");
    // 以10Hz发布n辆车中点
    while (ros::ok())
    {

        // 查询各AGV相对绝对地图的位置
        try
        {
            for (size_t i = 0; i < num; i++)
            {
                map_agv_pos[i] = buffer.lookupTransform("agv_0/map", agv_name[i], ros::Time(0), ros::Duration(0.1));
                x += map_agv_pos[i].transform.translation.x;
                y += map_agv_pos[i].transform.translation.y;
                yaw += map_agv_pos[i].transform.rotation.z;
            }
            tf2::Quaternion qtn;
            qtn.setRPY(0, 0, yaw / num);
            center.header.frame_id = "agv_0/map";
            center.header.stamp = ros::Time::now();
            center.child_frame_id = "agv_ass/base_link";
            center.transform.rotation.w = qtn.getW();
            center.transform.rotation.x = qtn.getX();
            center.transform.rotation.y = qtn.getY();
            center.transform.rotation.z = qtn.getZ();
            center.transform.translation.x = x / num;
            center.transform.translation.y = y / num;
            center.transform.translation.z = 0;
            broadcaster.sendTransform(center);
            // ROS_WARN("中心坐标更新成功");
            x = 0;
            y = 0;
            yaw = 0;
        }
        catch (const std::exception &e)
        {
            ROS_INFO_STREAM(e.what() << '\n');
            buffer.clear();
        }

        rate.sleep();
    }

    return 0;
}
