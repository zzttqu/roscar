#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <tf2_ros/transform_listener.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int32.h>
#include "tf2/utils.h"
#include <move_base_msgs/MoveBaseAction.h>

nav_msgs::OccupancyGrid filtered_map;
sensor_msgs::LaserScan filtered_scan;
int core_status_flag;
// void feedback_fun(const actionlib_msgs::GoalStatusArray::ConstPtr &feedback)
// {
//   feedback->status_list;
//   for (const auto &i : feedback->status_list)
//   {
//     // ROS_INFO_STREAM(i);
//   }
// }
// void goal_fun(const move_base_msgs::MoveBaseActionGoal::ConstPtr &feedback)
// {
//   // ROS_INFO_STREAM(feedback->goal );
// }

void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
  // 复制原始数据
  filtered_scan = *scan_msg;
}
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &map)
{
  // 当地图更新后
  filtered_map = *map;
  // ROS_INFO_STREAM_THROTTLE(1, "agv0" << map->header << " \r\n"
  //                                    << map->info);
}

void core_status(const std_msgs::Int32::ConstPtr &core)
{
  core_status_flag = core->data;
}
// 矩形
auto setCostToZero(nav_msgs::OccupancyGrid &map, double x1, double y1, double x2, double y2)
{
  // 按照坐标大小排成一个方块
  double min_x = std::min(x1, x2);
  double max_x = std::max(x1, x2);
  double min_y = std::min(y1, y2);
  double max_y = std::max(y1, y2);

  // 左上角
  int start_x = (int)((min_x - map.info.origin.position.x) / map.info.resolution);
  int start_y = (int)((min_y - map.info.origin.position.y) / map.info.resolution);

  // 右下角
  int end_x = (int)((max_x - map.info.origin.position.x) / map.info.resolution);
  int end_y = (int)((max_y - map.info.origin.position.y) / map.info.resolution);

  // 矩形内的都换成指定数值
  for (int x = start_x; x <= end_x; x++)
  {
    for (int y = start_y; y <= end_y; y++)
    {
      int index = x + y * map.info.width;
      if (index >= 0 && index < map.data.size())
      {
        map.data[index] = 100;
      }
    }
  }
  return map;
}
// 圆形
auto setCostToZero(nav_msgs::OccupancyGrid &map, double center_x, double center_y, double radius)
{
  // 圆心转换坐标
  int center_i = (int)((center_x - map.info.origin.position.x) / map.info.resolution);
  int center_j = (int)((center_y - map.info.origin.position.y) / map.info.resolution);

  // 计算在图里半径多大，平方方便计算
  double radius_cells_squared = std::pow(radius / map.info.resolution, 2);
  // Set the cost of all cells within the circle to 0
  for (int i = 0; i < map.info.width; i++)
  {
    for (int j = 0; j < map.info.height; j++)
    {
      double dx = i - center_i;
      double dy = j - center_j;
      // 这里就不用开方了
      double distance_squared = dx * dx + dy * dy;
      if (distance_squared <= radius_cells_squared)
      {
        int index = i + j * map.info.width;
        if (index >= 0 && index < map.data.size())
        {
          // map.data[index] = map.data[index];
          // ROS_INFO_STREAM_THROTTLE(1, "滤波地图" << map.data[index] << "数据位置" << index);
        }
      }
    }
  }
  map.header.stamp = ros::Time::now();
  // ROS_INFO_STREAM_THROTTLE(1, "滤波地图" << map.header << map.info);
  return map;
}

sensor_msgs::LaserScan setScanToNaN(sensor_msgs::LaserScan &filtered_scan, float angle)
{
  // 设置删除数据的角度范围

  // 遍历激光雷达数据，并将需要删除的数据设置为NaN
  // 这里是角度制度
  double an = (angle * 180.0 / M_PI);
  // ROS_INFO_STREAM("相对角度为" << an);
  an = 180 + an;
  an = fmod(an, 360.0);
  if (an < 0)
  {
    an += 360.0;
  }
  if (0 < an && an < 15)
  {
    an = 15;
  }
  if (344 < an)
  {
    an = 344;
  }
  int start_index = ceil(an - 15);

  int end_index = floor(an + 15);
  // 数据总共就360个。。。
  // ROS_INFO_STREAM(start_index << " " << end_index << " " << filtered_scan.ranges.size());

  // 要等信息来了之后再运行，要不然直接爆了，因为除以0
  // 不能用负值啊

  for (size_t i = start_index; i < end_index; i++)
  {
    if (filtered_scan.ranges[i] < 3)
    {
      filtered_scan.ranges[i] = std::numeric_limits<float>::infinity();
    }
  }
  return filtered_scan;
}

int main(int argc, char **argv)
{
  setlocale(LC_ALL, "");
  ros::init(argc, argv, "clear_map_region");
  ROS_INFO("滤波节点启动成功");
  // 地图清理节点开始运行
  ros::NodeHandle n;
  tf2_ros::Buffer buffer;
  // 不listen会直接导致超时！！！！
  tf2_ros::TransformListener listener(buffer);

  ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("scan", 1);
  ros::Publisher map_pub = n.advertise<nav_msgs::OccupancyGrid>("map", 1);

  // 订阅地图话题
  ros::Subscriber sub_scan = n.subscribe<sensor_msgs::LaserScan>("/agv_0/scan", 1, scanCallback);
  ros::Subscriber sub_map = n.subscribe<nav_msgs::OccupancyGrid>("/agv_0/map", 1, mapCallback);
  // 订阅中央管理节点
  ros::Subscriber agv_status_sub = n.subscribe<std_msgs::Int32>("/core_status", 10, core_status);

  //
  // ros::Subscriber feedback_sub = n.subscribe<actionlib_msgs::GoalStatusArray>("/move_base/status", 1, feedback_fun);
  // ros::Subscriber goal_sub = n.subscribe<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 1, goal_fun);
  ros::Rate rate(10);
  ros::Duration(1).sleep();
  int count;
  while (ros::ok())
  {
    ros::spinOnce();
    if (core_status_flag != 1)
    {
      ROS_WARN("地图滤波节点未检测中央节点上线");
      rate.sleep();
      continue;
    }
    if (core_status_flag == 1)
    {
      // 发布修改后的地图话题
      try
      {
        geometry_msgs::TransformStamped map_ass_pos = buffer.lookupTransform("agv_0/map", "agv_ass/base_link", ros::Time(0));
        // 知道AGV0向AGV1的角度就可以屏蔽附近的激光雷达数据了
        geometry_msgs::TransformStamped agv_02agv_1 = buffer.lookupTransform("agv_0/base_link", "agv_1/base_link", ros::Time(0));
        double x = map_ass_pos.transform.translation.x;
        double y = map_ass_pos.transform.translation.y;
        nav_msgs::OccupancyGrid filtered_map_pub = setCostToZero(filtered_map, x, y, 1);
        // ROS_INFO("发布了滤波地图");
        map_pub.publish(filtered_map_pub);
        x = agv_02agv_1.transform.translation.x;
        y = agv_02agv_1.transform.translation.y;
        float mid_angle = atan2(y, x);
        setScanToNaN(filtered_scan, mid_angle);
        scan_pub.publish(filtered_scan);
        // ROS_INFO("发布了滤波雷达");
      }
      catch (const std::exception &e)
      {
        ROS_ERROR_STREAM("地图生成失败" << e.what() << '\n');
        ros::Duration(0.5).sleep();
      }
    }

    rate.sleep();
  }

  return 0;
}