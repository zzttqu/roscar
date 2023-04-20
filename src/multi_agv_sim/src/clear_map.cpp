#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PolygonStamped.h>

ros::Publisher pub_map;  // 定义地图发布者
geometry_msgs::PolygonStamped polygon;  // 定义要清除的区域

// 回调函数，订阅地图，并清除指定区域的障碍物
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  nav_msgs::OccupancyGrid map = *msg;  // 复制地图

  // 将要清除的区域填充为0（即可通行）
  for (int i = 0; i < map.info.width; i++) {
    for (int j = 0; j < map.info.height; j++) {
      if (i >= polygon.polygon.points[0].x && i <= polygon.polygon.points[2].x
          && j >= polygon.polygon.points[0].y && j <= polygon.polygon.points[2].y) {
        int index = i + j * map.info.width;
        map.data[index] = 0;
      }
    }
  }

  // 发布清除后的地图
  pub_map.publish(map);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "clear_map_region");
  ros::NodeHandle nh;

  // 定义要清除的区域
  polygon.header.frame_id = "map";
  geometry_msgs::Point32 point;
  point.x = -1.0;
  point.y = -1.0;
  polygon.polygon.points.push_back(point);
  point.x = -1.0;
  point.y = 1.0;
  polygon.polygon.points.push_back(point);
  point.x = 1.0;
  point.y = 1.0;
  polygon.polygon.points.push_back(point);
  point.x = 1.0;
  point.y = -1.0;
  polygon.polygon.points.push_back(point);

  // 订阅地图话题
  ros::Subscriber sub_map = nh.subscribe("/map", 1, mapCallback);

  // 发布地图话题
  pub_map = nh.advertise<nav_msgs::OccupancyGrid>("/map_clear", 1, true);

  ros::spin();

  return 0;
}