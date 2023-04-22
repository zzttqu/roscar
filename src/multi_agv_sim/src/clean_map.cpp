#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PolygonStamped.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
// 定义地图发布者
geometry_msgs::PolygonStamped polygon; // 定义要清除的区域
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
          map.data[index] = 100;
        }
      }
    }
  }
  return map;
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &map, ros::Publisher &map_pub)
{
  // 当地图更新后
  nav_msgs::OccupancyGrid filtered_map = *map;

  nav_msgs::OccupancyGrid filtered_map_pub = setCostToZero(filtered_map, 1.0, 1.0, 1.0);
  ROS_INFO("发布了地图");
  map_pub.publish(filtered_map_pub);
}

int main(int argc, char **argv)
{
  setlocale(LC_ALL, "");
  ros::init(argc, argv, "clear_map_region");
  // 地图清理节点开始运行
  ros::NodeHandle n;

  // 订阅地图话题
  ros::Publisher map_pub = n.advertise<nav_msgs::OccupancyGrid>("/filtered_map", 1);
  ros::Subscriber sub_map = n.subscribe<nav_msgs::OccupancyGrid>("/agv_0/map", 1, boost::bind(mapCallback, _1, boost::ref(map_pub)));

  // 发布地图话题
  // pub_map = nh.advertise<nav_msgs::OccupancyGrid>("/map_clear", 1, true);

  ros::spin();

  return 0;
}