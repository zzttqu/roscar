#include <ros/ros.h>

int main(int argc, char *argv[])
{
    /* code */
    ros::init(argc, argv, "set_param");
    ros::NodeHandle n;
    n.setParam("type", "xiao_Huang");
    n.setParam("radius", 0.5);

    ros::param::set("num", 1);
    // 对同一数值的set表示对数值的覆盖修改
    ros::param::set("num", 5);
    return 0;
}
