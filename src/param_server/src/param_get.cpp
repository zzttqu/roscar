#include <ros/ros.h>

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    /* code */
    ros::init(argc,argv,"get_param");
    ros::NodeHandle n;

    
    double radius;
    if(ros::param::get("radius",radius)){
        ROS_INFO_STREAM("radius为:"<<radius);
    }
    //遍历元素
    std::vector<std::string> names;
    ros::param::getParamNames(names);
    for (auto &&name : names)
    {
        ROS_INFO_STREAM("参数名为:"<<name.c_str());
    }
    

    return 0;
}
