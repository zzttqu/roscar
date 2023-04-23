#! /bin/bash
catkin_make
source ~/.bashrc 
roslaunch multi_agv_sim model.launch &
sleep 3
echo "中心计算节点以及导航到组合点已启动完毕"
sleep 20

roslaunch multi_agv_sim center_launch.launch &
sleep 1
echo "娇贵的中心导航节点已启动(没有滤波地图和车的tf老子飞给你看)"

wait
exit 0