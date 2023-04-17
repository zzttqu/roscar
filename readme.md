<!--
 * @Author: zzttqu zzttqu@gmail.com
 * @Date: 2023-03-28 11:55:46
 * @LastEditors: zzttqu zzttqu@gmail.com
 * @LastEditTime: 2023-04-04 13:22:25
 * @FilePath: /roscar/readme.md
 * @Description:
 * 一个大学生的毕业设计
-->
# 注意事项
1. solidworks生成URDF的时候需要保证全英文，如果不能，则要在ubuntu里边安装unicode包然后重启，保证支持中文，否则会出现tf无法正常获取的情况


  # 不用rot_vel因为查看了参数服务器不对
  max_vel_theta: 3
  min_rot_vel: -3
查看参数服务器的方法rosrun rqt_reconfigure rqt_reconfigure
crtographer按照官方的方法安装，缺依赖项需要删除。
通过修改bashrc完成多个工作空间的source

多AGV时创建多个AGV使用了xacro中的命名空间，没用使用tf前缀，要注意xacro中所有话题，link，frame都要加上前缀，包括odom，gmapping建图节点也要加入命名空间。