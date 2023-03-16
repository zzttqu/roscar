# 注意事项
1. solidworks生成URDF的时候需要保证全英文，如果不能，则要在ubuntu里边安装unicode包然后重启，保证支持中文，否则会出现tf无法正常获取的情况


  # 不用rot_vel因为查看了参数服务器不对
  max_vel_theta: 3
  min_rot_vel: -3
查看参数服务器的方法rosrun rqt_reconfigure rqt_reconfigure