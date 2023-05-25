#include "serial_node.hpp"
using namespace std;

// sudo chmod 666 /dev/ttyUSB0
//  发送和接受串口消息
Motor_Parameter MOTOR_Parameters[4];
unsigned char STM32_Settings[16] = {'A'};
AGV_Vel agv_nav_vel;
// 车辆参数
int wheel_center_x = 250;
int wheel_center_y = 250;
serial::Serial se;

// uint8_t=unsigned char等价关系
// # TODO 完成设置代码

static void Speed_Trans(AGV_Vel agv_vel)
{
    // 运动学解算出四个轮子的线速度
    MOTOR_Parameters[0].target_speed = (float)agv_vel.X - agv_vel.Y - agv_vel.Yaw * (wheel_center_x + wheel_center_y);
    MOTOR_Parameters[1].target_speed = (float)agv_vel.X + agv_vel.Y + agv_vel.Yaw * (wheel_center_x + wheel_center_y);
    MOTOR_Parameters[2].target_speed = (float)agv_vel.X - agv_vel.Y + agv_vel.Yaw * (wheel_center_x + wheel_center_y);
    MOTOR_Parameters[3].target_speed = (float)agv_vel.X + agv_vel.Y - agv_vel.Yaw * (wheel_center_x + wheel_center_y);

    for (uint8_t i = 0; i < 4; i++)
    {
        // 要先变为角速度值，再转化为preloader数值,-1要在abs外边
        MOTOR_Parameters[i].preloader = (short)abs(PI * wheel_r_mm * 1000 / MOTOR_Parameters[i].target_speed) - 1;
        if (MOTOR_Parameters[i].preloader == -1)
        {
            MOTOR_Parameters[i].preloader = 15000;
        }
        // 转向判断
        MOTOR_Parameters[i].direction_Target = (MOTOR_Parameters[i].target_speed > 0) ? 1 : -1;
    }
    ROS_INFO_STREAM("X:" << agv_vel.X << "  Y:" << agv_vel.Y << "  Yaw:" << agv_vel.Yaw);
    std::ostringstream ss;
    for (int i = 0; i < 4; i++)
    {
        ss << "\n"
           << static_cast<char>('1' + i) << "电机preloader:" << MOTOR_Parameters[i].preloader
           << "方向为:" << MOTOR_Parameters[i].direction_Target;
    }
    ROS_INFO_STREAM(ss.str());
};

void STM32_Serial::Read_Data(uint8_t resBuff[], int buff_size)
{
    uint8_t ccbuff[64];
    if (se.available() > 0)
    {
        try
        {
            se.read(ccbuff, se.available());
            for (size_t i = 0; i < 32; i++)
            {
                if (ccbuff[i] == HEADER)
                {
                    uint8_t CRC = 0x00;
                    for (size_t j = 0; j < 32; j++)
                    {
                        resBuff[j] = ccbuff[i + j];
                    }
                    for (size_t i = 0; i < 30; i++)
                    {
                        CRC = resBuff[i] ^ CRC;
                    }
                    // std::ostringstream ss;
                    // for (int i = 0; i < 32; i++)
                    // {
                    //     ss << " " << hex
                    //        << (short)resBuff[i];
                    // }
                    // ROS_INFO_STREAM("接受数据校验未通过" << ss.str());
                    if (CRC == resBuff[30])
                    {
                        return;
                    }
                    else
                    {
                        std::ostringstream ss;
                        for (int i = 0; i < 30; i++)
                        {
                            ss << " " << hex
                               << (short)resBuff[i];
                        }
                        ROS_WARN_STREAM("接受数据校验未通过" << ss.str());
                        memset(resBuff, 0x00, buff_size);
                    }
                }
                else
                {
                    memset(resBuff, 0x00, buff_size);
                }
            }
            // ROS_INFO_STREAM(ccbuff);
        }
        catch (serial::IOException &e)
        {
            ROS_ERROR_STREAM("数据接收失败 " << e.what());
        }
    }
    memset(ccbuff, 0x00, 64);
}
// 数据转换和正向运动学解算
AGV_Vel STM32_Serial::Recieve_Speed_Trans(uint8_t Recieve_Buffer[])
{
    AGV_Vel agv_vel;
    for (size_t i = 0; i < 4; i++)
    {
        size_t base_idx = i * 6 + 2;
        short encoder;
        short voltage;
        short current;
        memcpy(&encoder, Recieve_Buffer + base_idx, sizeof(encoder));
        memcpy(&voltage, Recieve_Buffer + base_idx + 2, sizeof(encoder));
        memcpy(&current, Recieve_Buffer + base_idx + 4, sizeof(encoder));
        MOTOR_Parameters[i].current = current;
        MOTOR_Parameters[i].encoder = encoder;
        MOTOR_Parameters[i].voltage = voltage;
        // ROS_INFO("%hx %hx %hx %hx", Recieve_Buffer[base_idx], Recieve_Buffer[base_idx + 1], Recieve_Buffer[base_idx + 2], Recieve_Buffer[base_idx + 3]);
    }
    // 对编码器数据进行卡尔曼滤波
    // for (size_t i = 0; i < 4; i++)
    // {
    //     MOTOR_Parameters[i].encoder = static_cast<short>(MOTOR_Parameters->kf.filter(MOTOR_Parameters[i].encoder));
    // }
    MOTOR_Parameters[3].encoder = (short)MOTOR_Parameters[3].direction_Target*(abs( MOTOR_Parameters[3].target_speed/2/PI/wheel_r_mm*0.1*1000*2));
    agv_vel.X = 2 * PI * wheel_r_mm * (MOTOR_Parameters[0].encoder + MOTOR_Parameters[1].encoder + MOTOR_Parameters[2].encoder + MOTOR_Parameters[3].encoder) / 4.0 / dt / encoder_num / 2.0;
    agv_vel.Y = 2 * PI * wheel_r_mm * (-MOTOR_Parameters[0].encoder - MOTOR_Parameters[1].encoder + MOTOR_Parameters[2].encoder + MOTOR_Parameters[3].encoder) / 4.0 / dt / encoder_num / 2.0;
    agv_vel.Yaw = 2 * PI * wheel_r_mm * ((-MOTOR_Parameters[0].encoder + MOTOR_Parameters[1].encoder + MOTOR_Parameters[2].encoder - MOTOR_Parameters[3].encoder) / 4.0 / dt / 2.0 / (wheel_center_x + wheel_center_y) / encoder_num);
    // 转换为m
    agv_vel.X = int(agv_vel.X / 1000.0 * 100) / 100.0;
    agv_vel.Y = int(agv_vel.Y / 1000.0 * 100) / 100.0;
    agv_vel.Yaw = int(agv_vel.Yaw * 100) / 100.0;
    std::ostringstream ss;
    for (int i = 0; i < 4; i++)
    {
        ss << " "
           << MOTOR_Parameters[i].encoder;
    }
    ROS_INFO_STREAM(ss.str());
    ROS_INFO_STREAM("X速度为" << agv_vel.X << " Y速度为" << agv_vel.Y << " Z转角为" << agv_vel.Yaw);
    // ROS_INFO_STREAM("1号电机电压为" << MOTOR_Parameters[0].voltage << " 2号电机电压为" << MOTOR_Parameters[1].voltage << " 3号电机电压为" << MOTOR_Parameters[2].voltage << " 4号电机电压为" << MOTOR_Parameters[3].voltage);
    return agv_vel;
}
// STM32设置
void STM32_Serial::STM32_Set()
{
    // 开启速度回传
    STM32_Settings[1] = 'I';
    // 电机启动
    STM32_Settings[2] = 'I';
    se.write(STM32_Settings, sizeof(STM32_Settings));
    ROS_INFO_STREAM("设置成功" << STM32_Settings);
}
void STM32_Serial::STM32_Stop()
{
    // 关闭速度回传
    STM32_Settings[1] = 'S';
    // 电机停止
    STM32_Settings[2] = 'S';
    se.write(STM32_Settings, sizeof(STM32_Settings));
    ROS_INFO_STREAM("设置成功" << STM32_Settings);
}

void STM32_Serial::Send_Speed_Trans()
{
    memset(Send_Buffer, 0x00, sizeof(Send_Buffer));
    // 导航的速度数据切分后传入
    // 赋值到buffer中进行传输，四bit为一个float
    Speed_Trans(agv_nav_vel);
    Send_Buffer[0] = HEADER;
    // for (size_t i = 2; i < 14; i++)
    // {
    //     if (i < 4)
    //     {
    //         Send_Buffer[i] = MOTOR_Parameters[0].preloader.byte[(i % 2) ? 0 : 1];
    //     }
    //     else if (i < 6)
    //     {
    //         Send_Buffer[i] = MOTOR_Parameters[1].preloader.byte[(i % 2) ? 0 : 1];
    //     }
    //     else if (i < 8)
    //     {
    //         Send_Buffer[i] = MOTOR_Parameters[2].preloader.byte[(i % 2) ? 0 : 1];
    //     }
    //     else if (i < 10)
    //     {
    //         Send_Buffer[i] = MOTOR_Parameters[3].preloader.byte[(i % 2) ? 0 : 1];
    //     }
    //     else if (i < 14)
    //     {
    //         // Send_Buffer[i] = MOTOR_Parameters[(i - 2) % 4].direction_Target;
    //     }
    //}
    for (size_t i = 0; i < 4; i++)
    {
        size_t base_idx = i * 4 + 2;
        short preloader = MOTOR_Parameters[i].preloader;
        short direction = MOTOR_Parameters[i].direction_Target;
        memcpy(Send_Buffer + base_idx, &preloader, sizeof(preloader));
        memcpy(Send_Buffer + base_idx + 2, &direction, sizeof(direction));
    }

    uint8_t CRC = 0x00;
    for (size_t i = 0; i < 30; i++)
    {
        CRC = Send_Buffer[i] ^ CRC;
    }
    Send_Buffer[30] = CRC;
    Send_Buffer[31] = TAIL;

    // std::ostringstream ss;
    // for (int i = 0; i < 32; i++)
    // {
    //     ss << " " << hex
    //        << (short)Send_Buffer[i];
    // }
    // ROS_INFO_STREAM(ss.str());
}
// 串口初始化
int STM32_Serial::Serial_Init(string port, int baudrate)
{
    try
    {
        se.setPort(port);
        se.setBaudrate(baudrate);
        se.setTimeout(to);
        se.open();
        STM32_Set();
        ROS_INFO("打开串口成功");
        return 1;
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("打开串口失败" << e.what());
        return -1;
    }
}
int STM32_Serial::Serial_Close()
{
    try
    {
        STM32_Stop();
        se.close();
        ROS_INFO("关闭串口成功");
        return 1;
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR("关闭串口失败");
        return -1;
    }
}

// 发送速度信息
int STM32_Serial::Send_Speed_Msg()
{
    try
    {
        if (se.isOpen())
        {
            se.write(Send_Buffer, sizeof(Send_Buffer));
        }
        else
        {
            ROS_ERROR("串口未打开，无法发送信息");
        }
        return 1;
    }
    catch (const serial::IOException e)
    {
        ROS_ERROR("串口错误，无法发送信息");
        return -1;
    }
}

// 获取串口数据并转化
bool STM32_Serial::Get_Data()
{
    Read_Data(Recieve_Buffer, sizeof(Recieve_Buffer));             // 通过串口读取下位机发送过来的数据
    if (Recieve_Buffer[0] == HEADER && Recieve_Buffer[31] == TAIL) // 验证数据包的帧尾
    {
        //  ROS_INFO_STREAM("编码器为"<<MOTOR_Parameters[0].encoder<<" 电压为"<<MOTOR_Parameters[0].voltage<<" 电流为"<<MOTOR_Parameters[0].current);
        agv_encoder_vel = Recieve_Speed_Trans(Recieve_Buffer);
        // ROS_INFO("agv速度为x=%.3f y=%.3f z=%.3f", agv_encoder_vel.X, agv_encoder_vel.Y, agv_encoder_vel.Yaw);
        se.flush();
        return true;
    }
    return false;
}
// 发布odom数据
void STM32_Serial::Publish_Odom()
{
    // 定义tf 对象
    static tf2_ros::TransformBroadcaster odom_broadcaster;
    // 定义tf发布时需要的类型消息
    geometry_msgs::TransformStamped odom_trans;
    pub_odom = n.advertise<nav_msgs::Odometry>("/odom", 1000);
    nav_msgs::Odometry odom; // 实例化里程计话题数据

    ros::Time current_time = ros::Time::now();

    // 循环获取STM32速度,间隔是0.1s

    // 这里是AGV本身坐标系下的
    dx = (agv_encoder_vel.X * cos(agv_pos.Yaw) - agv_encoder_vel.Y * sin(agv_pos.Yaw)) * dt; // dt是宏定义的，是32读取编码器的时间
    dy = (agv_encoder_vel.X * sin(agv_pos.Yaw) + agv_encoder_vel.Y * cos(agv_pos.Yaw)) * dt;
    dz = agv_encoder_vel.Yaw * dt;
    // 转化到世界坐标系下
    agv_pos.Yaw += dz;
    // 根据角度简化防止超过2pai
    // agv_pos.Yaw = (agv_pos.Yaw > PI) ? (agv_pos.Yaw - 2 * PI) : ((agv_pos.Yaw < -PI) ? (agv_pos.Yaw + 2 * PI) : agv_pos.Yaw);
    // 世界坐标系下
    // agv_pos.X += dx * cos(agv_pos.Yaw) + dy * sin(agv_pos.Yaw);
    // agv_pos.Y += dx * sin(agv_pos.Yaw) + dy * cos(agv_pos.Yaw);
    agv_pos.X += dx;
    agv_pos.Y += dy;
    // 把Z轴转角转换为四元数进行表达
    tf2::Quaternion qtn;
    qtn.setRPY(0, 0, agv_pos.Yaw);

    // 发布坐标变换父子坐标系
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    // 填充获取的数据
    odom_trans.transform.translation.x = agv_pos.X; // x坐标
    odom_trans.transform.translation.y = agv_pos.Y; // y坐标
    odom_trans.transform.translation.z = 0;         // z坐标
    odom_trans.transform.rotation.x = qtn.getX();   // 旋转角
    odom_trans.transform.rotation.y = qtn.getY();
    odom_trans.transform.rotation.z = qtn.getZ();
    odom_trans.transform.rotation.w = qtn.getW();
    // 发布tf坐标变换
    odom_broadcaster.sendTransform(odom_trans);

    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";         // 里程计TF父坐标
    odom.pose.pose.position.x = agv_pos.X; // 位置
    odom.pose.pose.position.y = agv_pos.Y;
    odom.pose.pose.position.z = 0;
    odom.pose.pose.orientation.x = qtn.getX(); // 姿态，通过Z轴转角转换的四元数
    odom.pose.pose.orientation.y = qtn.getY();
    odom.pose.pose.orientation.z = qtn.getZ();
    odom.pose.pose.orientation.w = qtn.getW();

    odom.child_frame_id = "base_link";                // 里程计TF子坐标
    odom.twist.twist.linear.x = agv_encoder_vel.X;    // X方向速度
    odom.twist.twist.linear.y = agv_encoder_vel.Y;    // Y方向速度
    odom.twist.twist.angular.z = agv_encoder_vel.Yaw; // 绕Z轴角速度
    pub_odom.publish(odom);                           // Pub odometer topic //发布里程计话题
    // ros::spinOnce();
}
// 接收导航传入的速度数据
void STM32_Serial::Subsribe_cmd_vel()
{
    velocityCMD = n.subscribe<geometry_msgs::Twist>("/cmd_vel", 1000, boost::bind(&STM32_Serial::V_CallBack, this, _1));
}

void STM32_Serial::V_CallBack(const geometry_msgs::Twist::ConstPtr &msg)
{
    // 换算为mm/s
    float X = msg.get()->linear.x * 1000;
    float Y = msg.get()->linear.y * 1000;
    double Yaw = msg.get()->angular.z; // 这个是rad/s
    agv_nav_vel = {X, Y, Yaw};
    Send_Speed_Trans();
    Send_Speed_Msg();
}
STM32_Serial::STM32_Serial(ros::NodeHandle &node)
{
    n = node;
}
void MySigintHandler(int sig)
{
    // 这里主要进行退出前的数据保存、内存清理、告知其他节点等工作
    se.write("ASS");
    se.close();
    ROS_INFO("串口节点关闭咯!");
    ros::shutdown();
    exit(0);
}
// int main(int argc, char *argv[])
// {
//     int count = 0;
//     setlocale(LC_ALL, "");
//     setlocale(LC_CTYPE, "zh_CN.utf8");
//     ros::init(argc, argv, "serial_port");
//     ros::NodeHandle n;
//     ros::Rate rate(10);
//     STM32_Serial stm32_Serial(n);
//     stm32_Serial.Subsribe_cmd_vel();
//     signal(SIGINT, MySigintHandler);
//     ROS_INFO("serial node is running");
//     if (stm32_Serial.Serial_Init(port) == -1)
//     {
//         return -1;
//     }
//     while (ros::ok())
//     {
//         if (stm32_Serial.Get_Data())
//         {
//             stm32_Serial.Publish_Odom();
//         }
//         ros::spinOnce();
//         rate.sleep();
//         count++;
//     }
//     stm32_Serial.Serial_Close(port);
//     return 0;
// }
