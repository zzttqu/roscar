typedef union _uart_Float_
{
    short i_data;
    uint8_t byte[2];
} uart_INT;
typedef struct _AGV_Vel_
{
    float X;
    float Y;
    double Yaw;
} AGV_Vel;

// 这个是在世界坐标系下的坐标，不是车自身坐标系下的
typedef struct _AGV_Pos_
{
    float X;
    float Y;
    float Yaw;
} AGV_Pos;

typedef struct _Motor_Parameter_
{
  float target;
  uart_INT preloader;
  int direction_Target;
  uart_INT encoder;
  int direction_Now;
} Motor_Parameter;
