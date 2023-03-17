typedef union _uart_Float_
{
    int i_data;
    uint8_t byte[2];
} uart_INT;
typedef struct _Data_Reciever_
{
    unsigned char buffer[32];
    uart_INT X_speed;
    uart_INT Y_speed;
    uart_INT Z_speed;
} Data_Reciever;

typedef struct _Data_Sender_
{
    unsigned char buffer[32];
    unsigned char Data_Header;
    uart_INT X_speed;
    uart_INT Y_speed;
    uart_INT Z_speed;
    unsigned char Data_Tail;
} Data_Sender;

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
  uint8_t direction_Target;
  uart_INT encoder;
  uint8_t direction_Now;
} Motor_Parameter;
