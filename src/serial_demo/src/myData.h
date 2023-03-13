typedef union _uart_Float_
{
    float f_data;
    uint8_t byte[4];
} uart_Float;
typedef struct _Data_Reciever_
{
    unsigned char buffer[32];
    unsigned char Data_Header;
    uart_Float X_speed;
    uart_Float Y_speed;
    uart_Float Z_speed;
    unsigned char Data_Tail;
} Data_Reciever;

typedef struct _Data_Sender_
{
    unsigned char buffer[32];
    unsigned char Data_Header;
    uart_Float X_speed;
    uart_Float Y_speed;
    uart_Float Z_speed;
    unsigned char Data_Tail;
} Data_Sender;

typedef struct _AGV_Vel_
{
    float X;
    float Y;
    float Z;
} AGV_Vel;

// 这个是在世界坐标系下的坐标，不是车自身坐标系下的
typedef struct _AGV_Pos_
{
    float X;
    float Y;
    float Yaw;
} AGV_Pos;