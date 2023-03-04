typedef struct _Data_Transer_
{
    unsigned char buffer[64];
    struct _Data_
    {
        unsigned char Data_Header;
        float X_speed;
        float Y_speed;
        float Z_speed;
        unsigned char Data_Tail;
    } data;
} Data_Transer;

typedef struct _AGV_Vel_
{
    float X;
    float Y;
    float Z;
}AGV_Vel;

typedef struct _AGV_Pos_
{
    float X;
    float Y;
    float Z;
}AGV_Pos;