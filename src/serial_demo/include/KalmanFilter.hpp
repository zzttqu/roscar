#ifndef KalmanFilter_H
#define KalmanFilter_H
class KalmanFilter
{
public:
    KalmanFilter(double A, double Q, double H, double R, double x, double P, double z);
    KalmanFilter();
    double filter(double z);

private:
    double A_; // 系统状态转移矩阵
    double Q_; // 系统状态转移矩阵的协方差矩阵
    double H_; // 观测矩阵
    double R_; // 观测矩阵的协方差矩阵
    double x_; // 状态向量
    double P_; // 状态向量的协方差矩阵
    double z_; // 观测向量
};
#endif
// int main()
// {
//     // 初始化卡尔曼滤波器
//     KalmanFilter kf(1, 0.1, 1, 1, 0, 1, 0);

//     // 一组观测向量
//     double z[] = {1, 2, 3, 4, 5};

//     // 对每个观测向量进行预测和更新
//     for (int i = 0; i < 5; i++)
//     {
//         double x = kf.filter(z[i]);
//     }

//     return 0;
// }