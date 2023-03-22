#include "KalmanFilter.hpp"

KalmanFilter::KalmanFilter()
{
    A_ = 1;
    Q_ = 0.1;
    H_ = 1;
    R_ = 1;
    x_ = 0;
    P_ = 1;
    z_ = 0;
}
KalmanFilter::KalmanFilter(double A, double Q, double H, double R, double x, double P, double z)
{

    A_ = A;
    Q_ = Q;
    H_ = H;
    R_ = R;
    x_ = x;
    P_ = P;
    z_ = z;
}

double KalmanFilter::filter(double z)
{
    // 预测状态向量
    x_ = A_ * x_;
    // 预测状态向量的协方差矩阵
    P_ = A_ * P_ * A_ + Q_;
    // 计算卡尔曼增益
    double K = P_ * H_ / (H_ * P_ * H_ + R_);
    // 更新状态向量
    x_ = x_ + K * (z - H_ * x_);
    // 更新状态向量的协方差矩阵
    P_ = (1 - K * H_) * P_;
    return x_;
}

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