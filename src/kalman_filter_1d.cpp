#include "ros_math_algorithms/kalman_filter_1d.h"

KalmanFilter1D::KalmanFilter1D(double x, double A, double B, double H, double Q, double R, double P)
    : x_(x), A_(A), B_(B), H_(H), Q_(Q), R_(R), P_(P)
{
}

void KalmanFilter1D::initialize(double x, double A, double B, double H, double Q, double R, double P)
{
    x_ = x;
    A_ = A;
    B_ = B;
    H_ = H;
    Q_ = Q;
    R_ = R;
    P_ = P;
}

void KalmanFilter1D::predict()
{
    predict(0);
}

void KalmanFilter1D::predict(double u)
{
    x_ = A_ * x_ + B_ * u;
    P_ = A_ * P_ * A_ + Q_;
}

void KalmanFilter1D::update(double z)
{
    double K = P_ * H_ / (H_ * P_ * H_ + R_);
    x_ = x_ + K * (z - H_ * x_);
    P_ = (1 - K * H_) * P_;
}

void KalmanFilter1D::state(double* x, double* p) const
{
    *x = x_;
    *p = P_;
}
