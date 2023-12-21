#pragma once

#include "memory"

class KalmanFilter1D {

public:
    // Constructer.
    KalmanFilter1D(double x, double A, double B, double H, double Q, double R, double P);
    // Initialize the variables.
    void initialize(double x, double A, double B, double H, double Q, double R, double P);

    void predict();

    void predict(double u);

    void update(double z);

    void state(double* x, double* p) const;

    typedef std::shared_ptr<KalmanFilter1D> Ptr;

private:
    // State transition matrix.
    double A_;
    // Control input matrix.
    double B_;
    // Measurement matrix
    double H_;
    // Process noise covariance.
    double Q_;
    // Measurement noise covariance.
    double R_;
    // Estimated error covariance.
    double P_;
    // System state.
    double x_;
};
