#pragma once

#include <cmath>

namespace autonomous_driver
{

// Very simple 1D Kalman Filter:
// State x = scalar (e.g., center error)
// Model: x_k+1 = x_k + w,   w ~ N(0, Q)
// Meas:  z_k   = x_k + v,   v ~ N(0, R)
class Kalman1D
{
public:
    Kalman1D(double process_noise, double measurement_noise)
    : Q_(process_noise),
      R_(measurement_noise),
      x_(0.0),
      P_(1.0),
      initialized_(false)
    {}

    // Set initial state
    void reset(double x0)
    {
        x_ = x0;
        P_ = 1.0;
        initialized_ = true;
    }

    // Predict step: x_k+1 = x_k, P_k+1 = P_k + Q
    void predict()
    {
        if (!initialized_) return;
        P_ = P_ + Q_;
    }

    // Update step with a new measurement z
    void update(double z)
    {
        if (!initialized_) {
            reset(z);
            return;
        }

        // Kalman gain
        double K = P_ / (P_ + R_);
        // Update estimate
        x_ = x_ + K * (z - x_);
        // Update covariance
        P_ = (1.0 - K) * P_;
    }

    double value() const { return x_; }

private:
    double Q_;  // process noise variance
    double R_;  // measurement noise variance
    double x_;  // state estimate
    double P_;  // state covariance
    bool   initialized_;
};

} // namespace autonomous_driver
