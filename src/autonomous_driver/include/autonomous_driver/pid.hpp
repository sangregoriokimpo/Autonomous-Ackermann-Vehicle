#ifndef PID_HPP
#define PID_HPP

#include <rclcpp/rclcpp.hpp>
#include <algorithm>

class AdaptivePIDController
{
public:
    AdaptivePIDController(double base_kp = 0.6, double base_ki = 0.08, double base_kd = 0.25)
        : base_kp_(base_kp), base_ki_(base_ki), base_kd_(base_kd)
    {
        reset();
    }

    void reset()
    {
        error_sum_ = 0.0;
        last_error_ = 0.0;
        initialized_ = false;
    }

    void setBaseGains(double kp, double ki, double kd)
    {
        base_kp_ = kp;
        base_ki_ = ki;
        base_kd_ = kd;
    }

    void setIntegralLimits(double min_limit, double max_limit)
    {
        min_integral_ = min_limit;
        max_integral_ = max_limit;
    }

    void setDistanceThresholds(double very_close, double close, double moderate)
    {
        threshold_very_close_ = very_close;
        threshold_close_ = close;
        threshold_moderate_ = moderate;
    }

    void setDistanceFactors(double very_close_factor, double close_factor, double moderate_factor)
    {
        factor_very_close_ = very_close_factor;
        factor_close_ = close_factor;
        factor_moderate_ = moderate_factor;
    }

    double compute(double error, double distance_metric, rclcpp::Time current_time)
    {
        if (!initialized_)
        {
            last_error_ = error;
            last_time_ = current_time;
            initialized_ = true;
            return 0.0;
        }

        double dt = (current_time - last_time_).seconds();
        if (dt <= 0.0 || dt > 0.5)
        {
            dt = 0.1;
        }

        double distance_factor = computeDistanceFactor(distance_metric);

        double kp = base_kp_ * distance_factor;
        double ki = base_ki_ * distance_factor;
        double kd = base_kd_ * distance_factor;

        double p_term = kp * error;

        error_sum_ += error * dt;
        error_sum_ = std::clamp(error_sum_, min_integral_, max_integral_);
        double i_term = ki * error_sum_;

        double error_rate = (error - last_error_) / dt;
        double d_term = kd * error_rate;

        last_error_ = error;
        last_time_ = current_time;

        return p_term + i_term + d_term;
    }

    double getProportionalTerm(double error, double distance_metric) const
    {
        double distance_factor = computeDistanceFactor(distance_metric);
        return base_kp_ * distance_factor * error;
    }

    double getIntegralTerm(double distance_metric) const
    {
        double distance_factor = computeDistanceFactor(distance_metric);
        return base_ki_ * distance_factor * error_sum_;
    }

    double getDerivativeTerm(double distance_metric) const
    {
        if (!initialized_) return 0.0;
        double distance_factor = computeDistanceFactor(distance_metric);
        double dt = 0.1;
        double error_rate = (last_error_ - error_sum_) / dt;
        return base_kd_ * distance_factor * error_rate;
    }

    double getErrorSum() const { return error_sum_; }
    double getLastError() const { return last_error_; }
    bool isInitialized() const { return initialized_; }

private:
    double computeDistanceFactor(double distance_metric) const
    {
        if (distance_metric < threshold_very_close_)
        {
            return factor_very_close_;
        }
        else if (distance_metric < threshold_close_)
        {
            return factor_close_;
        }
        else if (distance_metric < threshold_moderate_)
        {
            return factor_moderate_;
        }
        else
        {
            return 1.0;
        }
    }

    double base_kp_;
    double base_ki_;
    double base_kd_;

    double error_sum_;
    double last_error_;
    rclcpp::Time last_time_;
    bool initialized_;

    double min_integral_ = -2.0;
    double max_integral_ = 2.0;

    double threshold_very_close_ = 1.5;
    double threshold_close_ = 2.5;
    double threshold_moderate_ = 3.5;

    double factor_very_close_ = 2.2;
    double factor_close_ = 1.6;
    double factor_moderate_ = 1.2;
};

#endif