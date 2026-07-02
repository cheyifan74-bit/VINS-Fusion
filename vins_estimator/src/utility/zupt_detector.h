#pragma once

#include <deque>
#include <eigen3/Eigen/Dense>
#include "utility.h"

// Sliding window variance (O(1) add, O(1) remove oldest)
struct SlidingWindowVar
{
    std::deque<double> buf;
    double sum = 0, sum_sq = 0;
    int max_size;

    SlidingWindowVar(int n = 200) : max_size(n) {}

    void add(double x)
    {
        buf.push_back(x);
        sum += x;
        sum_sq += x * x;
        while ((int)buf.size() > max_size)
        {
            double old = buf.front();
            buf.pop_front();
            sum -= old;
            sum_sq -= old * old;
        }
    }

    double var() const
    {
        int n = buf.size();
        if (n < 2)
            return 0;
        double mean = sum / n;
        return sum_sq / n - mean * mean;
    }

    int size() const { return (int)buf.size(); }

    void reset()
    {
        buf.clear();
        sum = 0;
        sum_sq = 0;
    }
};

// ZUPT (Zero Velocity Update) IMU classifier
// Pure IMU check: instantaneous + sliding window variance.
// Returns per-frame bool — no internal state machine.
class ZuptDetector
{
public:
    struct Params
    {
        double acc_gravity_tol;
        double gyr_norm_max;
        double acc_var_max;
        double gyr_var_max;
        int window_size;

        Params()
            : acc_gravity_tol(1.5), gyr_norm_max(0.2), acc_var_max(0.1), gyr_var_max(0.001), window_size(200)
        {
        }
    };

    ZuptDetector(const Params &p = Params()) : params_(p), acc_var_(p.window_size), gyr_var_(p.window_size) {}

    // Called for each IMU sample -> checks instantaneous conditions
    void addImuSample(const Eigen::Vector3d &acc, const Eigen::Vector3d &gyr)
    {
        double acc_err = fabs(acc.norm() - 9.81);
        double gyr_norm = gyr.norm();

        // Per-sample instantaneous checks
        if (acc_err > params_.acc_gravity_tol || gyr_norm > params_.gyr_norm_max)
            frame_instant_fail_ = true;

        // Sliding window variance
        acc_var_.add(acc.norm());
        gyr_var_.add(gyr.norm());
    }

    // Called once per camera frame: returns true if IMU looks stationary this frame
    bool update()
    {
        // Insufficient data → not ready
        if (acc_var_.size() < 5 || gyr_var_.size() < 5)
            return false;

        bool var_ok = (acc_var_.var() < params_.acc_var_max) &&
                      (gyr_var_.var() < params_.gyr_var_max);
        bool instant_ok = !frame_instant_fail_;

        frame_instant_fail_ = false; // reset for next frame

        return var_ok && instant_ok;
    }

    void configure(double acc_var, double gyr_var)
    {
        params_.acc_var_max = acc_var;
        params_.gyr_var_max = gyr_var;
    }

    // Debug info
    double accVar() const { return acc_var_.var(); }
    double gyrVar() const { return gyr_var_.var(); }
    int accSamples() const { return acc_var_.size(); }
    int gyrSamples() const { return gyr_var_.size(); }

    void reset()
    {
        frame_instant_fail_ = false;
        acc_var_.reset();
        gyr_var_.reset();
    }

private:
    Params params_;
    SlidingWindowVar acc_var_, gyr_var_;
    bool frame_instant_fail_ = false;
};
