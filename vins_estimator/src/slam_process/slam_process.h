#pragma once

#include <thread>
#include "../featureTracker/feature_tracker.h"
#include "../estimator/estimator.h"

class SlamProcess
{
public:
    SlamProcess() = default;
    ~SlamProcess();

    // ---- lifecycle ----
    void init();
    void start();

    // ---- public interfaces ----
    void inputIMU(double t, const Eigen::Vector3d &acc, const Eigen::Vector3d &gyr);
    void inputImage(double t, const cv::Mat &img0, const cv::Mat &img1 = cv::Mat());
    void inputFeature(double t, const std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>> &featureFrame);
    void restart();
    void changeSensorType(int use_imu, int use_stereo);

private:
    Estimator estimator_;
    FeatureTracker tracker_;
    std::thread track_thread_;
    std::thread process_thread_;
};
