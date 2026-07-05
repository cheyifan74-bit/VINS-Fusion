/*
 * @Description:
 * @Author: che yifan
 * @Date: 2026-07-05 13:32:31
 * @LastEditTime: 2026-07-05 13:34:44
 * @LastEditors: che yifan
 * @Reference:
 */
#pragma once

#include <thread>
#include "../featureTracker/feature_tracker.h"
#include "../estimator/estimator.h"

class SlamProcess
{
public:
    SlamProcess() = default;
    ~SlamProcess();

    void init();
    void start();
    void registerSubscribers(ros::NodeHandle &n);

    void inputIMU(double t, const Eigen::Vector3d &acc, const Eigen::Vector3d &gyr);
    void inputImage(double t, const cv::Mat &img0, const cv::Mat &img1 = cv::Mat());

private:
    Estimator estimator_;
    FeatureTracker tracker_;
    std::thread track_thread_;
    std::thread process_thread_;
};
