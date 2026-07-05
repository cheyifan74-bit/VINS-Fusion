/*
 * @Description:
 * @Author: che yifan
 * @Date: 2026-07-05 11:10:14
 * @LastEditTime: 2026-07-05 13:34:58
 * @LastEditors: che yifan
 * @Reference:
 */
#include "slam_process.h"
#include "../estimator/parameters.h"

SlamProcess::~SlamProcess()
{
    tracker_.stopTracking();
    if (track_thread_.joinable())
        track_thread_.join();
    if (process_thread_.joinable())
        process_thread_.join();
}

void SlamProcess::init()
{
    estimator_.setParameter();

    tracker_.readIntrinsicParameter(CAM_NAMES);

    tracker_.setEstimator(&estimator_);
    estimator_.setFeatureTracker(&tracker_);
}

void SlamProcess::start()
{
    // Tracker thread: KLT → inputFeature() → Estimator
    track_thread_ = std::thread(&FeatureTracker::processTracking, &tracker_);

    // VIO thread: processMeasurements() pulls from buffers
    process_thread_ = std::thread(&Estimator::processMeasurements, &estimator_);
}

void SlamProcess::inputIMU(double t, const Eigen::Vector3d &acc, const Eigen::Vector3d &gyr)
{
    estimator_.inputIMU(t, acc, gyr);
}

void SlamProcess::inputImage(double t, const cv::Mat &img0, const cv::Mat &img1)
{
    tracker_.inputImage(t, img0, img1);
}
