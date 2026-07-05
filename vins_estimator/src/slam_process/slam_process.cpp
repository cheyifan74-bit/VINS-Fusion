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
    // 1. Estimator
    estimator_.setParameter();

    // 2. FeatureTracker
    tracker_.readIntrinsicParameter(CAM_NAMES);

    // 3. Bind
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

// ---- public interfaces ----

void SlamProcess::inputIMU(double t, const Eigen::Vector3d &acc, const Eigen::Vector3d &gyr)
{
    estimator_.inputIMU(t, acc, gyr);
}

void SlamProcess::inputImage(double t, const cv::Mat &img0, const cv::Mat &img1)
{
    tracker_.inputImage(t, img0, img1);
}

void SlamProcess::inputFeature(double t, const std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>> &featureFrame)
{
    estimator_.inputFeature(t, featureFrame);
}

void SlamProcess::restart()
{
    estimator_.clearState();
    estimator_.setParameter();
}

void SlamProcess::changeSensorType(int use_imu, int use_stereo)
{
    estimator_.changeSensorType(use_imu, use_stereo);
}
