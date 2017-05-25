#include "kalman.h"

using namespace ATracker;


KalmanFilter::KalmanFilter(const float &_x, const float &_y, const float &_w, const float &_h, const float &dt)
{
    
    measurement = cv::Mat_<float>(2, 1, CV_32F);
    measurement.setTo(cv::Scalar(0));

    KF = cv::KalmanFilter(4, 2, 0);
    state = cv::Mat_<float>(4, 1);
    processNoise = cv::Mat(4, 1, CV_32F);
    measurement = cv::Mat_<float>(2, 1);
    measurement.setTo(cv::Scalar(0));

    KF.statePre.at<float>(0, 0) = _x;
    KF.statePre.at<float>(1, 0) = _y;
    KF.statePre.at<float>(2) = 0;
    KF.statePre.at<float>(3) = 0;

    KF.statePost.at<float>(0) = _x;
    KF.statePost.at<float>(1) = _y;
    KF.statePost.at<float>(2) = 0;
    KF.statePost.at<float>(3) = 0;

    KF.transitionMatrix = (cv::Mat_<float>(4, 4) << 1,0,dt,0,
                                                     0,1,0,dt,
                                                     0,0,1,0,
                                                     0,0,0,1);

    cv::setIdentity(KF.measurementMatrix);
    //cv::setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-10));

    KF.processNoiseCov=(cv::Mat_<float>(4, 4) <<
            pow(dt,4.0)/4.0,    0,  pow(dt,3.0)/2.0,    0,
            0,  pow(dt,4.0)/4.0,   0,  pow(dt,3.0)/2.0,
            pow(dt,3.0)/2.0,    0,  pow(dt,2.0),    0,
            0,  pow(dt,3.0)/2.0,  0,    pow(dt,2.0));

    KF.processNoiseCov*=.3;
    cv::setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1e-1));
    cv::setIdentity(KF.errorCovPost, cv::Scalar::all(.1));
    w = _w;
    h = _h;
    prediction = cv::Mat(cv::Size(4, 1), CV_32FC1);
    prediction.at<float>(0) = _x;
    prediction.at<float>(1) = _y;
    prediction.at<float>(2) = 0;
    prediction.at<float>(3) = 0;
}

cv::Mat KalmanFilter::predict()
{
    prediction = KF.predict();
    KF.statePre.copyTo(KF.statePost);
    KF.errorCovPre.copyTo(KF.errorCovPost);

    return prediction;
}

cv::Mat KalmanFilter::correct(const int &_x, const int &_y, const int& _w, const int& _h)
{
    measurement(0) = _x;
    measurement(1) = _y;
    w = _w;
    h = _h;
    const cv::Mat& estimated = KF.correct(measurement);
    return estimated;
}
