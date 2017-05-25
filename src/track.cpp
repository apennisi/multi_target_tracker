#include "track.h"

using namespace ATracker;

Track::Track(const float& _x, const float& _y, const float& _w, const float& _h, const KalmanParam& _param)
  : Entity()
{
  kf = std::shared_ptr<KalmanFilter>(new KalmanFilter(_x, _y, _w, _h, _param.dt()));
  ntimes_propagated = 0;
  ntime_missed = 0;
  isgood = false;
  m_label = -1;
  time = (double)cv::getTickCount();
}

const cv::Mat Track::predict()
{
  const cv::Mat& prediction = kf->predict();
  m_history.push_back(cv::Point2f(prediction.at<float>(0), prediction.at<float>(1)));
  checkHistory();
  return prediction;
}

const cv::Mat Track::correct(const float& _x, const float& _y, const float& _w, const float& _h)
{
  ntimes_propagated++;
  time_in_sec = ((double) cv::getTickCount()  - time) / cv::getTickFrequency();
  w = _w;
  h = _h;
  return kf->correct(_x, _y, _w, _h);
}

const cv::Rect Track::getRect()
{
  const cv::Mat& prediction = kf->getPrediction();
  const cv::Size& size = kf->getSize();
  return cv::Rect(cvRound(prediction.at<float>(0) - (size.width / 2)), cvRound(prediction.at<float>(1) - size.height),
		    size.width, size.height);
}

const std::string Track::label2string()
{
  std::stringstream ss;
  ss << m_label;
  return ss.str();
}

