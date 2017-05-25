#include "drawing.h"

using namespace ATracker;
using namespace ATracker::tools;

std::shared_ptr<Drawing> Drawing::m_instance = nullptr;

std::shared_ptr< Drawing > Drawing::instance()
{
  if(!m_instance)
  {
    m_instance = Drawing_ptr(new Drawing);
  }
  return m_instance;
}

const void Drawing::arrow(const cv::Point& start_point, const cv::Point& end_point, 
			  const cv::Scalar& color, cv::Mat& img) const
{
  cv::line(img, start_point, end_point, color, 2);
}

const void Drawing::history(const Points& _history, const cv::Scalar& color, cv::Mat& img) const
{
  for(const auto& point : _history)
  {
    cv::circle(img, point, 1, color, -1);
  }
}

const void Drawing::rectangle(const cv::Rect& _rect, const cv::Scalar& color, cv::Mat& img) const
{
  cv::rectangle(img, _rect, color, 2);
}

