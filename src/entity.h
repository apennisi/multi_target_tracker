/*
 *  ATracker 
 *  Copyright 2017 Andrea Pennisi
 *
 *  This file is part of AT and it is distributed under the terms of the
 *  GNU Lesser General Public License (Lesser GPL)
 *
 *
 *
 *  ATracker is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  ATracker is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with ATracker.  If not, see <http://www.gnu.org/licenses/>.
 *
 *
 *  ATracker has been written by Andrea Pennisi
 *
 *  Please, report suggestions/comments/bugs to
 *  andrea.pennisi@gmail.com
 *
 */

#ifndef _ENTITY_H_
#define _ENTITY_H_

#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>

#include "kalman.h"
#include "drawing.h"
#include "utils.h"

namespace ATracker
{
  class Entity
  {   
    friend class Tracker;
    
    public:
      Entity() { ; }
      virtual const cv::Rect getRect() = 0;
    public:
      const cv::Point2f getSpeed()
      {
	const cv::Mat& prediction = kf->getPrediction();
        return cv::Point2f(prediction.at<float>(2), prediction.at<float>(3));
      }
      const void drawTrack(cv::Mat& img, bool history = false, bool direction = true)
      {
	if(isgood)
	{
	  const cv::Rect& r = getRect();
	  const cv::Point& start_point = cv::Point(r.x + (r.width >> 1), r.y + (r.height >> 1));
	  const cv::Point2f& speed = getSpeed();
	  const cv::Point2f& end_point = cv::Point(start_point.x + .5*speed.x, start_point.y + .5*speed.y);
	  tools::Drawing::instance()->rectangle(r, color, img);
	  cv::putText(img, label2string().c_str(), r.tl(), cv::FONT_HERSHEY_SIMPLEX,
		0.55, cv::Scalar(0, 255, 0), 2, CV_AA);
	  if(direction) tools::Drawing::instance()->arrow(start_point, end_point, color, img);
	  if(history) tools::Drawing::instance()->history(m_history, color, img);
	}
      }
      inline const Points history() const
      {
	return m_history;
      }
    protected:
      typedef std::shared_ptr<KalmanFilter> Kalman_ptr;
    protected:
      virtual const std::string label2string() = 0;
    protected:
      void setColor(const cv::Scalar& _color)
      {
	color = _color;
      }
      const void setDt(const float& dt)
      {
	kf->setDt(dt);
      }
      inline const cv::Mat B() const
      {
	return kf->B();
      }
      inline const cv::Mat P() const
      {
	return kf->P();
      }
      inline const cv::Mat S() const
      {
	return kf->S();
      }
      const void checkHistory()
      {
	if(m_history.size() > history_size)
	{
	  m_history.erase(m_history.begin());
	}
      }
      inline const cv::Scalar getColor() 
      {
	return color;
      }
    protected:
      Points m_history;
      cv::Scalar color;
      Kalman_ptr kf;
      uint w, h;
      bool isgood;
    protected:
      constexpr static uint history_size = 10;
  };
}

#endif
