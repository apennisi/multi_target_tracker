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

#ifndef _TRACK_H_
#define _TRACK_H_

#include <iostream>
#include <memory>

#include "utils.h"
#include "entity.h"
#include "kalman.h"
#include "kalman_param.h"

namespace ATracker
{
  class Track : public Entity
  {
    friend class GroupTrack;
    friend class Tracker;
    
    public:
      Track() { ; }
      Track(const float& _x, const float& _y, const float& _w, const float& _h, const KalmanParam& _param);
      const cv::Rect getRect();
    protected:
      const cv::Mat predict();
      const cv::Mat correct(const float& _x, const float& _y, const float& _w, const float& _h);
    protected:
      bool operator==(const Track& _compare)
      {
	return m_label == _compare.label();
      }
      const uint nTimePropagation() const
      {
	return ntimes_propagated;
      }
       const cv::Mat getPrediction() const
      {
	return kf->getPrediction();
      }
      const cv::Point getPointPrediction() 
      {
	const cv::Mat& prediction = kf->getPrediction();
	return cv::Point2f(prediction.at<float>(0), prediction.at<float>(1));
      }
      const cv::Point2f getPosition()
      {
	const cv::Mat& prediction = kf->getPrediction();
        return cv::Point2f(prediction.at<float>(0), prediction.at<float>(1));
      }
      const double getTime() const
      {
          return time_in_sec;
      }
      const int label() const
      {
	return m_label;
      }
      void setLabel(const uint& _label)
      {
	m_label = _label;
      }
    private:
      const std::string label2string();
    private:
      int m_label;
      uint ntimes_propagated;
      double time;
      double time_in_sec;
      uint ntime_missed;
  };
}

#endif