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

#ifndef KALMAN_H
#define KALMAN_H

#include <opencv2/opencv.hpp>

namespace ATracker
{
    class KalmanFilter
    {
      public:
	  KalmanFilter() {;}
	  KalmanFilter(const float &_x, const float &_y, const float &_w, const float &_h, const float &dt);
	  inline cv::KalmanFilter kf()
	  {
	      return KF;
	  }

	  cv::Mat predict();
	  cv::Mat correct(const int &_x, const int &_y, const int& _w, const int& _h);
	  cv::Mat getPrediction()
	  {
	    return prediction;
	  }
	  const cv::Mat B() const
	  {
	    return KF.controlMatrix;
	  }
	  const cv::Mat P() const
	  {
	    return KF.errorCovPre;
	  }
	  const cv::Mat S() const
	  {
	    return KF.errorCovPre(cv::Rect(0, 0, 2, 2)) + KF.measurementNoiseCov;
	  }
	  const cv::Size getSize()
	  {
	      return cv::Size(w, h);
	  }
	  const void setDt(const float& dt)
	  {
	    KF.transitionMatrix.at<float>(2) = dt;
	    KF.transitionMatrix.at<float>(7) = dt;
	  }

      private:
	  //the kalman filter
	  cv::KalmanFilter KF;
	  cv::Mat_<float> measurement;
	  cv::Mat processNoise;
	  cv::Mat_<float> state;
	  cv::Mat prediction;
	  uint w, h;
    };

}

#endif //KALMAN_H
