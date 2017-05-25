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

#ifndef _HYPHOTHESIS_H_
#define _HYPHOTHESIS_H_

#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>

#include <iostream>
#include "lap.h"
#include "hungarianAlg.h"
#include "detection.h"
#include "kalman_param.h"
#include "track.h"
#include "utils.h"

using namespace ATracker::costs;

namespace ATracker 
{
  class Hyphothesis
  {
    private:
      typedef std::shared_ptr<Track> Track_ptr;
      typedef std::vector<Track_ptr> Tracks;
    public:
      static std::shared_ptr<Hyphothesis> instance();
      void new_hyphothesis(const cv::Mat& dummmy_assignments, Tracks& tracks, const Detections& detections, const uint& w, const uint& h,
	const uint& new_hyp_dummy_costs, Detections& prev_unassigned, const KalmanParam& param);
    private:
      static std::shared_ptr<Hyphothesis> m_instance;
    private:
      // 15 sig. digits for 0<=real(z)<=171
      // coeffs should sum to about g*g/2+23/24	
      constexpr static float g = 4.7422;
      constexpr static float sq2pi=  2.5066282746310005024157652848110;
    private:
      Hyphothesis() { ; }
      float beta_likelihood(const cv::Point2f& prev_unassigned, const float& alpha, const float& beta, const uint& w, const uint& h);
      float gamma(const float& z);
  };
}

#endif