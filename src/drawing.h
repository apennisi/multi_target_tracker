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

#ifndef _DRAWING_H_
#define _DRAWING_H_

#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>

#include "utils.h"

namespace ATracker
{
  namespace tools
  {
    class Drawing
    {
      private:
	typedef std::shared_ptr<Drawing> Drawing_ptr;
      public:
	static Drawing_ptr instance();
	const void history(const Points& _history, const cv::Scalar& color, cv::Mat& img) const;
	const void arrow(const cv::Point& start_point, const cv::Point& end_point, const cv::Scalar& color, cv::Mat& img) const;
	const void rectangle(const cv::Rect& _rect, const cv::Scalar& color, cv::Mat& img) const;
      private:
	static Drawing_ptr m_instance;
	Drawing() { ; }
    };
  }
}

#endif