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

#ifndef _GROUP_TRACK_H_
#define _GROUP_TRACK_H_

#include <map>

#include "entity.h"
#include "track.h"
#include "utils.h"
#include "hungarianAlg.h"
#include "kalman_param.h"

using namespace ATracker::costs;

namespace ATracker
{ 
  class GroupTrack : public Entity
  { 
    friend class Tracker;
    
    private:
      typedef std::shared_ptr<Track> Track_ptr;
      typedef std::vector<Track_ptr> Tracks;
      typedef std::shared_ptr<GroupTrack> Group_ptr;
      typedef std::vector<Group_ptr> Groups;
      typedef std::map<uint, cv::Mat> HistMap;
    public:
      GroupTrack() { ; }
      GroupTrack(const Tracks& _tracks, const cv::Mat& img, const float& dt);
    public:
      const cv::Rect getRect()
      {
	return rect;
      }
    private:
      const std::string label2string();
      bool check(const Group_ptr& _group);
      void merge(const Group_ptr& _group, const cv::Mat& img);
      void analyze_associations(const Detections& _detections, const UIntVec& _indices, 
				      Tracks& _tracks, const KalmanParam& _params, const uint& _width, 
				      const uint& _height, const cv::Mat& img);
      const cv::Mat predict();
      void insert(const Track_ptr& track, const cv::Mat& img);
    private:
      void resetDetection()
      {
	detection = cv::Rect(0, 0, 0, 0);
      }
      void correct() 
      {
	kf->correct(detection.x + (detection.width >> 1), detection.y + detection.height, detection.width, detection.height);
	rect = detection;
      }
      const uint size() const
      {
	return tracks.size();
      }
      
      const Tracks getTracks() const
      {
	return tracks;
      }
      const UIntVec getLabels() const
      {
	return labels;
      }
    private:
      UIntVec labels;
      cv::Rect rect;
      cv::Rect detection;
      Tracks tracks;
      HistMap trackHists;
    private:
      void updateRect(const cv::Rect& r);
      bool overlapRoi(const cv::Rect &_r1, const cv::Rect &_r2);
      cv::Mat computeHist(const cv::Mat& img);
  };
}

#endif