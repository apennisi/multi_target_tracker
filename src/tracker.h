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

#ifndef _TRACKER_H_
#define _TRACKER_H_

#include <iostream>
#include <utility>
#include <vector>
#include <iterator>

#include "entity.h"
#include "kalman_param.h"
#include "track.h"
#include "group_track.h"
#include "hypothesis.h"
#include "detection.h"
#include "hungarianAlg.h"
#include "utils.h"

namespace ATracker
{
  typedef std::shared_ptr<Entity> Entity_ptr;
  typedef std::vector<Entity_ptr> Entities;
      
  class Tracker
  {
    friend class Entity;
    friend class Track;
    friend class GroupTrack;
    
    private:
      typedef std::shared_ptr<Track> Track_ptr;
      typedef std::vector<Track_ptr> Tracks;
      typedef std::shared_ptr<GroupTrack> Group_ptr;
      typedef std::vector<Group_ptr> Groups;
    public:
      Tracker(const KalmanParam& _param);
      void track(Detections& _detections, const int& w, const int& h, cv::Mat& img);
    public:
      inline const void setSize(const uint& _w, const uint& _h)
      {
	width = _w;
	height = _h;
      }
      const void evolveTracks()
      {
	for(const auto& track : single_tracks)
	{
	  track->predict();
	}
	for(const auto& group : groups)
	{
	  group->predict();
	}
      }
      const Entities getTracks();
    private:
      cv::Mat associate_tracks(const Detections& _detections);
      void update_tracks(const cv::Mat& assigments, const Detections& _detections);
      void delete_tracks();
      void detect_occlusions(const cv::Mat& img);
      bool overlapRoi(const cv::Rect &_r1, const cv::Rect &_r2, const float& percentage);
      void check_multiple_detections(cv::Mat& assigments, const cv::Mat& costs);
      Detections manage_groups(const Detections& _detections, cv::Mat& img);
    private:
      KalmanParam param;
      Detections last_detection;
      Detections prev_unassigned;
      Tracks single_tracks;
      Groups groups;
      Entities tracks;
      uint width, height;
      cv::RNG rng;
      uint trackIds;
      float time;
  };
}

#endif