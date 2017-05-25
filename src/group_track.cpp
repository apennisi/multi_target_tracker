#include "group_track.h"

using namespace ATracker;

GroupTrack::GroupTrack(const Tracks& _tracks, const cv::Mat& img, const float& dt)
  : Entity(),  tracks(_tracks)
{
  detection = cv::Rect(0, 0, 0, 0);
  
  for(const auto& track : tracks)
  {
    labels.push_back(track->label());
    trackHists.insert(std::make_pair(track->label(), computeHist(img(track->getRect()))));
    updateRect(track->getRect());
  }
  
  isgood = true;
  rect = detection;
  kf = std::shared_ptr<KalmanFilter>(new KalmanFilter(detection.x + detection.width /  2, detection.y + detection.height, 
						      detection.width, detection.height, 1));
}

bool GroupTrack::check(const Group_ptr& _group)
{
  const Tracks& _tracks = _group->getTracks();
  for(const auto& track1 : tracks)
  {
    for(const auto& track2 : _tracks)
    {
      if(track1 == track2)
	return true;
    }
  }
  return false;
}

void GroupTrack::merge(const Group_ptr& _group, const cv::Mat& img)
{
  const Tracks& _tracks = _group->getTracks();
  for(const auto& track1 : _tracks)
  {
    bool isEqual = false;
    for(const auto& track2 : tracks)
    {
      if(track1 == track2)
      {
	isEqual = true;
	break;
      }
    }
    
    if(!isEqual)
    {
      insert(track1, img);
    }
    
  }
}

const std::string GroupTrack::label2string()
{
  std::stringstream ss;
  std::copy(labels.begin(), labels.end(), std::ostream_iterator<int>(ss, "-"));
  const std::string& s = ss.str();
  return s.substr(0, s.length()-1);
}

void GroupTrack::updateRect(const cv::Rect& r)
{
  if(detection == cv::Rect(0, 0, 0, 0))
  {
    detection = r;
  }
  else
  {
    int x_tl = fmin(detection.x, r.x);
    int y_tl = fmin(detection.y, r.y);
    int x_br = fmax(detection.x + detection.width, r.x + r.width);
    int y_br = fmax(detection.y + detection.height, r.y + r.height);
    detection = cv::Rect(x_tl, y_tl, x_br - x_tl, y_br - y_tl);
  }
}

void GroupTrack::analyze_associations(const Detections& _detections, const UIntVec& _indices, 
				      Tracks& _tracks, const KalmanParam& _params, const uint& _width, 
				      const uint& _height, const cv::Mat& img)
{
  cv::Mat assigmentsBin(cv::Size(_indices.size(), tracks.size()), CV_8UC1, cv::Scalar(0));
  cv::Mat costs(cv::Size(_indices.size(), tracks.size()), CV_32FC1);
  cv::Mat histCosts(cv::Size(_indices.size(), tracks.size()), CV_32FC1);
  cv::Mat mu;
  cv::Mat sigma;
  cv::Point2f t;
  assignments_t assignments;
  distMatrix_t cost(_indices.size() * tracks.size());
  MatVec detection_hists;
  
  for(const auto& idx : _indices)
  {
    detection_hists.push_back(computeHist(img(_detections.at(idx).getRect())));
  }

  //COMPUTE COSTS
  for(uint i = 0; i < tracks.size(); ++i)
  {
    mu = tracks.at(i)->predict();
    sigma = tracks.at(i)->S();
    t = cv::Point2f(mu.at<float>(0), mu.at<float>(1));
    
    for(uint j = 0; j < _indices.size(); ++j)
    {
      const uint& idx = _indices.at(j);
      cv::Mat detection(cv::Size(1, 2), CV_32FC1);
      detection.at<float>(0) = _detections.at(idx).x();
      detection.at<float>(1) = _detections.at(idx).y();
      costs.at<float>(i, j) =  cv::Mahalanobis(detection, cv::Mat(t), sigma.inv());
      histCosts.at<float>(i, j) = cv::compareHist(trackHists[tracks.at(i)->label()], detection_hists[j], CV_COMP_BHATTACHARYYA);
    }
  }
  
  //std::cout << costs << std::endl;
  
  //NORMALIZE COSTS
  cv::normalize(costs, costs, 1, 0,cv::NORM_MINMAX);
  
  //COMPUTE GLOBAL COSTS
  costs = .6 * costs + .4 * histCosts;
  
  for(uint i = 0; i < tracks.size(); ++i)
  {
    for(uint j = 0; j < _indices.size(); ++j)
    {
      cost.at(i + j * tracks.size() ) = costs.at<float>(i, j);
    }
  }
  
  
  AssignmentProblemSolver APS;
  APS.Solve(cost, tracks.size(), _indices.size(), assignments, 
	    ATracker::costs::AssignmentProblemSolver::optimal);
  
  for(uint i = 0; i < assignments.size(); ++i)
  {
    if(costs.at<float>(i, assignments[i]) <= 0.5 && assignments[i] != -1)
      assigmentsBin.at<uchar>(i, assignments[i]) = 1;
  }
   
  for(uint i = 0; i  < assigmentsBin.rows; ++i)
  {
    for(uint j = 0; j < assigmentsBin.cols; ++j)
    {
      if(assigmentsBin.at<uchar>(i, j) == uchar(1))	
      {
	const uint& idx = _indices.at(j);
	tracks.at(i)->correct(_detections.at(idx).x(), _detections.at(idx).y(), _detections.at(idx).w(), _detections.at(idx).h());
	tracks.at(i)->ntime_missed = 0;
      }
    }
  }
  
  cv::Mat ass_sum;
  cv::reduce(assigmentsBin, ass_sum, 1, CV_REDUCE_SUM, CV_32S);
  
  for(uint i = 0; i < ass_sum.total(); ++i)
  {
    if(ass_sum.at<int>(i) == 0)
    {
      tracks.at(i)->ntime_missed++;
    }
  }
  
  //ERASE TRACKS
  const uint& track_size = tracks.size();
  for(int i = track_size - 1; i >= 0; --i)
  {
    const cv::Point2f& p = tracks.at(i)->getPointPrediction();
    const uint& ntime_missed = tracks.at(i)->ntime_missed;
    if(p.x < 0 || p.x >= _width || p.y < 0 || p.y >= _height)
    {
      tracks.erase(tracks.begin() + i);
    }
  }
  
  auto trackOverlapping = [](const cv::Rect& _r1, const cv::Rect& _r2)
  {
    int x_tl = fmax(_r1.x, _r2.x);
    int y_tl = fmax(_r1.y, _r2.y);
    int x_br = fmin(_r1.x + _r1.width, _r2.x + _r2.width);
    int y_br = fmin(_r1.y + _r1.height, _r2.y + _r2.height);
    if(x_tl < x_br && y_tl < y_br)
    {
      const float& area = (x_br - x_tl) * (y_br - y_tl);
      return ((area / (float)_r1.area()) > .3 ||
					  (area / (float)_r2.area()) > .3 );
    }
    return false;
  };
  
  //CHEK IF THERE IS AT LEAST ONE OVERLAPPING
  bool overlapping = false;
  for(uint i = 0; i < tracks.size() - 1; ++i)
  {
    for(uint j = i + 1; j < tracks.size(); ++j)
    {
      if(trackOverlapping(tracks.at(i)->getRect(), tracks.at(j)->getRect()))
      {
	overlapping = true;
	break;
      }
    }
    if(overlapping) break;
  }
  
  if(!overlapping)
  {
    for(const auto& track : tracks)
    {
      _tracks.push_back(track);
    }
    tracks.clear();
  }
  
  //PUT BACK THE TRACKS OUT OF BOUNDING BOX
  for(int i = tracks.size() - 1; i >= 0; --i)
  {
    if(!overlapRoi(rect, tracks.at(i)->getRect()))
    {
      _tracks.push_back(tracks.at(i));
      trackHists.erase(tracks.at(i)->label());
      tracks.erase(tracks.begin() + i);
    }
  }
  
  //COMPUTE THE RECT
  labels.clear();
  for(const auto& track : tracks)
  {
    labels.push_back(track->label());
    updateRect(track->getRect());
  } 
}

bool GroupTrack::overlapRoi(const cv::Rect &_global, const cv::Rect &_local)
{
    int x_tl = fmax(_global.x, _local.x);
    int y_tl = fmax(_global.y, _local.y);
    int x_br = fmin(_global.x + _global.width, _local.x + _local.width);
    int y_br = fmin(_global.y + _global.height, _local.y + _local.height);
    
    if(x_tl < x_br && y_tl < y_br )
    {
      const float& area = (x_br - x_tl) * (y_br - y_tl);
      return ((area / (float)_local.area()) >= .7 );
    }
    return false;
}

cv::Mat GroupTrack::computeHist(const cv::Mat& img)
{
  cv::Mat hsv;
  cv::cvtColor(img, hsv, CV_BGR2HSV);
  static const int channels[] = {0, 1, 2}; 
  static const int h_bins = 64; 
  static const int s_bins = 64; 
  static const int v_bins = 64;
  static const int hist_size[] = {h_bins, s_bins, v_bins};
  static const float hranges[] = {0, 180};
  static const float sranges[] = {0, 256};
  static const float vranges[] = {0, 256};
  static const float* ranges[] = {hranges, sranges, vranges};
  static const cv::Mat mask;
  static const int dims = 3;
  cv::Mat srcs[] = {img};
  cv::Mat hist;
  cv::calcHist(srcs, sizeof(srcs), channels, mask, hist, dims, hist_size, ranges, true, false);
  cv::norm(hist, hist);
  return hist;
}

void GroupTrack::insert(const Track_ptr& track, const cv::Mat& img)
{
  tracks.push_back(track);
  labels.push_back(track->label());
  trackHists.insert(std::make_pair(track->label(), computeHist(img(track->getRect()))));
  detection = rect;
  updateRect(track->getRect());
  rect = detection;
  detection = cv::Rect(0, 0, 0, 0);
}

const cv::Mat GroupTrack::predict()
{
  const cv::Mat& prediction = kf->predict();
  const cv::Point2f& p = cv::Point2f(prediction.at<float>(0), prediction.at<float>(1));
  m_history.push_back(p);
  checkHistory();
  return prediction;
}

