#include <iostream>
#include <memory>
#include <string>
#include <fstream>

#include "track.h"
#include "tracker.h"
#include "imagemanager.h"
#include "kalman_param.h"


std::map<int, std::vector< std::vector< std::string > > > petsReading(const std::string& _gt)
{
    std::ifstream file(_gt);
    if(!file.is_open())
    {
      std::cerr << "Error: cannot read the file: " << _gt << std::endl;
      exit(-1);
    }
    
    
    std::string line;
    std::string delimiter(",");
    std::map<int, std::vector< std::vector< std::string > > > pets;
   

    while (std::getline(file, line))
    {
      if(line.size() == 0) continue;
      auto start = 0U;
      auto end = line.find(delimiter);
      std::vector<std::string> row;
      while (end != std::string::npos)
      {
	row.push_back(line.substr(start, end - start));
	start = end + delimiter.length();
	end = line.find(delimiter, start);
      }
      row.push_back(line.substr(start, end - start));
      
      const uint& n = atoi(row[1].c_str());
      std::vector< std::vector<std::string> > detections;
      uint j = 2;
      for(uint i = 0; i < n; ++i)
      {
	std::vector<std::string> currDetections;
	try
	{
	  currDetections.push_back(row[j]);
	  currDetections.push_back(row[++j]);
	  currDetections.push_back(row[++j]);
	  currDetections.push_back(row[++j]);
	}
	catch(...)
	{
	  std::cerr << "Error: cannot read parse:\n " << line << std::endl;
	  exit(-1);
	}
	++j;
	detections.push_back(currDetections);
      }
  
      pets.insert(std::make_pair(atoi(row[0].c_str()), detections));
    }

    return pets;
}

int main(int argc, char** argv)
{
  if(argc != 4)
  {
    std::cerr << "Usage: " << std::endl;
    std::cout << "\t ./" << argv[0] << "detection_file_name.txt image_folder kalman_param.txt" << std::endl;
  }
  
  std::map<int, std::vector< std::vector< std::string > > > detections = petsReading(argv[1]);
  cv::Mat image, imageClone, imageTracks;
  
  std::vector< std::vector < std::string > > curr;
  ImageManager img(argv[2]);
  std::string param_filename(argv[3]);
  ATracker::KalmanParam param;
  param.read(param_filename);
  
  ATracker::Tracker tr(param);
  
  std::vector<cv::Rect> rects;
  std::vector<cv::Point2f> points;
  std::vector<ATracker::Detection> dets;
  
  cv::Rect rect;
  const double& milliseconds = 1000 / 7;
  
  
  for(uint i = 0; i < detections.size(); ++i)
  {
    rects.clear();
    points.clear();
    image = cv::imread(img.next(1));
    imageClone = image.clone();
    imageTracks = image.clone();
    int w = image.cols;
    int h = image.rows;
    
    curr = detections[i+1];
    std::stringstream ss;
    int j = 0;
    for(const auto &c : curr)
    {
      rect = cv::Rect(cvRound(atof(c[0].c_str())), cvRound(atof(c[1].c_str())),
                  cvRound(atof(c[2].c_str())), cvRound(atof(c[3].c_str())));
      
      rects.push_back(rect);
      
      points.push_back(cv::Point2f(rect.x + (rect.width >> 2), rect.y + rect.height));
      ATracker::Detection d(rect.x + (rect.width >> 2), rect.y + rect.height, rect.width, rect.height);
      dets.push_back(d);
      
      cv::rectangle(imageClone, rect, cv::Scalar(0, 0, 255), 3 );
      
      ss.str("");
      ss << j;
      
      cv::putText(imageClone, ss.str(), cv::Point(cvRound(atof(c[0].c_str())), cvRound(atof(c[1].c_str()))), cv::FONT_HERSHEY_SIMPLEX,
		  0.55, cv::Scalar(0, 255, 0), 2, CV_AA);
      
      ++j;
    }
    tr.setSize(w, h);
    tr.track(dets, w, h, image);
    
    Entities tracks = tr.getTracks();
    for(auto& track : tracks)
    {
      track->drawTrack(imageTracks);
    }

    cv::imshow("DETECTIONS", imageClone);
    cv::imshow("TRACKS", imageTracks);
    cv::waitKey(cvRound(milliseconds));
    dets.clear();  
  }
}
