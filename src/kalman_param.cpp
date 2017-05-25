#include "kalman_param.h"

using namespace ATracker;


void KalmanParam::read(const std::string& filename) 
{
  std::ifstream file;
  
  try
  {
    file.open(filename);
  }
  catch(...)
  {
    std::cerr << "Cannot open " << filename << std::endl;
    file.close();
    exit(-1);
  }
  
  if(!file.is_open())
  {
    std::cerr << "Error: file " << filename << " not found!" << std::endl;
    exit(-1);
  }
  
  
  std::string line;
  while(std::getline(file, line))
  {
      std::remove_if(line.begin(), line.end(), isspace);
      if(line.empty())
      {
	continue;
      }
      else if(line.find("[ASSOC_DUMMY_COST]") != std::string::npos)
      {
	std::getline(file, line);
	try
	{
	  assoc_dummy_cost = atoi(line.c_str());
	}
	catch(...)
	{
	  std::cerr << "Error in converting the ASSOC_DUMMY_COST: " << line << std::endl;
	  exit(-1);
	}
      }
      else if(line.find("[NEW_HYP_DUMMY_COST]") != std::string::npos)
      {
	std::getline(file, line);
	try
	{
	  new_hyp_dummy_cost = atoi(line.c_str());
	}
	catch(...)
	{
	  std::cerr << "Error in converting the NEW_HYP_DUMMY_COST: " << line << std::endl;
	  exit(-1);
	}
      }
      else if(line.find("[MINPROPAGATE]") != std::string::npos)
      {
	std::getline(file, line);
	try
	{
	  min_propagate = atoi(line.c_str());
	}
	catch(...)
	{
	  std::cerr << "Error in converting the MINPROPAGATE: " << line << std::endl;
	  exit(-1);
	}
      }
      else if(line.find("[MAXMISSED]") != std::string::npos)
      {
	std::getline(file, line);
	try
	{
	  max_missed = atoi(line.c_str());
	}
	catch(...)
	{
	  std::cerr << "Error in converting the MAXMISSED: " << line << std::endl;
	  exit(-1);
	}
      }
      else if(line.find("[DT]") != std::string::npos)
      {
	std::getline(file, line);
	try
	{
	  d_t = atoi(line.c_str());
	}
	catch(...)
	{
	  std::cerr << "Error in converting the DT: " << line << std::endl;
	  exit(-1);
	}
      }
      else
      {
	std::cerr << "Option: " << line << " does not exist!" << std::endl;
	exit(-1);
      } 
  }
 
  file.close();
  
  print();
}
