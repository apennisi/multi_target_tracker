#include "hypothesis.h"

using namespace ATracker;

std::shared_ptr<Hyphothesis> Hyphothesis::m_instance = nullptr;

std::shared_ptr< Hyphothesis > Hyphothesis::instance()
{
  if(!m_instance)
  {
    m_instance = std::shared_ptr<Hyphothesis>(new Hyphothesis);
  }
  
  return m_instance;
}

void Hyphothesis::new_hyphothesis(const cv::Mat& dummmy_assignments, Tracks& tracks, const Detections& detections, const uint& w, const uint& h, 
				     const uint& new_hyp_dummy_costs, Detections& prev_unassigned, const KalmanParam& param)
{
  cv::Mat assignments = dummmy_assignments.clone();
  
  assignments.setTo(1, assignments == 255);
  
  //Unassigned observations
  cv::Mat ass_sum;
  cv::reduce(assignments, ass_sum, 0, CV_REDUCE_SUM, CV_32S);
   
  cv::Mat tmp_unassigned = ass_sum == 0;
  cv::Mat unassigned;
  cv::findNonZero(tmp_unassigned, unassigned);
  
  uint nunassigned = unassigned.total();
  
  cv::Mat new_hyp = cv::Mat(cv::Size(prev_unassigned.size(), nunassigned), CV_32FC1, cv::Scalar(0));
  distMatrix_t cost(prev_unassigned.size() * nunassigned);
  assignments_t new_assignments;
  
  if(cost.size() != 0)
  {   
    float amp;
    cv::Point2f elem;
    const uint& nHrows = (uint) new_hyp.rows;
    const uint& nHcols = (uint) new_hyp.cols;
    for(uint i = 0; i < nHrows; ++i)
    {
      const uint idx = unassigned.at<cv::Point>(i).x;
      elem = (cv::Point2f)detections.at(idx)();
      for(uint j = 0; j < nHcols; ++j)
      {
	// How "good" are tracks that started at one of the previously
	// unassigned observations? The cost amplification is high in 
	// the middle, but low on the sides. 
	amp = beta_likelihood(prev_unassigned.at(j)(), 1.5, 1.5, w, h);
	
	elem = prev_unassigned.at(j)() - elem;
	
	new_hyp.at<float>(i, j) = amp * sqrt(elem.x * elem.x + elem.y * elem.y);
	cost.at(i + j * new_hyp.rows ) = amp * new_hyp.at<float>(i, j);
      }
    }
    
    
    const uint& rows = new_hyp.rows;
    const uint& cols = new_hyp.cols;

    uint n_w, n_h;
    if(rows < cols)
    {
      //std::cout << "cols < rows" << std::endl;
      n_w = 2*cols;
      n_h = 2*(rows + (cols - rows));
      //std::cout << n_w << " " << n_h << std::endl;
    }
    else if (rows > cols)
    {
      //std::cout << "rows > cols" << std::endl;
      n_w = 2*(cols + (rows - cols));
      n_h = 2 * rows;
      //std::cout << n_w << " " << n_h << std::endl;
    }
    else
    {
      n_w = cols; 
      n_h = rows;
    }
    
    
    cv::Mat lap_costs = cv::Mat(cv::Size(n_w, n_h), CV_32SC1, cv::Scalar(new_hyp_dummy_costs));
    new_hyp.copyTo(lap_costs(cv::Rect(0, 0, cols, rows)));
    
    
    std::vector<int> colsol(n_w);
    std::vector<int> rowsol(n_w);
    
    std::vector<int> u(n_w), v(n_w);
    
    std::vector< std::vector<int> > lap_costs_matrix(n_h, std::vector<int>(n_w));
    for(uint i = 0; i < n_h; ++i)
    {
      for(uint j = 0; j < n_w; ++j)
      {
	lap_costs_matrix[i][j] = lap_costs.at<int>(i, j);
      }
    }
    
    LapCost::instance()->lap(n_w, lap_costs_matrix, rowsol, colsol, u, v);
    
    new_hyp = cv::Mat(cv::Size(n_w, n_h), CV_32SC1, cv::Scalar(0));
    
    
    for(uint i = 0; i < n_w; ++i)
    {
      new_hyp.at<int>((i*n_w)+rowsol.at(i)) = 1;
    }
  
    
    cv::Mat new_assigments = new_hyp(cv::Rect(0, 0, cols, rows));
    new_assigments.convertTo(new_assigments, CV_32S);

    if(new_assigments.total() > 1)
    {
      ass_sum = cv::Mat(cv::Size(1, rows), new_assigments.type(), cv::Scalar(0));
      const uint& nAcols = new_assigments.cols;
      for(uint i = 0; i < nAcols; ++i)
      {
	ass_sum += new_assigments(cv::Rect(i, 0, 1, rows));
      }
    }
    else
    {
      ass_sum = new_assigments.clone();
    }
    
    
    if(ass_sum.cols > 1)
    {
      std::cout << "ass_sum.cols > 1 " << std::endl;
      exit(-1);
    }
    
    tmp_unassigned = ass_sum == 1;
    cv::Mat new_unassigned;
    cv::findNonZero(tmp_unassigned, new_unassigned);
    
    if(new_unassigned.total() != 0)
    {
      const uint& nUtotal = new_unassigned.total();
      for(uint i = 0; i < nUtotal; ++i)
      {
	const int idx = unassigned.at<cv::Point>(new_unassigned.at<cv::Point>(i).y).x;
	Track_ptr tr(new Track(detections.at(idx).x(), detections.at(idx).y(), detections.at(idx).w(), detections.at(idx).h(), param));
	tracks.push_back(tr);
      }
    }
  }
  
  prev_unassigned.clear();
  const uint& dSize = detections.size();
  for(uint i = 0; i < dSize; ++i)
  {
    if(ass_sum.at<int>(i) == 0 || 
	(new_assignments.size() > 0 && std::find(new_assignments.begin(), new_assignments.end(), i) == new_assignments.end()))
    {
      prev_unassigned.push_back(detections.at(i));
    }
  }
  
 
}

float Hyphothesis::beta_likelihood(const cv::Point2f& prev_unassigned, const float& alpha, const float& beta, const uint& w, const uint& h)
{
  const float& x = prev_unassigned.x / float(w);
  const float& y = prev_unassigned.y / float(h);
  
  const float& norm = gamma(alpha + beta) / gamma(alpha) * gamma(beta);
  
  const float& likelihood_x = norm * std::pow(x, alpha - 1) * std::pow((1 - x),(beta - 1)); 
  const float& likelihood_y = norm * std::pow(y, alpha - 1) * std::pow((1 - y),(beta - 1));
  
  return likelihood_x * likelihood_y;
}

static std::vector<float> c =  {0.99999999999999709182, 57.156235665862923517, -59.597960355475491248,
						14.136097974741747174, -0.49191381609762019978, .000033994649984811888699,
						.000046523628927048575665, -.000098374475304879564677, .00015808870322491248884,
						-.00021026444172410488319, .00021743961811521264320, -.00016431810653676389022,
						  .000084418223983852743293, -.000026190838401581408670, .0000036899182659531622704};

float Hyphothesis::gamma(const float& z)
{
  const float& m_z = z - 1;
  const float& m_zh = m_z + .5;
  const float& m_zgh = m_zh + g;
  
  //trick for avoiding FP overflow above z=141
  const float& zp = std::pow(m_zgh, m_zh*.5);
  
  float ss = .0;
  for(int i = 13; i >= 0; --i)
  {
    ss += (c.at(i+1) / (z + (i+1)));
  }
  
  return (sq2pi * (c.at(0) + ss) * (zp * exp(-m_zgh) * zp));
}


