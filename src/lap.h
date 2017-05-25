/************************************************************************
*
*  lap.h
   version 1.0 - 21 june 1996
   author  Roy Jonker, MagicLogic Optimization Inc.
   modified by Andrea Pennisi
   
   header file for LAP
*
**************************************************************************/

#ifndef _LAP_H_
#define _LAP_H_

#include <stdio.h>
#include <iostream>
#include <vector>
#include <memory>
#include <limits>

namespace ATracker
{
  namespace costs
  {
    class LapCost
    {
      public:
	static std::shared_ptr<LapCost> instance();
	int lap(int dim, const std::vector< std::vector<int> >& assigncost,
		    std::vector<int>& rowsol, std::vector<int>& colsol, std::vector<int>& u, std::vector<int>& v);
	void checklap(int dim, int **assigncost,
			int *rowsol, int *colsol, int *u, int *v);
      private:
	typedef int row;
	typedef int col;
	typedef int cost;
      private:
	static std::shared_ptr<LapCost> m_instance;
	LapCost() { ; }
      private:
	constexpr static int BIG = std::numeric_limits<int>::max();
	
    };

  }
}

#endif