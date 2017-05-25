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

#ifndef _KALMAN_PARAM_H_
#define _KALMAN_PARAM_H_

#include <iostream>
#include <fstream>
#include <algorithm>
#include <boost/algorithm/string.hpp>

namespace ATracker
{
  class KalmanParam
  {
    public:
      KalmanParam() { ; }
      void read(const std::string& filename);
      const uint minpropagate() const
      {
	return min_propagate;
      }
      const uint dt() const
      {
	return d_t;
      }
      const uint assocdummycost() const
      {
	return assoc_dummy_cost;
      }
      const uint newhypdummycost() const
      {
	return new_hyp_dummy_cost;
      }
      const uint maxmissed() const
      {
	return max_missed;
      }
      KalmanParam& operator=(const KalmanParam& _param)
      {
	this->min_propagate = _param.minpropagate();
	this->d_t = _param.dt();
	this->assoc_dummy_cost = _param.assocdummycost();
	this->new_hyp_dummy_cost = _param.newhypdummycost();
	this->max_missed = _param.maxmissed();
	return *this;
      }
      const void print() const
      {
	std::cout << "[MINPROPAGATE]: " << min_propagate << std::endl;
	std::cout << "[MAXMISSED]: " << max_missed << std::endl;
	std::cout << "[ASSOCDUMMYCOST]: " << assoc_dummy_cost << std::endl;
	std::cout << "[NEW_HYP_DUMMY_COST]: " << new_hyp_dummy_cost << std::endl;
	std::cout << "[DT]: " << d_t << std::endl;
      }
    private:
      uint max_missed;
      uint min_propagate;
      uint d_t;
      uint assoc_dummy_cost;
      uint new_hyp_dummy_cost;
  };
}

#endif