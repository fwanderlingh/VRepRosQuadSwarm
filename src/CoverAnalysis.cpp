//	Copyright (c) 2014, Francesco Wanderlingh. 					//
//	All rights reserved.										//
//	License: BSD (http://opensource.org/licenses/BSD-3-Clause)	//

/*
 * CoverAnalysis.cpp
 *
 *  Created on: May 23, 2014
 *      Author: francescow
 */

#include <quadcopter_ctrl/CoverAnalysis.h>
#include <quadcopter_ctrl/termColors.h>
#include <algorithm>   /* max_element */
#include <cstdio>
#include <iostream>
#include <iterator>
#include <numeric>      /* multiply, accumulate */
#include <cmath>

using std::cout;
using std::endl;
using std::vector;
using std::max_element;

CoverAnalysis::CoverAnalysis()
{
  // TODO Auto-generated constructor stub

}

CoverAnalysis::CoverAnalysis(vector<vector<int> > paths_vec, int numberOfRobots, int numberOfNodes)
{
  Paths = paths_vec;
  numRobots = numberOfRobots;
  numNodes = numberOfNodes;

  vecOfLengths(pathLengths);
  totalLength = totalPathsLength();
}

CoverAnalysis::~CoverAnalysis()
{
  // TODO Auto-generated destructor stub
}


void CoverAnalysis::vecOfLengths(vector<int>& vecOfLenghts){

  for (vector< vector<int> >::iterator itr = Paths.begin(); itr != Paths.end(); ++itr){
    vecOfLenghts.push_back( itr->size() - 1 );
  }
}


int CoverAnalysis::totalPathsLength(){

  int _totalLength = std::accumulate(pathLengths.begin(), pathLengths.end(), 0.0);
  //printf("%sTotal Paths Length: %d%s\n", TC_MAGENTA, _totalLength, TC_NONE);
  return _totalLength;

}


int CoverAnalysis::getLongestPath(){
  /// Returns the length of the longest path

  return *max_element(pathLengths.begin(), pathLengths.end());
  //printf("%sMax Path Length: %d%s\n", TC_MAGENTA, max, TC_NONE);

}


double CoverAnalysis::getStDev(){

  double mean = totalLength / pathLengths.size();

  std::vector<double> diff(pathLengths.size());
  std::transform(pathLengths.begin(), pathLengths.end(), diff.begin(),
                 std::bind2nd(std::minus<double>(), mean));
  double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
  double stdev = std::sqrt(sq_sum / pathLengths.size());

  return stdev;

}




