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
#include <algorithm>
#include <iterator>

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
}

CoverAnalysis::~CoverAnalysis()
{
  // TODO Auto-generated destructor stub
}


int CoverAnalysis::longestPath(){

  vector<int> pathLengths;
  /// Return the length of the longest path
  for (vector< vector<int> >::iterator itr = Paths.begin(); itr != Paths.end(); ++itr){

    pathLengths.push_back( itr->size() );
  }

  return *max_element(pathLengths.begin(), pathLengths.end());

}

int CoverAnalysis::totalPathsLength(){

  int totalLength;
  /// Return the length of the longest path
  for (vector< vector<int> >::iterator itr = Paths.begin(); itr != Paths.end(); ++itr){

    totalLength +=  itr->size() ;
  }

  return totalLength;

}

float CoverAnalysis::calcPathOverlap(){
  /// Calculate the overlap over all the paths

  int pathOverlap = totalPathsLength() - numRobots*2 - numNodes;

  return pathOverlap;
}

float CoverAnalysis::calcMapOverlap( vector<int> map ){
  /// Calculate the overlap over all the count Map

  int mapOverlap;

  mapOverlap = std::accumulate(map.begin(), map.end(), 0) - numNodes - numRobots ;

  return mapOverlap;
}



void CoverAnalysis::loadPaths(vector<vector<int> > paths_vec){

  Paths = paths_vec;

}
