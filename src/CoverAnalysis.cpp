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
#include <algorithm>
#include <cstdio>
#include <iostream>
#include <iterator>
#include <numeric>      /* multiply, accumulate */

using std::cout;
using std::endl;
using std::vector;
using std::max_element;

CoverAnalysis::CoverAnalysis()
{
  // TODO Auto-generated constructor stub
  cout << '\a';
  cout << '\a';
}

CoverAnalysis::CoverAnalysis(vector<vector<int> > paths_vec, int numberOfRobots, int numberOfNodes)
{
  Paths = paths_vec;
  numRobots = numberOfRobots;
  numNodes = numberOfNodes;
  cout << '\a';
  cout << '\a';
}

CoverAnalysis::~CoverAnalysis()
{
  // TODO Auto-generated destructor stub
}


int CoverAnalysis::longestPath(){
  /// Returns the length of the longest path

  vector<int> pathLengths;

  for (vector< vector<int> >::iterator itr = Paths.begin(); itr != Paths.end(); ++itr){
    pathLengths.push_back( itr->size() - 1 );
  }

  int max = *max_element(pathLengths.begin(), pathLengths.end());
  printf("%sMax Path Length: %d%s\n", TC_MAGENTA, max, TC_NONE);

  return max;

}

int CoverAnalysis::totalPathsLength(){

  int totalLength = 0;
  /// Return the length of the longest path
  for (vector< vector<int> >::iterator itr = Paths.begin(); itr != Paths.end(); ++itr){

    totalLength +=  (itr->size() - 1) ;
  }

  printf("%sTotal Paths Length: %d%s\n", TC_MAGENTA, totalLength, TC_NONE);

  return totalLength;

}

float CoverAnalysis::pathsOverlapIndex(){
  /// Calculate the overlap over all the paths

  int pathOverlap = totalPathsLength() - numRobots*2 - numNodes;

  cout << "Path Overlap Index: " << pathOverlap << endl;

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
