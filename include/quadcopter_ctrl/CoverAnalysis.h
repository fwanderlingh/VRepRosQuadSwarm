/*
 * CoverAnalysis.h
 *
 *  Created on: May 23, 2014
 *      Author: francescow
 */

#ifndef COVERANALYSIS_H_
#define COVERANALYSIS_H_

#include <vector>

using std::vector;

class CoverAnalysis
{
  int numRobots;
  int numNodes;
  vector<vector<int> > Paths;


  int longestPath();
  int totalPathsLength();
  float calcPathOverlap();
  float calcMapOverlap();

public:
  CoverAnalysis();
  CoverAnalysis(vector<vector<int> > paths_vec, int numberOfRobots, int numberOfNodes);
  virtual ~CoverAnalysis();

  void loadPaths(vector<vector<int> > paths_vec);
};

#endif /* COVERANALYSIS_H_ */
