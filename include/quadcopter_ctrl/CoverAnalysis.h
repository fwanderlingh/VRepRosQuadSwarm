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


public:
  CoverAnalysis();
  CoverAnalysis(vector<vector<int> > paths_vec, int numberOfRobots, int numberOfNodes);
  virtual ~CoverAnalysis();

  void loadPaths(vector<vector<int> > paths_vec);
  int longestPath();
  int totalPathsLength();
  float pathsOverlapIndex();
  float calcMapOverlap(vector<int> map);

};

#endif /* COVERANALYSIS_H_ */
