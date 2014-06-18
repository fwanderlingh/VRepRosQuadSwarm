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

  vector<int> pathLengths;
  int totalLength;


public:
  CoverAnalysis();
  CoverAnalysis(vector<vector<int> > paths_vec, int numberOfRobots, int numberOfNodes);
  virtual ~CoverAnalysis();


  void vecOfLengths(vector<int>& vecOfLenghts);
  int totalPathsLength();
  int getLongestPath();
  double getStDev();

  int getTotalLength() const
  {
    return totalLength;
  }
};

#endif /* COVERANALYSIS_H_ */
