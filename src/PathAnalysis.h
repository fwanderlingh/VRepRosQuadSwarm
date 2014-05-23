/*
 * PathAnalysis.h
 *
 *  Created on: May 23, 2014
 *      Author: francescow
 */

#ifndef PATHANALYSIS_H_
#define PATHANALYSIS_H_

#include <vector>

using std::vector;

class PathAnalysis
{
  vector< vector<int>> paths;

  float findLongest();  /// Return the length of the longest path
  float calculateOverlap();     /// Calculate the overlap over all the paths

public:
  PathAnalysis();
  PathAnalysis(vector< vector<int>> paths_vec);
  virtual ~PathAnalysis();

  void loadPaths(vector< vector<int>> paths_vec);
};

#endif /* PATHANALYSIS_H_ */
