/*
 * VrpGreedyAstar.h
 *
 *  Created on: Apr 29, 2014
 *      Author: francescow
 */

#ifndef VRPGREEDYASTAR_H_
#define VRPGREEDYASTAR_H_

#include <vector>
#include <fstream>
#include <cstring>
#include "graphStructs.h"
#include "Astar.h"

using std::vector;


class VrpGreedyAstar
{
  int gridSizeX;
  int gridSizeY;
  int numRobots;
  bool neighbourAvailable;
  vector<graphNode> graphNodes;
  vector<int> unvisitedNodes;
  vector<int> access_vec;
  double minDist;			//distance between two adjacent blocks
  vector< vector<int> > Paths;
  vector<int> pathTentative;
  vector<int> astarTent;
  vip choice;
  double deltaBest;
  double deltavip;
  double bigL;
  double liMin;

  int numFreeNodes;

  vector< vector<int> >::iterator it;
  vector< vector<int> >::iterator itr; // Iterator for Paths
  vector<int>::iterator itc; // Iterator for position inside path
  vector<int>::size_type v; // Iterator for unvisited nodes


  void checkBest(bool neighbour);
  double pathLength(vector<int> &pathToEvaluate);
  void loadMatrixFile(std::string acc_matrix_path);
  double dist(graphNode &a, graphNode &b);
  void init();
  void printTentative();


public:
  VrpGreedyAstar();
  VrpGreedyAstar(std::string acc_matrix_path);
  VrpGreedyAstar(std::string acc_matrix_path, int agents);
  virtual ~VrpGreedyAstar();
  void solve();

  const vector<graphNode>& getGraphNodes() const
  {
    return graphNodes;
  }

  int getNumRobots() const
  {
    return numRobots;
  }

  int getNumFreeNodes() const
  {
    return numFreeNodes;
  }

  const vector<vector<int> >& getPaths() const
  {
    return Paths;
  }
};



#endif /* VRPGREEDYASTAR_H_ */
