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
  float minDist;			//distance between two adjacent blocks
  vector< vector<int> > Paths;
  vector<int> pathTentative;
  vector<int> astarTent;
  vector<int> bestAstar;
  vip choice;
  float deltaBest;
  float deltavip;
  float bigL;
  float bigLTent;
  float liMin;
  int numFreeNodes;

  vector< vector<int> >::iterator it;
  vector< vector<int> >::iterator itr;
  vector<int>::iterator itc;
  vector<int>::size_type v;


  void checkBest(bool neighbour);
  float pathLength(vector<int> &pathToEvaluate);
  void loadMatrixFile(std::string acc_matrix_path);
  float dist(graphNode &a, graphNode &b);
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
