/*
 * VrpGreedy.h
 *
 *  Created on: Apr 29, 2014
 *      Author: francescow
 */

#ifndef VRPGREEDY_H_
#define VRPGREEDY_H_

#include <vector>
#include <fstream>
#include <cstring>
#include "graphStructs.h"
#include <climits>


using std::vector;


class VrpGreedy
{

  int gridSizeX;
  int gridSizeY;
  int numRobots;
  vector<graphNode> graphNodes;
  const static int Inf = INT_MAX/2-1;
  vector< vector<int> > graph;
  vector< vector<int> > distanceMat;
  vector<int> fwTent;
  vector<int> access_vec;
  vector<int> unvisitedNodes;
  vector< vector<int> > Paths;
  vector<int> pathTentative;
  vip choice;
  double deltaBest;
  double deltavip;
  double bigL;
  double liMin;

  int numFreeNodes;

  double minDist; //distance between two adjacent blocks

  vector< vector<int> >::iterator it;
  vector< vector<int> >::iterator itr;
  vector<int>::iterator itc;
  vector<int>::size_type v;

  double pathLength(vector<int> &pathToEvaluate);
  void loadMatrixFile(std::string acc_matrix_path);
  void createGraph();
  float dist(graphNode &a, graphNode &b);
  void checkBest(bool isNeighbour);



public:
  VrpGreedy();
  VrpGreedy(std::string acc_matrix_path);
  VrpGreedy(std::string acc_matrix_path, int agents);
  virtual ~VrpGreedy();
  void init();
  void solve();

  int getNumRobots() const
  {
    return numRobots;
  }

  const vector<vector<int> >& getPaths() const
  {
    return Paths;
  }

  const vector<graphNode>& getGraphNodes() const
  {
    return graphNodes;
  }

  int getNumFreeNodes() const
  {
    return numFreeNodes;
  }
};





#endif /* VRPGREEDY_H_ */
