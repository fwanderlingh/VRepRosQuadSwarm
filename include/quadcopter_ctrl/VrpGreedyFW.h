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
  int numAgents;
  vector<graphNode> graphNodes;
  const static int Inf = INT_MAX/2-1;
  vector< vector<int> > graph;
  vector< vector<int> > distanceMat;
  vector<int> fwTent;
  vector<int> access_vec;
  vector<int> unvisitedNodes;
  vector<int> path;
  vector< vector<int> > Paths;
  vector<int> pathTentative;
  vip choice;
  float deltaBest;
  float deltavip;
  float bigL;
  float liMin;

  int numFreeNodes;

  float minDist; //distance between two adjacent blocks

  vector< vector<int> >::iterator it;
  vector< vector<int> >::iterator itr;
  vector<int>::iterator itc;
  vector<int>::size_type v;

  float pathLength(vector<int> &pathToEvaluate);
  void loadMatrixFile(std::string acc_matrix_path);
  void createGraph();
  void createCycles();
  float dist(graphNode &a, graphNode &b);
  void checkBest(bool isNeighbour);



public:
  VrpGreedy();
  VrpGreedy(std::string acc_matrix_path);
  VrpGreedy(std::string acc_matrix_path, int agents);
  virtual ~VrpGreedy();
  void init();
  void solve();
  void copyPathsTo(vector< vector<int> > &);
  void copyGraphTo(vector< graphNode > &);
  void performanceIndexes();

};





#endif /* VRPGREEDY_H_ */
