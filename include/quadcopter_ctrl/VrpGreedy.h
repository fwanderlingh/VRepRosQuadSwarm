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


using std::vector;


class VrpGreedy
{

  int gridSizeX;
  int gridSizeY;
  int numAgents;
  vector<graphNode> graphNodes;
  vector<int> unvisitedNodes;
  vector<int> access_vec;
  vector<int> path;
  vector< vector<int> > Paths;
  vector<int> pathTentative;
  vip choice;
  float deltaBest;
  float deltavip;
  float bigL;
  float bigLTent;
  float liMin;

  float minDist; //distance between two adjacent blocks

  vector< vector<int> >::iterator itr;
  vector<int>::iterator itc;
  vector<int>::size_type v;

  float pathLength(vector<graphNode> graph, vector<int> &pathToEvaluate);
  void loadMatrixFile(std::string acc_matrix_path);
  void createCycles();
  float dist(graphNode &a, graphNode &b);


public:
  VrpGreedy();
  VrpGreedy(std::string acc_matrix_path);
  VrpGreedy(std::string acc_matrix_path, int agents);
  virtual ~VrpGreedy();
  void init();
  void solve();
  void copyPathsTo(vector< vector<int> > &);
  void copyGraphTo(vector< graphNode > &);

};





#endif /* VRPGREEDY_H_ */
