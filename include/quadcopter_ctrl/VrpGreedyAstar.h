/*
 * VrpGreedyAstar.h
 *
 *  Created on: Apr 29, 2014
 *      Author: francescow
 */

#ifndef VrpGreedyAstarASTAR_H_
#define VrpGreedyAstarASTAR_H_

#include <vector>
#include <fstream>
#include <cstring>
#include "graphStructs.h"
#include "astar.h"
#include "2darray.h"

using std::vector;


class VrpGreedyAstar
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

  alg::Array2D<unsigned char> astar_grid;  /* Grid contains the same data as access_vec but the A* algorithm
                                         * only accepts Array2D<unsigned char> type as input. */

  float pathLength(vector<graphNode> graph, vector<int> &pathToEvaluate);
  void loadMatrixFile(std::string acc_matrix_path);
  void createCycles();
  float dist(graphNode &a, graphNode &b);
  void init();


public:
  VrpGreedyAstar();
  VrpGreedyAstar(std::string acc_matrix_path);
  VrpGreedyAstar(std::string acc_matrix_path, int agents);
  virtual ~VrpGreedyAstar();
  void solve();
  void copyPathsTo(vector< vector<int> > &);
  void copyGraphTo(vector< graphNode > &);

};





#endif /* VrpGreedyAstarASTAR_H_ */
