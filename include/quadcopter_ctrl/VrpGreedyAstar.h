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
#include "graphNodeStruct.h"


using std::vector;

struct vip{
  int v;                                // Vertex (node) index (of the unvisited list)
  vector< vector<int> >::iterator i;    // Index of Path (#robots)
  vector<int>::iterator p;              // Position index inside the path

  void set_vip( int nodeIndex,
                 vector< vector<int> >::iterator pathIndex,
                 vector<int>::iterator position ){
    v = nodeIndex;
    i = pathIndex;
    p = position;
  }
};


/// CLASS ///

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

  float pathLength(vector<graphNode> graph, vector<int> &pathToEvaluate);
  void loadMatrixFile(std::ifstream &access_mat);
  void createCycles();
  float dist(graphNode &a, graphNode &b);


public:
  VrpGreedyAstar();
  VrpGreedyAstar(std::ifstream &access_mat);
  VrpGreedyAstar(std::ifstream &access_mat, int agents);
  virtual ~VrpGreedyAstar();
  void init();
  void solve();
  void copyPathsTo(vector< vector<int> > &);
  void copyGraphTo(vector< graphNode > &);

};





#endif /* VrpGreedyAstarASTAR_H_ */
