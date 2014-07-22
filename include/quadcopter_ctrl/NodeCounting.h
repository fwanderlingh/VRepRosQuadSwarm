/*
 * NodeCounting.h
 *
 *  Created on: May 9, 2014
 *      Author: francescow
 */

#ifndef NODECOUNTING_H_
#define NODECOUNTING_H_

#include <vector>
#include <fstream>
#include "graphStructs.h"

using std::vector;

class NodeCounting
{
  int STARTNODE;
  int gridSizeX;
  int gridSizeY;
  int currentNode;
  vector<graphNode> graphNodes;
  vector< vector <int> > graph;
  vector<int> access_vec;       //It could be "bool" but I left "int" for future map developments
  vector<int> unvisited;
  int unvisitedCount;

  int numFreeNodes;
  vector<int> finalPath;

  void createEdgeMat();
  void loadMatrixFile(std::ifstream &access_mat);
  void createGraph(std::ifstream & INFILE);
  void loadGraphFile(std::ifstream &graph_mat);
  void loadPosVecFile(std::ifstream &Pos_vec);

public:
  NodeCounting();
  NodeCounting(std::ifstream & INFILE);
  virtual ~NodeCounting();
  void init_acc(std::ifstream & INFILE);
  void init_graph_pos(std::ifstream &graph_mat, std::ifstream &Pos_vec);

  void incrCount(int nodeIndex, bool nodeType);
  void findNext();
  float getCurrentCoord(char coordinate);
  bool getCurrentType();
  bool isCompleted();


  const vector<int>& getFinalPath() const
  {
    return finalPath;
  }

  int getCurrentIndex() const
  {
    return currentNode;
  }

  int getNumFreeNodes() const
  {
    return numFreeNodes;
  }
};

#endif /* NODECOUNTING_H_ */
