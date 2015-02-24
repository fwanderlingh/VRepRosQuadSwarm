/*
 * NodeCountingLoss.h
 *
 *  Created on: May 9, 2014
 *      Author: francescow
 */

#ifndef NODECOUNTINGLOSS_H_
#define NODECOUNTINGLOSS_H_

#include <vector>
#include <fstream>
#include "graphStructs.h"

using std::vector;

class NodeCountingLoss
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
  int minVisit;

  int numFreeNodes;
  vector<int> finalPath;

  void createEdgeMat();
  void loadMatrixFile(std::ifstream &access_mat);
  void createGraph(std::ifstream & INFILE);
  void loadGraphFile(std::ifstream &graph_mat);
  void loadPosVecFile(std::ifstream &Pos_vec);

public:
  NodeCountingLoss();
  NodeCountingLoss(std::ifstream & INFILE);
  virtual ~NodeCountingLoss();
  void init_acc(std::ifstream & INFILE, int startingNode, int minVis);
  void init_graph_pos(std::ifstream &graph_mat, std::ifstream &Pos_vec, int startingNode, int minVis);

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

  const vector<graphNode>& getGraphNodes() const
  {
    return graphNodes;
  }

  int getGridSizeX() const
  {
    return gridSizeX;
  }

  int getGridSizeY() const
  {
    return gridSizeY;
  }
};

#endif /* NODECOUNTINGLOSS_H_ */
