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
  vector<int> access_vec;       //It could be "bool" but I left "int" for future map developments
  vector<int> unvisited;
  int unvisitedCount;

  int numFreeNodes;
  vector<int> finalPath;

  void loadMatrixFile(std::ifstream &access_mat);

public:
  NodeCounting();
  NodeCounting(std::ifstream & INFILE);
  virtual ~NodeCounting();
  void initGraph(std::ifstream & INFILE);
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
