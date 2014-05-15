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
#include "graphNodeStruct.h"

using std::vector;

class NodeCounting
{
  int gridSizeX;
  int gridSizeY;
  int currentNode;
  vector<graphNode> graphNodes;
  vector<int> access_vec;       //It could be "bool" but I left "int" for future map developments
  vector<int> unvisited;
  int unvisitedCount;

  void loadMatrixFile(std::ifstream &access_mat);

public:
  NodeCounting();
  NodeCounting(std::ifstream & INFILE);
  virtual ~NodeCounting();
  void initGraph(std::ifstream & INFILE);
  void incrCount(int nodeIndex, bool nodeType);
  void findNext();
  float getCurrentCoord(char coordinate);
  int getCurrentIndex();
  bool getCurrentType();
  bool isCompleted();


};

#endif /* NODECOUNTING_H_ */
