/*
 * LRTAstar.h
 *
 *  Created on: May 9, 2014
 *      Author: francescow
 */

#ifndef LRTASTAR_H_
#define LRTASTAR_H_

#include <vector>
#include <fstream>
#include "graphStructs.h"

using std::vector;

class LRTAstar
{
  int gridSizeX;
  int gridSizeY;
  int currentNode;
  vector<graphNode> graphNodes;
  vector<int> access_vec;       //It could be "bool" but I left "int" for future map developments
  vector<int> unvisited;
  int unvisitedCount;
  int prevNode;

  void loadMatrixFile(std::ifstream &access_mat);

public:
  LRTAstar();
  LRTAstar(std::ifstream & INFILE);
  virtual ~LRTAstar();
  void initGraph(std::ifstream & INFILE);
  void incrCount(int prevIndex, int currIndex, bool currType);
  void findNext();
  float getCurrentCoord(char coordinate);
  int getCurrentIndex();
  int getPrevIndex();
  bool getCurrentType();
  bool isCompleted();


};

#endif /* LRTASTAR_H_ */
