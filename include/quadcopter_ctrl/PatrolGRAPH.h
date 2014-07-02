/*
 * PatrolGRAPH.h
 *
 *  Created on: Jun 21, 2014
 *      Author: francescow
 */

#ifndef PATROLGRAPH_H_
#define PATROLGRAPH_H_

#include <vector>
#include <fstream>
#include "graphStructs.h"
#include <boost/array.hpp>

using std::vector;

class PatrolGRAPH
{
  int gridSizeX;
  int gridSizeY;
  vector<int> access_vec;       // It could be "bool" but I left "int" for future map developments
  vector<int> unvisited;
  int unvisitedCount;
  vector<graphNode> graphNodes;
  vector< vector <int> > graph;
  vector< vector <int> > edgeCountMat;
  vector< vector <double> > PTM; /// PTM=Probability Transition Matrix
  int currentNode;
  boost::array<int, 2> chosenEdge;


  int numFreeNodes;
  vector<int> finalPath;

  void createEdgeMat();
  void computeProbabilityMat();
  void loadMatrixFile(std::ifstream &access_mat);


public:
  PatrolGRAPH();
  virtual ~PatrolGRAPH();
  void initGraph(std::ifstream & INFILE);
  void incrCount(int currIndex, bool currType, boost::array<int, 2> edge);
  void findNext();
  bool getCurrentType();
  bool isCompleted();
  float getCurrentCoord(char coordinate);

  int getCurrentNode() const
  {
    return currentNode;
  }

  boost::array<int, 2> getChosenEdge() const
  {
    return chosenEdge;
  }

  const vector<int>& getFinalPath() const
  {
    return finalPath;
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

#endif /* PATROLGRAPH_H_ */
