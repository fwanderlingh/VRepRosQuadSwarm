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
  int STARTNODE;
  int gridSizeX;
  int gridSizeY;
  vector<int> access_vec;       // It could be "bool" but I left "int" for future map developments
  vector<int> unvisited;
  int unvisitedCount;
  vector<graphNode> graphNodes;
  vector< vector <int> > graph;
  vector< vector <int> > edgeCountMat;
  vector< vector <double> > PTM; /// PTM=Probability Transition Matrix
  bool optimized;
  int currentNode;
  boost::array<int, 2> chosenEdge;


  int numFreeNodes;
  vector<int> finalPath;

  void createEdgeMat();
  void computeProbabilityMat();
  void loadPosVecFile(std::ifstream &Pos_vec);
  void initGraph(std::ifstream & INFILE);
  void loadMatrixFile(std::ifstream &access_mat);
  void loadPTMFile(std::ifstream &PTM_mat);
  void loadGraphFile(std::ifstream &graph_mat);

public:
  PatrolGRAPH();
  virtual ~PatrolGRAPH();
  void init_acc(std::ifstream & graph_mat);
  void init_graph_pos(std::ifstream &graph_mat, std::ifstream &Pos_vec);
  void init_acc_ptm(std::ifstream & access_mat, std::ifstream & PTM_mat);
  void init_graph_pos_ptm(std::ifstream &graph_mat, std::ifstream &Pos_vec, std::ifstream &PTM_mat);

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
