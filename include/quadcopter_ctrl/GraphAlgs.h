/*
 * GraphAlgs.h
 *
 *  Created on: Jul 1, 2014
 *      Author: francescow
 */

#ifndef GRAPHALGS_H_
#define GRAPHALGS_H_

#include <iostream>
#include <vector>
#include "graphStructs.h"
#include "Utils.h"
#include <termColors.h>
#include <cstdio>


using std::cout;
using std::endl;
using std::vector;

class GraphAlgs
{
  int gridSizeX;
  int gridSizeY;
  int numRobots;
  vector< vector<int> > edgeGraph;
  vector<graphNode> graphNodes;
  vector<int> access_vec;
  vector<int> nodeCountMap;
  int num_nl;

  void init();

public:
  GraphAlgs();
  GraphAlgs(std::string matrix_path);
  virtual ~GraphAlgs();

  void loadMatrixFile(std::string matrix_path, vector<int> &container);
  void loadGraphFromFile(std::string matrix_path);
  void loadNodeCountMapFromFile(std::string matrix_path);
  void createGraph();
  void printNumOfEdges();
  void stdevFromExpectedVisits();

  template<typename T>
  void printRowMajorMat(T matrix);


};


template<typename T>
void GraphAlgs::printRowMajorMat(T matrix){

  for(int j=0; j<gridSizeX*gridSizeY; j++){
    if(j%gridSizeY == 0) cout << endl;
    if(graphNodes.at(j).occupied == 1){
      printf("%s", TC_RED);
      Utils::spaced_cout(matrix[j]);
      printf("%s", TC_NONE);

    }else{
      Utils::spaced_cout(matrix[j]);
    }
  }
  cout << endl << endl;

}

#endif /* GRAPHALGS_H_ */
