//	Copyright (c) 2014, Francesco Wanderlingh. 					//
//	All rights reserved.										//
//	License: BSD (http://opensource.org/licenses/BSD-3-Clause)	//

/*
 * GraphAlgs.cpp
 *
 *  Created on: Jul 1, 2014
 *      Author: francescow
 */

#include "GraphAlgs.h"
#include <cstdlib>
#include <limits>
#include <fstream>
#include <algorithm>
#include <cassert>
#include <algorithm>
#include <cmath>


GraphAlgs::GraphAlgs()
{
  // TODO Auto-generated constructor stub

}

GraphAlgs::GraphAlgs(std::string matrix_path)
{

  loadGraphFromFile(matrix_path);
  init();

}


GraphAlgs::~GraphAlgs()
{
  // TODO Auto-generated destructor stub
}


void GraphAlgs::loadMatrixFile(std::string matrix_path, vector<int> &container){
  /**
   * Here gridSizeX and gridSizeY are deducted from the size of the input matrix file
   */

  std::ifstream access_mat;
  access_mat.open( matrix_path.c_str() );

  if( access_mat.is_open() ) {
    int val;
    num_nl = 0;
    while( access_mat >> val ){
      if(access_mat.peek() == '\n') num_nl++;
      container.push_back( val );
    }
    access_mat.close();

  }
  else{ cout << "Error reading file!" << endl; }
}


void GraphAlgs::init(){

  gridSizeX = num_nl;
  gridSizeY = access_vec.size()/num_nl;
  //cout << "Matrix size is: " << gridSizeX << "x" << gridSizeY << endl;
  //printf("%sAccess Map:%s",TC_RED, TC_NONE);
  //printRowMajorMat(access_vec);

  graphNodes.resize(gridSizeX*gridSizeY);
  createGraph();

}


void GraphAlgs::createGraph(){

  const int n = gridSizeX*gridSizeY;

  assert(n == access_vec.size());
  vector<int> _graph(n, 0);
  vector< vector<int> > __graph(n, _graph);
  edgeGraph = __graph;
  int row, col;    // Main indexes
  int row_shift, col_shift; // To move around spatial adjacents
  int nb_row, nb_col;       // Adjacent indexes


  for(int i=0; i<n; i++){
    if(access_vec[i] == 0){
      graphNodes.at(i).occupied = 0;
      row = i/gridSizeY;
      col = i%gridSizeY;
      for(row_shift=-1; row_shift<=1; row_shift++){
        for(col_shift=-1; col_shift<=1; col_shift++){
          nb_row = row + row_shift;
          nb_col = col + col_shift;
          if( (nb_row>=0) && (nb_row<gridSizeX) && (nb_col>=0) && (nb_col<gridSizeY) ///RANGE CHECK
              && (row_shift!=0 || col_shift!=0)  ///<--- don't check same node of current
              && (row_shift*col_shift == 0)  ///<--- don't allow diagonal movements
          )
            //&& ( (row_shift*row_shift xor col_shift*col_shift)==1 ) <--last 2 statements compressed in one condition
          {
            if(access_vec[nb_row*gridSizeY+nb_col] == 0){
              /// Create the edge between "i" and its "free neighbour"
              edgeGraph[i][nb_row*gridSizeY+nb_col] = 1;
              edgeGraph[nb_row*gridSizeY+nb_col][i] = 1;
              graphNodes.at(i).numEdges++;
            }
          }
        }
      }
    }
  }
}


void GraphAlgs::loadGraphFromFile(std::string matrix_path){
  loadMatrixFile(matrix_path, access_vec);
}


void GraphAlgs::loadNodeCountMapFromFile(std::string matrix_path){
  loadMatrixFile(matrix_path, nodeCountMap);
  //printf("%sCount Map:%s",TC_RED, TC_NONE);
  //printRowMajorMat(nodeCountMap);
}


void GraphAlgs::printNumOfEdges(){

  cout << "Num of Edges:\n";
  for(int j=0; j<gridSizeX*gridSizeY; j++){
    if(j%gridSizeY == 0) cout << endl;
    Utils::spaced_cout(graphNodes.at(j).numEdges);
  }
  cout << endl << endl;

}


void GraphAlgs::stdevFromExpectedVisits(){

  printNumOfEdges();
  vector<int> missingVisits(nodeCountMap);
  vector<double> normalizedCountMap(nodeCountMap.size());
  std::transform(nodeCountMap.begin(), nodeCountMap.end(), normalizedCountMap.begin(),
                 normalizedCountMap.begin(), std::plus<double>());

  std::vector<int>::iterator it = std::max_element(nodeCountMap.begin(), nodeCountMap.end());
  int max_count_nodeIndex = (int)(it - nodeCountMap.begin());
  int max_count = nodeCountMap.at(max_count_nodeIndex);
  //cout << "max_count=" << max_count << endl;
  int visitFlag = 0;
  int tempCount = max_count;

  // FIXME this method is too empirical
  // try instead to find the divisor which minimizes the difference from the expected frequency
  while(tempCount > 0){
    tempCount = tempCount - graphNodes.at(max_count_nodeIndex).numEdges;
    visitFlag++;
  }
  cout << "Visit Flag:" << visitFlag << endl;


  for(int i=0; i<nodeCountMap.size(); i++){
    missingVisits.at(i) = missingVisits.at(i) - ( 3*graphNodes.at(i).numEdges );
  }

/*  vector<double> countDeviation(normalizedCountMap.size());
  for(int i=0; i<nodeCountMap.size(); i++){
    if(graphNodes.at(i).numEdges > 0){
      countDeviation.at(i) = missingVisits.at(i)/(double)graphNodes.at(i).numEdges;
    }
  }
*/
  printRowMajorMat(nodeCountMap);
  printRowMajorMat(missingVisits);
//  printRowMajorMat(countDeviation);

}
