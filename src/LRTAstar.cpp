//	Copyright (c) 2014, Francesco Wanderlingh. 			//
//	All rights reserved.						//				//
//	License: BSD (http://opensource.org/licenses/BSD-3-Clause)	//

/*
 * LRTAstar.cpp
 *
 *  Created on: May 9, 2014
 *      Author: francescow
 */

#include "LRTAstar.h"
#include <iostream>
#include <cmath>        /* sqrt, pow */
#include <limits>       /* numeric_limits */
#include <numeric>      /* multiply, accumulate */
#include <cstdlib>
#include <cassert>
#include <sstream>
#include <termColors.h>
#include <iterator>
#include <Utils.h>
#include <unistd.h>

//#define DEBUG_PRINT

using std::cout;
using std::endl;
using std::vector;


LRTAstar::LRTAstar() :  STARTNODE(0), gridSizeX(0), gridSizeY(0), currentNode(STARTNODE),
    unvisitedCount(std::numeric_limits<int>::max())
{
  srand(time(NULL) xor getpid()<<16);
  // THE DEFAULT CONSTRUCTOR IS ONLY USED TO DECLARE CLASS INSTANCES AS
  // GLOBAL. WITHOUT RUNNING "initGraph()" AFTER, THE CLASS CANNOT WORK.


}


LRTAstar::LRTAstar(std::ifstream & INFILE) : STARTNODE(0), currentNode(STARTNODE),
    unvisitedCount(std::numeric_limits<int>::max())
{
  srand(time(NULL) xor getpid()<<16); // xor getpid()<<16);
  createGraph(INFILE);
}

LRTAstar::~LRTAstar()
{
  // TODO Auto-generated destructor stub
}


void LRTAstar::loadMatrixFile(std::ifstream &access_mat){

  if( access_mat.is_open() ) {
    int val;
    int num_nl = 0;
    while( access_mat >> val ){
      if(access_mat.peek() == '\n') num_nl++;
      access_vec.push_back( val );
    }

    access_mat.close();

    gridSizeX = num_nl;
    gridSizeY = access_vec.size()/num_nl;

  }
  else{ cout << "Error reading file!" << endl; }

}


void LRTAstar::loadGraphFile(std::ifstream &graph_mat){

  std::string line;
  while ( getline( graph_mat, line ) ) {
    std::istringstream is( line );
    graph.push_back(
        vector<int>( std::istream_iterator<int>(is),
                          std::istream_iterator<int>() ) );
  }
  /*
  cout << "\nGraph:\n";
  for(int i=0;i<graph.size();i++){
    for(int j=0; j<graph.at(1).size();j++){
      printf("%d  ",graph[i][j]);;
    }
    cout << endl;
  }
   */
  numFreeNodes = static_cast<int>(graph.size());

  AStarCount.resize(numFreeNodes, 0);

  unvisited.resize(numFreeNodes, 1);
  unvisitedCount = numFreeNodes;

  graphNodes.resize(numFreeNodes);
}


void LRTAstar::loadPosVecFile(std::ifstream &Pos_vec){

  std::string line;
  vector< vector<int> > positionVec;

  while ( getline( Pos_vec, line ) ) {
    std::istringstream is( line );
    positionVec.push_back(
        vector<int>( std::istream_iterator<int>(is),
                          std::istream_iterator<int>() ) );
  }
  /*
  cout << "\nPos Vec:\n";
  for(int i=0;i<positionVec.size();i++){
    for(int j=0; j<positionVec.at(1).size();j++){
      printf("%d  ",positionVec[i][j]);;
    }
    cout << endl;
  }
   */

  for(int i=0; i < numFreeNodes; i++){
    graphNodes.at(i).setPos(static_cast<double>(positionVec[0][i]),
                            static_cast<double>(positionVec[1][i]) );
  }

  assert(graphNodes.size() == graph.size());

}


void LRTAstar::createGraph(std::ifstream & INFILE){

  loadMatrixFile(INFILE);       /// Filling the "access_vec" and defining grid sizes

  //cout << "Matrix size is: " << gridSizeX << "x" << gridSizeY << endl;

  AStarCount.resize(gridSizeX*gridSizeY, 0);

  graphNodes.resize(gridSizeX*gridSizeY);
  unvisited.resize(gridSizeX*gridSizeY, 0);

  // Graph initialisation - to every node is assigned a position in a Row-Major order
  for(int i=0; i<gridSizeX; i++){
    for(int j=0; j<gridSizeY; j++){
      graphNodes.at((i*gridSizeY) + j).setPos((float)i, (float)j);  ///Position is multiplied by 2 since the access map is sub-sampled
      graphNodes.at((i*gridSizeY) + j).occupied = access_vec.at((i*gridSizeY) + j);

      if(access_vec.at((i*gridSizeY) + j) == 0)  unvisited.at((i*gridSizeY) + j) = 1;
      // cout << (int)graphNodes.at((i*gridSizeY) + j).occupied << " ";
    }
    //cout << endl;
  }
  unvisitedCount = std::accumulate(unvisited.begin(),unvisited.end(), 0);
  cout << "unvisitedCount: " << unvisitedCount << endl;

  numFreeNodes = unvisitedCount;

  createEdgeMat();

}


void LRTAstar::init_acc(std::ifstream & access_mat, int startingNode, int minVis){
  /** If input argument of init is only 1 then we assume we have no
   * optimized Probability Transition Matrix and the input file is
   * the Occupancy Grid (access_mat).
   */
  minVisit = minVis;
  createGraph(access_mat);
  currentNode = nextNode = startingNode;
  finalPath.push_back(currentNode);
}


void LRTAstar::init_graph_pos(std::ifstream &graph_mat, std::ifstream &Pos_vec, int startingNode, int minVis){
  /** In this case we don't have an occupancy grid but already a matrix
   * representing the graph so we need to know the position of the vertices,
   * information contained in Pos_Vec. No optimised PTM.
   */
  minVisit = minVis;
  loadGraphFile(graph_mat);
  loadPosVecFile(Pos_vec);
  currentNode = nextNode =  startingNode;
  finalPath.push_back(currentNode);
}


void LRTAstar::incrCount(int currIndex, int nextIndex, bool isNextVisited){

  AStarCount.at(currIndex) = AStarCount.at(nextIndex) + 1;

  graphNodes.at(nextIndex).nodeCount++;
  //graphNodes.at(currIndex).nodeCount++;

  if(isNextVisited == 0){
    unvisited.at(nextIndex) = 0;
    // The sum of all elements of unvisited is performed so that when sum is zero we
    // know we have finished. Check is performed in "findNext()"
    unvisitedCount = std::accumulate(unvisited.begin(),unvisited.end(), 0);
  }


  /*
   * PRINT MAP FOR DEBUGGING PURPOSES
   */
  /*
  for(int i=0; i<gridSizeX; i++){
    for(int j=0; j<gridSizeY; j++){
      cout << (int)graphNodes.at((i*gridSizeY) + j).nodeCount << " ";
    }
    cout << endl;
  }

  cout << "====================================" << endl;
   */
}

void LRTAstar::createEdgeMat(){

  const int n = gridSizeX*gridSizeY;

  assert(n == access_vec.size());
  /// Here we create a zero matrix of the size of the graph
  vector< vector<int> > _graph(n, vector<int> (n, 0));
  graph = _graph;

  int row, col;    // Main indexes
  int row_shift, col_shift; // To move around spatial adjacents
  int nb_row, nb_col;       // Adjacent indexes

#ifdef DEBUG_PRINT
  printf("%sOccupancy Map:%s",TC_RED, TC_NONE);
  for(int j=0; j<n; j++){
    if(j%gridSizeY == 0) cout << endl;
    if( access_vec[j] == 1 ){
      printf("%s",TC_RED);
      Utils::spaced_cout(j);
      printf("%s", TC_NONE);
    }
    else Utils::spaced_cout(j);
  }
  cout << endl << endl;
#endif

  for(int i=0; i<n; i++){
    if(access_vec[i] == 0){
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
              graph[i][nb_row*gridSizeY+nb_col] = 1;
              graph[nb_row*gridSizeY+nb_col][i] = 1;
              graphNodes.at(i).numEdges++;
            }
          }
        }
      }
    }
  }

}


/// *** MAIN METHOD *** ///
void LRTAstar::findNext(){




  if( !isCompleted() ){
    /// Look for adjacent nodes and find the one with the smallest number of visits
    /// Before being able to do the adjacency check we have to recover the (i,j) index
    /// values encoded in the graph 1D array as "i*gridSizeY + j" (row-major order)

    currentNode = nextNode;

#ifdef DEBUG_PRINT
    cout << "\n---\n";
    for(int i=0; i<graphNodes.size(); ++i){
      if(i%gridSizeY == 0) cout << endl;
      cout << graphNodes.at(i).nodeCount << " ";
    }
    cout << "\n\n";
    cout << "I'm on node " << currentNode << " - Node Count=" << graphNodes[currentNode].nodeCount << endl;
#endif

    int bestCount = std::numeric_limits<int>::max();
    vector<int> best_vec;

    //cout << "graph.size(): " << graph.size() << endl;
    for(int j=0; j < graph.size(); j++){
      if(graph[currentNode][j] == 1){
        int tentIndex = j;
        int tentCount = AStarCount.at(tentIndex); // graphNodes.at(tentIndex).nodeCount;

#ifdef DEBUG_PRINT
        cout << "LRTA count of " << tentIndex << " is " << tentCount << endl;
#endif

        if( tentCount <= bestCount ){

          ///Look if there are other nodes with same count
          if( tentCount == bestCount ){
            best_vec.push_back(tentIndex);
          }else{
            best_vec.clear();
            best_vec.push_back(tentIndex);
          }

          bestCount = tentCount;

        }//End checkBest
      }
    }

    /// Now if there is more than one element in the vector choose one randomly.
    /// If size()==1 the modulus function always returns 0 (the first element)
    assert(best_vec.size() != 0);
    int randIndex = rand()%best_vec.size();
    nextNode = best_vec.at(randIndex);

#ifdef DEBUG_PRINT
    cout << "Chosen Node: " << nextNode << endl;
#endif

    finalPath.push_back(nextNode);
  }

}


float LRTAstar::getCurrentCoord(char coordinate){

  switch(coordinate){
    case 'x':
      return graphNodes.at(currentNode).posx;
      break;
    case 'y':
      return graphNodes.at(currentNode).posy;
      break;
      /// 'z' for now is omitted on purpose, since the height depends on the robot
      /// (check quadcopterRosCtrl.cpp or quadLRTA.cpp for further details)
  }

}


bool LRTAstar::getCurrType(){

  if(graphNodes.at(currentNode).nodeCount == 0)
    return 0;
  else
    return 1;
}


bool LRTAstar::getNextType(){

  if(graphNodes.at(nextNode).nodeCount == 0)
    return 0;
  else
    return 1;
}


bool LRTAstar::isCompleted(){

  int count_reached = 0;
  for(int i=0; i<graphNodes.size(); ++i){
    //if(i%gridSizeY == 0) cout << endl;
    //cout << graphNodes.at(i).nodeCount << " ";
    if( graphNodes.at(i).nodeCount >= minVisit ){
      ++count_reached;
    }
  }
  //cout << endl;
  if(count_reached == graphNodes.size()){
    return 1;
  }else return 0;

  /*
  if(unvisitedCount==0)
    return true;
  else
    return false;
   */
}
