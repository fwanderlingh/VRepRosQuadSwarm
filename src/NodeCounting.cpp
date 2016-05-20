//	Copyright (c) 2014, Francesco Wanderlingh. 					//
//	All rights reserved.										//
//	License: BSD (http://opensource.org/licenses/BSD-3-Clause)	//

/*
 * NodeCounting.cpp
 *
 *  Created on: May 9, 2014
 *      Author: francescow
 */

#include "NodeCounting.h"
#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <cmath>        /* sqrt, pow */
#include <limits>       /* numeric_limits */
#include <numeric>      /* multiply, accumulate */
#include <stdlib.h>     /* srand, rand */
#include <cassert>
#include <sstream>
#include <iterator>
#include <termColors.h>
#include <unistd.h>
#include <Utils.h>
//#define STARTNODE 5

using std::cout;
using std::endl;

#define DEBUG_PRINT


NodeCounting::NodeCounting() : STARTNODE(0),gridSizeX(0), gridSizeY(0), numFreeNodes(0), minVisit(1),
    currentNode(STARTNODE), unvisitedCount(std::numeric_limits<int>::max())
{
  srand(time(NULL) xor getpid()<<16); // xor getpid()<<16);
  /// XXX Read! XXX
  // THE DEFAULT CONSTRUCTOR IS ONLY USED TO DECLARE CLASS INSTANCES AS
  // GLOBAL. WITHOUT RUNNING one of the "init_*()" functions AFTER, THE CLASS CANNOT WORK.

}


NodeCounting::NodeCounting(std::ifstream & INFILE) : unvisitedCount(std::numeric_limits<int>::max())
{
  srand(time(NULL) xor getpid()<<16); // xor getpid()<<16);
  createGraph(INFILE);
}

NodeCounting::~NodeCounting()
{

  // TODO Auto-generated destructor stub
}



void NodeCounting::loadMatrixFile(std::ifstream &access_mat){
  /// Loading the access_vec and defining grid sizes

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


void NodeCounting::createGraph(std::ifstream & INFILE){

  loadMatrixFile(INFILE);       /// THe access_vec and defining grid sizes

  //cout << "Matrix size is: " << gridSizeX << "x" << gridSizeY << endl;

  graphNodes.resize(gridSizeX*gridSizeY);
  unvisited.resize(gridSizeX*gridSizeY, 0);

  // Graph initialisation - to every node is assigned a position in a Row-Major order
  for(int i=0; i<gridSizeX; i++){
    for(int j=0; j<gridSizeY; j++){
      graphNodes.at((i*gridSizeY) + j).setPos((double)i, (double)j);  ///Position is multiplied by 2 since the access map is sub-sampled
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


void NodeCounting::createEdgeMat(){

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



void NodeCounting::loadGraphFile(std::ifstream &graph_mat){

  std::string line;
  while ( getline( graph_mat, line ) ) {
    std::istringstream is( line );
    graph.push_back(
        std::vector<int>( std::istream_iterator<int>(is),
                          std::istream_iterator<int>() ) );
  }

#ifdef DEBUG_PRINT
  cout << "\nGraph:\n";
  for(int i=0;i<graph.size();i++){
    for(int j=0; j<graph.at(1).size();j++){
      printf("%d  ",graph[i][j]);;
    }
    cout << endl;
  }
#endif

  numFreeNodes = static_cast<int>(graph.size());
  unvisited.resize(numFreeNodes, 1);
  unvisitedCount = numFreeNodes;

  graphNodes.resize(numFreeNodes);
}



void NodeCounting::loadPosVecFile(std::ifstream &Pos_vec){

  std::string line;
  std::vector< std::vector<int> > positionVec;

  while ( getline( Pos_vec, line ) ) {
    std::istringstream is( line );
    positionVec.push_back(
        std::vector<int>( std::istream_iterator<int>(is),
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


void NodeCounting::init_acc(std::ifstream & access_mat, int startingNode, int minVis){
  /** If input argument of init is only 1 then the input file is
   * the Occupancy Grid (access_mat).
   */
  minVisit = minVis;
  createGraph(access_mat);
  currentNode = startingNode;
  finalPath.push_back(currentNode);
}


void NodeCounting::init_graph_pos(std::ifstream &graph_mat, std::ifstream &Pos_vec,
                                      int startingNode, int minVis){
  /** In this case we don't have an occupancy grid but already a matrix
   * representing the graph so we need to know the position of the vertices,
   * information contained in Pos_Vec.
   */

  minVisit = minVis;
  loadGraphFile(graph_mat);
  loadPosVecFile(Pos_vec);
  finalPath.push_back(currentNode);
}



void NodeCounting::incrCount(int nodeIndex, bool isNodeVisited){

  graphNodes.at(nodeIndex).nodeCount++;

  if(isNodeVisited == 0){
    unvisited.at(nodeIndex) = 0;
    // The sum of all elements of unvisited is performed so that when sum is zero we
    // know we have finished. Check is performed in "findNext()"
    unvisitedCount = std::accumulate(unvisited.begin(),unvisited.end(), 0);
  }


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


/****    MAIN METHOD    ****/
void NodeCounting::findNext(){

#ifdef DEBUG_PRINT
 cout << "\n---\n";
  for(int i=0; i<graphNodes.size(); ++i){
    if(i%gridSizeY == 0) cout << endl;
    cout << graphNodes.at(i).nodeCount << " ";
  }
  cout << "\n\n";
  cout << "I'm on node " << currentNode << " - Node Count=" << graphNodes[currentNode].nodeCount << endl;
#endif

  if( !isCompleted() ){

    int bestCount = std::numeric_limits<int>::max();
    std::vector<int> best_vec;

    for(int j=0; j < graph.size(); j++){
      if(graph[currentNode][j] == 1){
        int tentIndex = j;
        int tentCount = graphNodes.at(tentIndex).nodeCount;

#ifdef DEBUG_PRINT
        cout << "Count of " << tentIndex << " is " << tentCount << endl;
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

    /* OLD STUFF COMMENT:
     * Look for adjacent nodes and find the one with the smallest number of visits
    / Before being able to do the adjacency check we have to recover the (i,j) index
    / values encoded in the graph 1D array as "i*gridSizeY + j" (row-major order) */

    /*
    int current_i = currentNode/gridSizeY;
    int current_j = currentNode%gridSizeY;

    for(int i_shift=-1; i_shift<=1; ++i_shift){
      for(int j_shift=-1; j_shift<=1; ++j_shift){

        int tent_i = current_i + i_shift;
        int tent_j = current_j + j_shift;

        if( (tent_i>=0) && (tent_i<gridSizeX) && (tent_j>=0) && (tent_j<gridSizeY) && ///RANGE CHECK
            (i_shift!=0 || j_shift!=0)  ///<--- don't check same node of current
            && (i_shift*j_shift == 0) ///<--- don't check oblique nodes
            )
        {
          int tentIndex = tent_i*gridSizeY + tent_j;

          if( (graphNodes.at(tentIndex).occupied == 0) ){   ///OCCUPANCY CHECK
          //cout << "Checking node n." << tentIndex << endl;

            if( graphNodes.at(tentIndex).nodeCount <= bestCount ){

              ///Look if there are other nodes with same count
              if( graphNodes.at(tentIndex).nodeCount == bestCount ){
                best_vec.push_back(tentIndex);
              }else{
                best_vec.clear();
                best_vec.push_back(tentIndex);
              }

              bestCount = graphNodes.at(tentIndex).nodeCount;

            }//End checkBest
          }//End occupancy check
        }//End range check
      }//End j_shift for-loop (y scan)
    } //End i_shift for-loop (x scan)
     */


    /// Now if there is more than one element in the vector choose one randomly.
    /// If size()==1 the modulus function always returns 0 (the first element)
    assert(best_vec.size() != 0);
    int randIndex = rand()%best_vec.size();
    currentNode = best_vec.at(randIndex);

#ifdef DEBUG_PRINT
    cout << "Chosen Node: " << currentNode << endl;
#endif

    finalPath.push_back(currentNode);

  }
}


float NodeCounting::getCurrentCoord(char coordinate){
  switch(coordinate){
    case 'x':
      return graphNodes.at(currentNode).posx;
      break;
    case 'y':
      return graphNodes.at(currentNode).posy;
      break;
      /// 'z' for now is omitted on purpose, since the height depends on the robot
      /// (check quadcopterRosCtrl.cpp or quadNodeCount.cpp for further details)
  }
}


bool NodeCounting::getCurrentType(){

  if(graphNodes.at(currentNode).nodeCount == 0)
    return 0;
  else
    return 1;
}


bool NodeCounting::isCompleted(){
  //std::cout << "size?" << graphNodes.size() << std::endl;
  //std::cout << "unvisitedcount?" << unvisitedCount << std::endl;
  int count_reached = 0;
  for(int i=0; i<graphNodes.size(); ++i){
    //if(i%gridSizeY == 0) cout << endl;
    //cout << graphNodes.at(i).nodeCount << " ";
    if( graphNodes.at(i).nodeCount >= minVisit ){
      ++count_reached;
    }
  }
  //cout << endl;
  if(count_reached == numFreeNodes){
    return 1;
  }else return 0;
  
  
 // if(unvisitedCount == 0)
 //   return true;
 // else
 //   return false;
   
}
