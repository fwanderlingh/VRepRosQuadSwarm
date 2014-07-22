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
#include <iterator>

//#define STARTNODE 5

using std::cout;
using std::endl;


LRTAstar::LRTAstar() :  STARTNODE(0), gridSizeX(0), gridSizeY(0), currentNode(STARTNODE),
                          unvisitedCount(std::numeric_limits<int>::max())
{

  // THE DEFAULT CONSTRUCTOR IS ONLY USED TO DECLARE CLASS INSTANCES AS
  // GLOBAL. WITHOUT RUNNING "initGraph()" AFTER, THE CLASS CANNOT WORK.

}


LRTAstar::LRTAstar(std::ifstream & INFILE) : STARTNODE(0), currentNode(STARTNODE),
    unvisitedCount(std::numeric_limits<int>::max())
{
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
        std::vector<int>( std::istream_iterator<int>(is),
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

  unvisited.resize(numFreeNodes, 1);
  unvisitedCount = numFreeNodes;

  graphNodes.resize(numFreeNodes);
}


void LRTAstar::loadPosVecFile(std::ifstream &Pos_vec){

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


void LRTAstar::createGraph(std::ifstream & INFILE){

  srand(time(NULL) xor getpid()<<16); // xor getpid()<<16);
  loadMatrixFile(INFILE);       /// Filling the "access_vec" and defining grid sizes

  //cout << "Matrix size is: " << gridSizeX << "x" << gridSizeY << endl;

  graphNodes.resize(gridSizeX*gridSizeY);
  unvisited.resize(gridSizeX*gridSizeY, 0);

  // Graph initialisation - to every node is assigned a position in a Row-Major order
  for(int i=0; i<gridSizeX; i++){
    for(int j=0; j<gridSizeY; j++){
      graphNodes.at((i*gridSizeY) + j).setPos((float)i*2, (float)j*2);  ///Position is multiplied by 2 since the access map is sub-sampled
      graphNodes.at((i*gridSizeY) + j).occupied = access_vec.at((i*gridSizeY) + j);

      if(access_vec.at((i*gridSizeY) + j) == 0)  unvisited.at((i*gridSizeY) + j) = 1;
      // cout << (int)graphNodes.at((i*gridSizeY) + j).occupied << " ";
    }
    //cout << endl;
  }
  unvisitedCount = std::accumulate(unvisited.begin(),unvisited.end(), 0);

  numFreeNodes = unvisitedCount;

  STARTNODE = gridSizeY/2;
  nextNode = currentNode = STARTNODE;
  //cout << STARTNODE << endl;
}


void LRTAstar::init_acc(std::ifstream & access_mat){
  /** If input argument of init is only 1 then we assume we have no
   * optimized Probability Transition Matrix and the input file is
   * the Occupancy Grid (access_mat).
   */
  createGraph(access_mat);
  finalPath.push_back(currentNode);
}


void LRTAstar::init_graph_pos(std::ifstream &graph_mat, std::ifstream &Pos_vec){
  /** In this case we don't have an occupancy grid but already a matrix
   * representing the graph so we need to know the position of the vertices,
   * information contained in Pos_Vec. No optimised PTM.
   */
  loadGraphFile(graph_mat);
  loadPosVecFile(Pos_vec);
  finalPath.push_back(currentNode);
}


void LRTAstar::incrCount(int currIndex, int nextIndex, bool isNextVisited){

  graphNodes.at(currIndex).nodeCount = graphNodes.at(nextIndex).nodeCount + 1;
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

/// *** MAIN METHOD *** ///
void LRTAstar::findNext(){


//FIXME quadcopter is reaching position before last one, fix!
/*
  if(unvisitedCount == 0){
    reachLastOne++;
  }
*/
  if( !isCompleted() ){
    /// Look for adjacent nodes and find the one with the smallest number of visits
    /// Before being able to do the adjacency check we have to recover the (i,j) index
    /// values encoded in the graph 1D array as "i*gridSizeY + j" (row-major order)

    currentNode = nextNode;

    int bestCount = std::numeric_limits<int>::max();
    std::vector<int> best_vec;

    for(int j=0; j < graph.size(); j++){
      if(graph[currentNode][j] == 1){
        int tentIndex = j;
        int tentCount = graphNodes.at(tentIndex).nodeCount;

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

  if(unvisitedCount==0)
    return true;
  else
    return false;
}
