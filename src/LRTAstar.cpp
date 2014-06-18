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
#include <climits>      /* INT_MAX */
#include <numeric>      /* multiply, accumulate */
#include <cstdlib>

#define STARTNODE 5

using std::cout;
using std::endl;


LRTAstar::LRTAstar() : gridSizeX(0), gridSizeY(0),
                          currentNode(STARTNODE), unvisitedCount(INT_MAX),
                          nextNode(STARTNODE)
{

  cout << "unvisitedCount:" << unvisitedCount << endl;

}


LRTAstar::LRTAstar(std::ifstream & INFILE) : currentNode(STARTNODE), unvisitedCount(INT_MAX),
                                                 nextNode(STARTNODE)
{

  initGraph(INFILE);

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


void LRTAstar::initGraph(std::ifstream & INFILE){

  srand(time(NULL) xor getpid()<<16);
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
}


void LRTAstar::incrCount(int currIndex, int nextIndex, bool nextType){

  graphNodes.at(currIndex).nodeCount = graphNodes.at(nextIndex).nodeCount + 1;
  //graphNodes.at(currIndex).nodeCount++;

  if(nextType == 0){
    unvisited.at(nextIndex) = 0;
    // The sum of all elements of unvisited is performed so that when sum is zero we
    // know we have finished. Check is performed in "findNext()"
    unvisitedCount = std::accumulate(unvisited.begin(),unvisited.end(), 0);
  }


  if(unvisitedCount == 0){
    finalCountMap.resize(gridSizeX*gridSizeY);

    for(int i=0; i<(gridSizeX*gridSizeY); i++){

      finalCountMap.at(i) = graphNodes.at(i).nodeCount;
    }
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

  if( !isCompleted() ){
    /// Look for adjacent nodes and find the one with the smallest number of visits
    /// Before being able to do the adjacency check we have to recover the (i,j) index
    /// values encoded in the graph 1D array as "i*gridSizeY + j" (row-major order)

    currentNode = nextNode;

    int bestCount = INT_MAX;
    int bestNeighbour = currentNode;
    std::vector<int> best_vec;

    int current_i = currentNode/gridSizeY;
    int current_j = currentNode%gridSizeY;

    for(int i_shift=-1; i_shift<=1; ++i_shift){
      for(int j_shift=-1; j_shift<=1; ++j_shift){

        int tent_i = current_i + i_shift;
        int tent_j = current_j + j_shift;

        if( (tent_i>=0) && (tent_i<gridSizeX) && (tent_j>=0) && (tent_j<gridSizeY) ///RANGE CHECK
            && (i_shift!=0 || j_shift!=0)  ///<--- don't check same node of current
            && (i_shift*j_shift == 0)  ///<--- don't allow diagonal movements
            )
        {
          int tentIndex = tent_i*gridSizeY + tent_j;

          if( (graphNodes.at(tentIndex).occupied == 0) ){   ///OCCUPANCY CHECK

            if( graphNodes.at(tentIndex).nodeCount <= bestCount ){

              ///Look if there are other nodes with same count
              if( graphNodes.at(tentIndex).nodeCount == bestCount ){
                best_vec.push_back(tentIndex);
              }else{
                best_vec.clear();
                best_vec.push_back(tentIndex);
                bestCount = graphNodes.at(tentIndex).nodeCount;
              }

            }//End checkBest
          }//End occupancy check
        }//End range check
      }//End j_shift for-loop (y scan)
    } //End i_shift for-loop (x scan)

    /// Now if there is more than one element in the vector choose one randomly.
    /// If size()==1 the modulus function always returns 0 (the first element)

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
    /// (check quadcopterRosCtrl.cpp or quadNodeCount.cpp for further details)
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

  if(unvisitedCount == 0)
    return true;
  else
    return false;
}
