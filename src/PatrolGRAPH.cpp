//	Copyright (c) 2014, Francesco Wanderlingh. 					//
//	All rights reserved.										//
//	License: BSD (http://opensource.org/licenses/BSD-3-Clause)	//

/*
 * PatrolGRAPH.cpp
 *
 *  Created on: Jun 21, 2014
 *      Author: francescow
 */

#include "PatrolGRAPH.h"
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <numeric>
#include <algorithm>
#include <cassert>
#include <climits> /** For CHAR_BIT **/
#include <limits>  /** For numeric_limits **/
#include "termColors.h"
#include "Utils.h"
#include "fstream"
#include <sstream>

#define DEBUG_PRINT

using std::cout;
using std::endl;


PatrolGRAPH::PatrolGRAPH() : STARTNODE(0), gridSizeX(0), gridSizeY(0),
    currentNode(STARTNODE), optimized(false), minVisit(1), numFreeNodes(0),
    unvisitedCount(std::numeric_limits<int>::max())
{
  srand(time(NULL) xor getpid()<<16); // xor getpid()<<16);
  // TODO Auto-generated constructor stub

}

PatrolGRAPH::~PatrolGRAPH()
{
  // TODO Auto-generated destructor stub
}


void PatrolGRAPH::loadMatrixFile(std::ifstream &access_mat){

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


void PatrolGRAPH::loadGraphFile(std::ifstream &graph_mat){

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

  /// Here we create a zero matrix of the size of the graph
  vector< vector<int> > _graph(numFreeNodes, vector<int> (numFreeNodes, 0));
  edgeCountMat = _graph;

  unvisited.resize(numFreeNodes, 1);
  unvisitedCount = numFreeNodes;

  graphNodes.resize(numFreeNodes);
}



void PatrolGRAPH::loadPosVecFile(std::ifstream &Pos_vec){

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



void PatrolGRAPH::loadPTMFile(std::ifstream &PTM_mat){

  std::string line;

  while ( getline( PTM_mat, line ) ) {
    std::istringstream is( line );
    PTM.push_back(
        std::vector<double>( std::istream_iterator<double>(is),
                             std::istream_iterator<double>() ) );
  }

  cout << PTM.size() << "  " << edgeCountMat.size() << endl;
  assert(PTM.size() == edgeCountMat.size());
  assert(PTM.at(1).size() == edgeCountMat.at(1).size());

#ifdef DEBUG_PRINT
  cout << "\nOptimized PTM:\n";
  for(int i=0;i<PTM.size();i++){
    for(int j=0; j<PTM.at(1).size();j++){
      if(PTM[i][j] == 0) printf("   0  ");
      else printf("%.2f  ",PTM[i][j]);;
    }
    cout << endl;
  }
#endif
}


void PatrolGRAPH::init_acc(std::ifstream & access_mat,
                           int startingNode, int minVis){
  /** If input argument of init is only 1 then we assume we have no
   * optimized Probability Transition Matrix and the input file is
   * the Occupancy Grid (access_mat).
   */
  optimized = false;
  minVisit = minVis;
  cout << "Optimized: false" << endl;
  createGraph(access_mat);
  currentNode = startingNode;
  finalPath.push_back(currentNode);
  computeProbabilityMat();
}


void PatrolGRAPH::init_graph_pos(std::ifstream &graph_mat,
                                 std::ifstream &Pos_vec,
                                 int startingNode, int minVis){
  /** In this case we don't have an occupancy grid but already a matrix
   * representing the graph so we need to know the position of the vertices,
   * information contained in Pos_Vec. No optimised PTM.
   */
  optimized = false;
  cout << "Optimized: false" << endl;
  minVisit = minVis;
  loadGraphFile(graph_mat);
  loadPosVecFile(Pos_vec);
  computeProbabilityMat();
  currentNode = startingNode;
  finalPath.push_back(currentNode);
}


void PatrolGRAPH::init_acc_ptm(std::ifstream & access_mat,
                               std::ifstream & PTM_mat,
                               int startingNode, int minVis){
  /** In this case we have an Occupancy Grid and an optimised
   *  Probability Transition Matrix.
   */
  optimized = true;
  cout << "Optimized: true" << endl;
  minVisit = minVis;
  createGraph(access_mat);
  loadPTMFile(PTM_mat);
  currentNode = startingNode;
  finalPath.push_back(currentNode);
}


void PatrolGRAPH::init_graph_pos_ptm(std::ifstream &graph_mat,
                                     std::ifstream &Pos_vec,
                                     std::ifstream &PTM_mat,
                                     int startingNode, int minVis){
  /** In this case we don't have an occupancy grid but already a matrix
   * representing the graph so we need to know the position of the vertices,
   * information contained in Pos_Vec. We also load the optimised PTM.
   */
  optimized = true;
  cout << "Optimized: true" << endl;
  minVisit = minVis;
  loadGraphFile(graph_mat);
  loadPosVecFile(Pos_vec);
  loadPTMFile(PTM_mat);
  currentNode = startingNode;
  finalPath.push_back(currentNode);
}


void PatrolGRAPH::createGraph(std::ifstream & INFILE){

  loadMatrixFile(INFILE);       /// Filling the "access_vec" and defining grid sizes

  cout << "Matrix size is: " << gridSizeX << "x" << gridSizeY << endl;

  graphNodes.resize(gridSizeX*gridSizeY);
  unvisited.resize(gridSizeX*gridSizeY, 0);

  // Graph initialisation - to every node is assigned a position in a Row-Major order
  for(int i=0; i<gridSizeX; i++){
    for(int j=0; j<gridSizeY; j++){
      graphNodes.at((i*gridSizeY) + j).setPos((double)i, (double)j);
      graphNodes.at((i*gridSizeY) + j).occupied = access_vec.at((i*gridSizeY) + j);

      if(access_vec.at((i*gridSizeY) + j) == 0)  unvisited.at((i*gridSizeY) + j) = 1;
      // cout << (int)graphNodes.at((i*gridSizeY) + j).occupied << " ";
    }
    //cout << endl;
  }
  unvisitedCount = std::accumulate(unvisited.begin(),unvisited.end(), 0);

  numFreeNodes = unvisitedCount;


  createEdgeMat();

}


void PatrolGRAPH::createEdgeMat(){

  const int n = gridSizeX*gridSizeY;

  cout << "n=" << n << ", access_vec.size()=" << access_vec.size() << endl;
  assert(n == access_vec.size());

  /// Here we create a zero matrix of the size of the graph
  vector< vector<int> > _graph(n, vector<int> (n, 0));
  graph = _graph;
  edgeCountMat = _graph;

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

#ifdef DEBUG_PRINT
  cout << "\nGraph:\n";
  for(int i=0;i<graph.size();i++){
    for(int j=0; j<graph.at(1).size();j++){
      printf("%d  ",graph[i][j]);;
    }
    cout << endl;
  }
#endif

}


void PatrolGRAPH::computeProbabilityMat(){
  /**
   * Here, starting from the graph's edges matrix we derive the Probability
   * Transition Matrix (PTM), where p_ij = 1/|A_i| (A_i=number of incident edges).
   * IN THIS CASE I'M ASSUMING A UNIFORM PROBABILITY MATRIX, i.e. sum{j}(p_ji)=1 ∀i;
   */

  vector< vector<double> > temp_PTM(graph.size(), vector<double>(graph.size(), 0));

  double num_edges;

  /// Here the sum of probabilities of OUT-going edges is equal to 1
  for(int i=0; i<graph.size(); i++){
    num_edges = std::accumulate(graph.at(i).begin(), graph.at(i).end(), 0.0);
    if(num_edges != 0){
      std::transform(graph.at(i).begin(), graph.at(i).end(),
                     temp_PTM.at(i).begin(), std::bind2nd(std::divides<double>(),num_edges));
    }
  }

  PTM = temp_PTM;


#ifdef DEBUG_PRINT
  cout << "\nStandard PTM:\n";
  for(int i=0;i<PTM.size();i++){
    for(int j=0; j<PTM.size();j++){
      if(PTM[i][j] == 0) printf("   0  ");
      else printf("%.2f  ",PTM[i][j]);;
    }
    cout << endl;
  }
#endif

}


void PatrolGRAPH::incrCount(int currIndex, bool currType, boost::array<int, 2> chosenEdge){

  /// Increment vertex count
  graphNodes.at(currIndex).nodeCount++;

  /// Increment edge count
  edgeCountMat[chosenEdge.at(0)][chosenEdge.at(1)]++;

  if(currType == 0){
    unvisited.at(currIndex) = 0;
    // The sum of all elements of unvisited is performed so that when sum is zero we
    // know we have finished. Check is performed in "findNext()"
    unvisitedCount = std::accumulate(unvisited.begin(),unvisited.end(), 0);

    //cout << "unvisitedCount: " << unvisitedCount << endl;
  }


  /*
 //PRINT MAP FOR DEBUGGING PURPOSES

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
void PatrolGRAPH::findNext(){

#ifdef DEBUG_PRINT
  cout << "---\n";
  for(int i=0; i<graphNodes.size(); ++i){
    if(i%gridSizeY == 0) cout << endl;
    cout << graphNodes.at(i).nodeCount << " ";
  }
  cout << "\n\n";
  cout << "I'm on node " << currentNode << " - Node Count=" << graphNodes[currentNode].nodeCount+1 << endl;
#endif

  if( !isCompleted() ){
    /// Look for adjacent nodes and find the one with the smallest number of visits
    /// Before being able to do the adjacency check we have to recover the (i,j) index
    /// values encoded in the graph 1D array as "i*gridSizeY + j" (row-major order)

    /// The count is locally incremented by one (the global increment will be done
    /// after the choosing phase, by publishing a message to a common topic).
    double currentNodeCount = graphNodes[currentNode].nodeCount + 1;

    ///The "i" index of the chosen edge will be the one of the node we're on now = current node
    chosenEdge.at(0) = currentNode;
    std::vector<int> best_vec;

    double deltaP;
    double deltaP_best = std::numeric_limits<double>::max();
    int bestEdgeCount = std::numeric_limits<int>::max();
    int tentIndex;


    for(int j=0; j < graph.size(); j++){
      if(graph[currentNode][j] == 1){
        tentIndex = j;



        if(PTM[currentNode][tentIndex] == 0){
          printf("%sWARNING: 0-probability detected on (%d,%d)%s\n", TC_RED, currentNode, tentIndex, TC_NONE);
        }

        deltaP = ( (double)edgeCountMat[currentNode][tentIndex]/currentNodeCount )
                                                                          - PTM[currentNode][tentIndex];
#ifdef DEBUG_PRINT
        cout << "Edge(" << currentNode << ", " << tentIndex << "), v_c=" << currentNodeCount << ", k_cj=" <<  edgeCountMat[currentNode][tentIndex] << endl;
        cout << "p_cj(d)=" << PTM[currentNode][tentIndex] << ", p_cj(a)=" << (double)edgeCountMat[currentNode][tentIndex]/currentNodeCount << ", delta=" << deltaP << endl;
        cout << endl;
#endif

        if(deltaP <= deltaP_best){
          if(deltaP == deltaP_best){
            best_vec.push_back(tentIndex);
          }else{
            best_vec.clear();
            best_vec.push_back(tentIndex);
          }
          deltaP_best = deltaP;
        }//End checkBest
      }
    }



    /// Now if there is more than one element in the vector choose one randomly.
    /// If size()==1 the modulus function always returns 0 (the first element)
    assert(best_vec.size() != 0);
    int randIndex = rand()%best_vec.size();

#ifdef DEBUG_PRINT
    cout << "Chosen Edge: (" << currentNode << ", " << best_vec.at(randIndex) << ")" << endl;
#endif

    /// The "j" index of the chosen edge will be the one of the (best) target node,
    /// that now also becomes the new current node.
    currentNode = best_vec.at(randIndex);
    chosenEdge.at(1) = currentNode;

    finalPath.push_back(currentNode);
  }
  /*
  cout << "Vertex 15 counts:\n"
       << "Visits: " << graphNodes[15].nodeCount << ", "
       << "Edge (15,4):  " << edgeCountMat[15][4] << ", "
       << "Edge (15,16): " << edgeCountMat[15][16] << ", "
       << "Edge (15,27): " << edgeCountMat[15][16] << endl
       << "---" << endl;
   */

}


bool PatrolGRAPH::getCurrentType(){

  if(graphNodes.at(currentNode).nodeCount == 0)
    return 0;
  else
    return 1;
}


float PatrolGRAPH::getCurrentCoord(char coordinate){
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


bool PatrolGRAPH::isCompleted(){

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

  /*
  if(unvisitedCount == 0)
    return true;
  else
    return false;
   */
}
