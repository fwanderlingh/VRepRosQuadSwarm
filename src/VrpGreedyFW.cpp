//	Copyright (c) 2014, Francesco Wanderlingh. 					//
//	All rights reserved.										//
//	License: BSD (http://opensource.org/licenses/BSD-3-Clause)	//

/*
 * VrpGreedy.cpp
 *
 *  Created on: Apr 29, 2014
 *      Author: francescow
 */

#include "VrpGreedyFW.h"
#include "quadcopter_ctrl/FloydWarshall.h"
#include <quadcopter_ctrl/CoverAnalysis.h>
#include "Utils.h"
#include <iostream>
#include <cfloat>
#include <limits>
#include <cassert>
#include <cmath>
#include <ctime>
#include <sstream>
#include <iterator>
#include <algorithm>


#define MAX_FOV 2
#define SQRT2 1.4143
/// Ceiled to avoid failure in check condition in "solve()"
/// due to small decimal errors.

#define DEF_GRID_X 4
#define DEF_GRID_Y 4
#define DEF_NUM_ROB 3

using std::cout;
using std::endl;
using std::vector;

struct IncGenerator {
  int current_;
  IncGenerator (int start) : current_(start) {}
  int operator() () { return current_++; }
};



VrpGreedy::VrpGreedy() : STARTNODE(0),
    gridSizeX(0),
    gridSizeY(0),
    numRobots(0),  // Default Initialisation
    v(0)
{
/*  cout << "Default configuration:" << endl;
  cout << "- Default grid is empty" << endl;
  cout << "- " << numRobots << " Robots" << endl;

  access_vec.resize(gridSizeX*gridSizeY, 0);

  // Parameters initialisation
  deltaBest = FLT_MAX;
  deltavip = FLT_MAX;
  bigL = -FLT_MAX;
  liMin = FLT_MAX;*/

}


VrpGreedy::~VrpGreedy()
{
  // TODO Auto-generated destructor stub
}


void VrpGreedy::loadGraphFile(std::ifstream &graph_mat){

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

  graphNodes.resize(numFreeNodes);

  unvisitedNodes.resize(numFreeNodes);
  IncGenerator g (0);
  std::generate(unvisitedNodes.begin(), unvisitedNodes.end(), g); // Fill with 0, 1, ..., numFreeNodes.

  std::cin.get();
}


void VrpGreedy::loadPosVecFile(std::ifstream &Pos_vec){

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


void VrpGreedy::init_acc(std::ifstream &access_mat, int agents){

  numRobots = agents;
  createGraph(access_mat);
}


void VrpGreedy::init_graph_pos(std::ifstream &graph_mat, std::ifstream &Pos_vec, int agents){
  /** In this case we don't have an occupancy grid but already a matrix
   * representing the graph so we need to know the position of the vertices,
   * information contained in Pos_Vec. No optimised PTM.
   */
  numRobots = agents;
  loadGraphFile(graph_mat);
  loadPosVecFile(Pos_vec);
}


void VrpGreedy::loadMatrixFile(std::ifstream &access_mat){
  /**
   * Here gridSizeX and gridSizeY are deducted from the size of the input matrix file
   */

  if( access_mat.is_open() ) {
    int val;
    int num_nl = 0;
    while( access_mat >> val ){
      if(access_mat.peek() == '\n') num_nl++;
      access_vec.push_back( val );
    }

    gridSizeX = num_nl;
    gridSizeY = access_vec.size()/num_nl;

    access_mat.close();

  }
  else{ cout << "Error reading file!" << endl; }

}


void VrpGreedy::createEdgeMat(){

    const int n = gridSizeX*gridSizeY;

    assert(n == access_vec.size());
    vector<int> _graph(n, Inf);
    vector< vector<int> > __graph(n, _graph);
    graph = __graph;
    int row, col;    // Main indexes
    int row_shift, col_shift; // To move around spatial adjacents
    int nb_row, nb_col;       // Adjacent indexes
/*
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
*/

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
              //&& (row_shift*col_shift == 0)  ///<--- don't allow diagonal movements
              )
              //&& ( (row_shift*row_shift xor col_shift*col_shift)==1 ) <--last 2 statements compressed in one condition
            {
              if(access_vec[nb_row*gridSizeY+nb_col] == 0){
                /// Create the edge between "i" and its "free neighbour"
                graph[i][nb_row*gridSizeY+nb_col] = 1;
                graph[nb_row*gridSizeY+nb_col][i] = 1;
              }
            }
          }
        }
      }
    }

    distanceMat = graph;

    for(int i=0; i<n; i++) distanceMat[i][i] = 0;

}


void VrpGreedy::createGraph(std::ifstream & INFILE){

  loadMatrixFile(INFILE);

  //cout << "Matrix size is: " << gridSizeX << "x" << gridSizeY << endl;

  graphNodes.resize(gridSizeX*gridSizeY);
  unvisitedNodes.reserve( graphNodes.size() );

  STARTNODE = gridSizeY/2;
  access_vec.at(STARTNODE) = 1;    //STARTNODE is set as start for all the agents

  /// Graph initialisation - to every node is assigned a position
  for(int i=0; i<gridSizeX; i++){
    for(int j=0; j<gridSizeY; j++){
      graphNodes.at((i*gridSizeY) + j).setPos((double)i*2, (double)j*2);
      graphNodes.at((i*gridSizeY) + j).occupied = access_vec.at((i*gridSizeY) + j);
      //cout << (int)graphNodes.at((i*gridSizeY) + j).occupied << " ";
      if (graphNodes.at((i*gridSizeY)+ j).occupied == 0){
        unvisitedNodes.push_back((i*gridSizeY) + j); //Adding the free nodes to the list of unvisited
      }
    }
    //cout << endl;
  }

  //unvisitedNodes.shrink_to_fit();

  numFreeNodes = unvisitedNodes.size();

  createEdgeMat();


}


void VrpGreedy::solve(){

  /// Path initialisation
   vector<int> path;
   path.push_back(STARTNODE);       // Every path initially is just 2 nodes: Start + End(=start)
   path.push_back(STARTNODE);

   for(int i = 0; i < numRobots; i++){
     Paths.push_back(path); // Define a path for every robot
   }

   Paths.reserve( graphNodes.size() * numRobots); // To avoid an excessive number of implicit resizes


  printf("\n%s** Using the VRP-FloydWarshall algorithm **%s\n", TC_CYAN, TC_NONE);

  FloydWarshall myFW(graph);
  myFW.solve(distanceMat);

  //myFW.printMatrix(graph);

  //cout << "\nDistances: ";
  //myFW.printMatrix(distanceMat);

  //std::cin.get();

  int start1, start2, target;
  vector<int> fwPath;

  pathTentative.reserve(numFreeNodes+2);


  while(unvisitedNodes.size() > 0){
/*
    std::cout << "The contents of Paths are:" << endl;
    for (itr = Paths.begin(); itr != Paths.end(); ++itr){
      cout << "#" << itr - Paths.begin() << ": ";
      for (itc = itr->begin(); itc != itr->end(); ++itc){
        std::cout << *itc << ' ';
      }
      std::cout << '\n';
    }
    std::cin.get();
*/
    // Delta increments initialisation
    deltaBest = FLT_MAX;
    deltavip = FLT_MAX;
    bigL = -FLT_MAX;

    /// Initialise bigL as the MAX path length among all the current paths
    for(it = Paths.begin(); it!=Paths.end(); ++it){
      if( pathLength(*it) > bigL ) bigL = pathLength(*it);
    }

    ///*** MAIN NESTED LOOPS ***///

    for (v = 0; v < unvisitedNodes.size(); v++ ){                      // For every Node v
      for (itr = Paths.begin(); itr != Paths.end(); ++itr){            // On every path i
        liMin = FLT_MAX;
        for (itc = (itr->begin()+1); itc != itr->end(); ++itc){        // In every position p (reversed)


          //cout << "_ Insert position = " << (itc - itr->begin()) << " _" << endl;
/*
          printf("Inserting node %d between node %d and %d\n", unvisitedNodes[v], *(itc-1), *itc );
          printf("The edges value are: e(%d,%d)=%d, e(%d,%d)=%d\n",
                  *(itc-1), unvisitedNodes[v], graph[*(itc-1)][unvisitedNodes[v]],
                   unvisitedNodes[v], *itc, graph[*itc][unvisitedNodes[v]]);
*/


          /** This is necessary to avoid modifying the current path
           * vector (which is NOT allowed since we're iterating
           * inside it), and keep track of the insertion index.  **/
          pathTentative = *itr;
          vector<int>::iterator iTent = pathTentative.begin() + (itc - itr->begin());

          /// Following if: If node is adjacent to path just insert it ///
          if( graph[*(itc-1)][unvisitedNodes[v]] == 1 && graph[*itc][unvisitedNodes[v]] == 1){

            pathTentative.insert(iTent, unvisitedNodes[v]);

            checkBest(true);

          }
          /// Otherwise: insert the shortest traversable path to "get there and go back" ///
          else{
            target = unvisitedNodes[v];
            fwTent.clear();

            //if((dist(graphNodes[*(itc - 1)], graphNodes[unvisitedNodes[v]])) > minDist){
            if(graph[*(itc-1)][unvisitedNodes[v]] != 1){

              ///Way there
              //cout << "\nWay there: ";
              start1 = *(itc-1);
              myFW.getPath(start1, target, fwPath);
              if(fwPath.size()>2){
                for(int i=1; i<(fwPath.size()-1); i++){
                  //cout << fwPath[i] << " ";
                  fwTent.push_back(fwPath[i]);
                }
              }
            }
            fwPath.clear();


            fwTent.push_back(unvisitedNodes[v]);

            //if((dist(graphNodes[*itc], graphNodes[unvisitedNodes[v]])) > minDist){
            if(graph[*itc][unvisitedNodes[v]] != 1){

              ///Way back
              //cout << "\nWay back: ";
              start2 = *itc;
              myFW.getPath(target, start2, fwPath);
              if(fwPath.size()>2){
                for(int i=1; i<(fwPath.size()-1); i++){
                  //cout << fwPath[i] << " ";
                  fwTent.push_back(fwPath[i]);
                }
              }
            }
            fwPath.clear();


            pathTentative.insert(iTent, fwTent.begin(), fwTent.end());

            checkBest(false);

          }

        } // END P(positions)
      } // END I (robots)
    } // END V (nodes)

/*
    printf("About to insert node %d in position %d of path %d\n",
           unvisitedNodes[choice.v], (int)(choice.p - choice.i->begin()), (int)(choice.i-Paths.begin()));
    printf("The contents of the interPath are: ");
    for(int i=0; i<choice.interPath.size();i++) cout << choice.interPath.at(i) << " ";
    std::cin.get();
  */

    if(choice.neighb == true){
      /// Inserting chosen best node v, in position p, in path of robot i
      choice.i->insert(choice.p, unvisitedNodes[choice.v]);
    }else{
      /// Inserting chosen best node v, along with all the path to reach it
      choice.i->insert(choice.p, choice.interPath.begin(), choice.interPath.end());
    }

    //cout << "About to erase it from unvisited list\n";
    unvisitedNodes.erase(unvisitedNodes.begin() + choice.v);     // Delete it from list of unvisited

    cout << "unvisitedNodes.size() = " << (int)unvisitedNodes.size() << "       " << '\r';
    cout.flush();

  } // END WHILE

  cout << '\n';
/*
  cout << "The contents of Paths are:" << endl;
  for (itr = Paths.begin(); itr != Paths.end(); ++itr){
    cout << "#" << itr - Paths.begin() << ": ";
    for (itc = itr->begin(); itc != itr->end(); ++itc){
      cout << *itc << ' ';
    }
    cout << '\n';
  }
*/

}



double VrpGreedy::pathLength(vector<int> &path){

  //FIXME distance//
  double length = 0;
  for(vector<graphNode>::size_type i = 0; i < (path.size() - 1); i++){
    length = length + sqrt(pow((graphNodes[path[i]].posx - graphNodes[path[i+1]].posx),2) +
                           pow((graphNodes[path[i]].posy - graphNodes[path[i+1]].posy),2) /*+
                           pow((graphNodes[path[i]].posz - graphNodes[path[i+1]].posz),2)*/ );
  }
  return length;
}


void VrpGreedy::checkBest(bool isNeighbour){

  double tentPathLenght = pathLength(pathTentative);

  if(tentPathLenght < liMin){
    liMin = tentPathLenght;

    // * Objective Function * //
    deltavip = liMin - bigL;

    if(deltavip < deltaBest){
      deltaBest = deltavip;
      choice.set_vipn(v, itr, itc, isNeighbour);
      if(!isNeighbour){
        choice.interPath = fwTent;
      }
    }//End Check Global Best
  }//End Check Local Best

}
