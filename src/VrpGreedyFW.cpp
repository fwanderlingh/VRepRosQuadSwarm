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
#include <cassert>
#include <cmath>
#include <ctime>


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


VrpGreedy::VrpGreedy() : gridSizeX(DEF_GRID_X),
    gridSizeY(DEF_GRID_Y),
    numRobots(DEF_NUM_ROB),  // Default Initialisation
    minDist(FLT_MAX),
    v(0)
{
  cout << "Default configuration:" << endl;
  cout << "- Default grid is empty" << endl;
  cout << "- " << numRobots << " Robots" << endl;

  access_vec.resize(gridSizeX*gridSizeY, 0);

  // Parameters initialisation
  deltaBest = FLT_MAX;
  deltavip = FLT_MAX;
  bigL = -FLT_MAX;
  liMin = FLT_MAX;

}


VrpGreedy::VrpGreedy(std::string acc_matrix_path) : numRobots(DEF_NUM_ROB)
{
  loadMatrixFile(acc_matrix_path);
}


VrpGreedy::VrpGreedy(std::string acc_matrix_path, int agents)
{
  loadMatrixFile(acc_matrix_path);
  numRobots = agents;
}


VrpGreedy::~VrpGreedy()
{
  // TODO Auto-generated destructor stub
}


float VrpGreedy::pathLength(vector<int> &path){


  float length = FLT_MIN;
  for(vector<graphNode>::size_type i = 0; i < (path.size() - 1); i++){
    length = length + sqrt(pow((graphNodes[path[i]].posx - graphNodes[path[i+1]].posx),2) +
                           pow((graphNodes[path[i]].posy - graphNodes[path[i+1]].posy),2) /*+
                           pow((graphNodes[path[i]].posz - graphNodes[path[i+1]].posz),2)*/ );
  }
  return length;
}


void VrpGreedy::loadMatrixFile(std::string acc_matrix_path){
  /**
   * Here gridSizeX and gridSizeY are deducted from the size of the input matrix file
   */

  std::ifstream access_mat;
  access_mat.open( acc_matrix_path.c_str() );

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


void VrpGreedy::createGraph(){

    const int n = gridSizeX*gridSizeY;

    assert(n == access_vec.size());
    vector<int> _graph(n, Inf);
    vector< vector<int> > __graph(n, _graph);
    graph = __graph;
    int row, col;    // Main indexes
    int row_shift, col_shift; // To move around spatial adjacents
    int nb_row, nb_col;       // Adjacent indexes

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


void VrpGreedy::init(){

  createGraph();

  //cout << "Matrix size is: " << gridSizeX << "x" << gridSizeY << endl;

  graphNodes.resize(gridSizeX*gridSizeY);
  unvisitedNodes.reserve( graphNodes.size() );

  int STARTNODE = gridSizeY/2;
  access_vec.at(STARTNODE) = 1;    //STARTNODE is set as start for all the agents


  /// Graph initialisation - to every node is assigned a position
  for(int i=0; i<gridSizeX; i++){
    for(int j=0; j<gridSizeY; j++){
      graphNodes.at((i*gridSizeY) + j).setPos((float)i*2, (float)j*2);
      graphNodes.at((i*gridSizeY) + j).occupied = access_vec.at((i*gridSizeY) + j);
      //cout << (int)graphNodes.at((i*gridSizeY) + j).occupied << " ";
      if (graphNodes.at((i*gridSizeY)+ j).occupied == 0){
        unvisitedNodes.push_back((i*gridSizeY) + j); //Adding the free nodes to the list of unvisited
      }
    }
    //cout << endl;
  }

  numFreeNodes = unvisitedNodes.size();

  minDist = (dist(graphNodes.at(0), graphNodes.at(1)) + FLT_MIN)*SQRT2;
  //cout << "minDist=" << minDist << endl;

  // Path initialisation
  vector<int> path;
  path.push_back(STARTNODE);       // Every path initially is just 2 nodes: Start + End(=start)
  path.push_back(STARTNODE);

  for(int i = 0; i < numRobots; i++){
    Paths.push_back(path); // Define a path for every agent
  }

  Paths.reserve( graphNodes.size() * numRobots);

  //createCycles();
}



void VrpGreedy::solve(){

  init();

  printf("\n%s** Using the VRP-FloydWarshall algorithm **%s\n", TC_CYAN, TC_NONE);

  FloydWarshall myFW(graph);
  myFW.solve(distanceMat);


  //myFW.printMatrix(graph);


  //cout << "\nDistances: ";
  //myFW.printMatrix(distanceMat);

  //std::cin.get();

  int start1, start2, target;
  vector<int> fwPath;


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

  cout << "The contents of Paths are:" << endl;
  for (itr = Paths.begin(); itr != Paths.end(); ++itr){
    cout << "#" << itr - Paths.begin() << ": ";
    for (itc = itr->begin(); itc != itr->end(); ++itc){
      cout << *itc << ' ';
    }
    cout << '\n';
  }


}



float VrpGreedy::dist(graphNode &a, graphNode &b){

  float dist = sqrt ( pow(a.posx - b.posx, 2.0)
                      + pow(a.posy - b.posy, 2.0)
                      //+ pow(a.posz - b.posz, 2.0)
                      );

  //cout << "Dist=" << dist << endl;
  return dist;
}


void VrpGreedy::checkBest(bool isNeighbour){

  int tentPathLenght = pathLength(pathTentative);

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
