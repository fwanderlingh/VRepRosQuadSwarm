//	Copyright (c) 2014, Francesco Wanderlingh.                 //
//	All rights reserved.                                       //
//	License: BSD (http://opensource.org/licenses/BSD-3-Clause) //

/*
 * FW_test.cpp
 *
 *  Created on: Jun 6, 2014
 *      Author: francescow
 */

#include <iostream>
#include <curses.h>
#include <cstdlib>
#include <climits>
#include <vector>
#include <quadcopter_ctrl/termColors.h>
#include <quadcopter_ctrl/FloydWarshall.h>
#include <cassert>

using std::cout;
using std::endl;
using std::vector;

const int Inf = INT_MAX/2-1;// graph[i][j] = Inf if no edge

int _access_vec[] = {0, 0, 0,
                     0, 1, 0,
                     1, 1, 0,
                     0, 0, 0};

void spaced_cout(int value);

int main(int argc, char **argv){

  int gridSizeX = 4;
  int gridSizeY = 3;
  const int n = gridSizeX*gridSizeY;
  vector<int> access_vec(_access_vec, _access_vec + sizeof(_access_vec)/sizeof(int));

  vector<int> _graph(n, Inf);
  vector< vector<int> > graph(n, _graph);
  vector< vector<int> > dist(n, _graph);
  int row, col;    // Main indexes
  int row_shift, col_shift; // To move around spatial adjacents
  int nb_row, nb_col;       // Adjacent indexes

  printf("%sOccupancy Map:%s",TC_RED, TC_NONE);
  for(int j=0; j<n; j++){
    if(j%gridSizeY == 0) cout << endl;
    if( access_vec[j] == 1 ){
      printf("%s",TC_RED);
      spaced_cout(j);
      printf("%s", TC_NONE);
    }
    else spaced_cout(j);
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
            && (row_shift*col_shift == 0)  ///<--- don't allow diagonal movements
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

  dist = graph;

  for(int i=0; i<n; i++) dist[i][i] = 0;


  FloydWarshall myFW(graph);
  myFW.solve(dist);

  //printf("%sDistances:%s", TC_GREEN, TC_NONE);
  //myFW.printMatrix(dist);

  int start;
  int target;
  vector <int> path;

  cout << "Provide start vertex: ";
  std::cin >> start;
  assert(start<n);

  cout << "Provide target vertex: ";
  std::cin >> target;
  assert(target<n);

  printf("%sThe path from %d to %d%s",  TC_CYAN, start, target, TC_NONE);
    myFW.getPath(start, target, path);
    if( path.size() > 0 ){
      printf("%s is long %d and goes through:%s\n",
             TC_CYAN, dist[start][target], TC_NONE);
      //if( fwPath.size() == 2 ) cout << "Come on man, they're adjacent!\n";
      //else{
        for(int k=0; k<path.size(); k++){
          cout << path.at(k) << " ";
        }
        cout << endl;
      //}
    }

  return 0;

}

void spaced_cout(int value){
    if(value < 10) cout << "  " << value;
    else cout << " " << value;
}




