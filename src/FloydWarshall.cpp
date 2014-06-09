//	Copyright (c) 2014, Francesco Wanderlingh. 					//
//	All rights reserved.										//
//	License: BSD (http://opensource.org/licenses/BSD-3-Clause)	//

/*
 * FloydWarshall.cpp
 *
 *  Created on: Jun 6, 2014
 *      Author: francescow
 */

#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <climits>
#include <vector>
#include <termColors.h>
#include <algorithm>
#include <quadcopter_ctrl/FloydWarshall.h>
#include "Utils.h"

using std::cout;
using std::endl;
using std::vector;

FloydWarshall::FloydWarshall(){

};


FloydWarshall::FloydWarshall(vector< vector <int> >& G){

  graph = G;
  dist = G;
  parent = G;
};


FloydWarshall::~FloydWarshall()
{

}


void FloydWarshall::loadGraph(vector< vector <int> >& G){

  graph = G;
  dist = G;
  parent = G;
}


void FloydWarshall::printMatrix(vector< vector<int> >& matrix){
   int i, j ,k;
   int n = matrix.size();
   printf("%s\n    ", TC_GREEN);
   for(j=0; j<n; j++){
    Utils::spaced_cout(j);
   }
   printf("%s", TC_NONE);
   cout << endl;
   for(j=0; j<n; j++){
     printf("%s", TC_GREEN);
     Utils::spaced_cout(j);
     printf("%s", TC_NONE);
     for(k=0; k<n; k++){
       if (matrix[j][k] == Inf || matrix[j][k] == -1) cout << "   -";
       else{
         Utils::spaced_cout(matrix[j][k]);
       }
     }
     cout << endl;
   }
   cout << endl;
 }


void FloydWarshall::solve(vector< vector <int> >& D){

  int n = graph.size();

  dist = D;

  //cout << "Size is " << n << endl;

    for(int i = 0; i < n; i++ ){
      for(int j = 0; j < n; j++ ){
        if ( dist[i][j] != 0 && dist[i][j] != Inf ){
          parent[i][j] = i;
        }
        else{
          parent[i][j] = -1;
        }
      }
    }

    for( int k = 0; k < n; k++ ){
      for( int i = 0; i < n; i++ ){
        for( int j = 0; j < n; j++ ){
          int kDist = dist[i][k] + dist[k][j];
          if( kDist < dist[i][j] ){
            dist[i][j] = kDist;
            parent[i][j] = parent[k][j];
          }
        }
      }
    }

    D = dist;

  }


void FloydWarshall::getPath(int i, int j, vector <int>& path){
  if (i == j){
    //cout << i << " ";
    path.push_back(i);
  }else if (parent[i][j] == -1){
    cout << " Path does not exist" << endl;
  }else{
    getPath(i, parent[i][j], path);
    //cout << j << " ";
    path.push_back(j);
  }
}
