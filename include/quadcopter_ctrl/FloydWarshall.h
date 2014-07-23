/*
 * FloydWarshall.hpp
 *
 *  Created on: Jun 6, 2014
 *      Author: francescow
 */

#ifndef FLOYDWARSHALL_HPP_
#define FLOYDWARSHALL_HPP_

#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <climits>
#include <vector>
#include <termColors.h>


using std::cout;
using std::endl;
using std::vector;


class FloydWarshall
{
  static const int Inf = INT_MAX/2-1; // graph[i][j] = Inf if no edge

  vector< vector <int> > graph;
  vector< vector <int> > dist;
  vector< vector <int> > parent;


public:
  FloydWarshall();
  FloydWarshall(vector< vector <int> >& G);
  virtual ~FloydWarshall();
  void loadGraph(vector< vector <int> >& G);

  void printMatrix(vector< vector<int> >& matrix);
  void solve(vector< vector <int> >& D);
  void getPath(int i, int j, vector <int>& path);

};

#endif /* FLOYDWARSHALL_HPP_ */
