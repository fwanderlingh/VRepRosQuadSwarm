//	Copyright (c) 2014, Francesco Wanderlingh. 					//
//	All rights reserved.										//
//	License: BSD (http://opensource.org/licenses/BSD-3-Clause)	//

/*
 * graphAlgsTest.cpp
 *
 *  Created on: Jul 1, 2014
 *      Author: francescow
 */


#include "GraphAlgs.h"
#include <vector>

using std::cout;
using std::endl;
using std::vector;


std::string get_selfpath(void);

int main(int argc, char** argv){

  std::string filename = "access_mat_subs";
  //std::string filename = "little_matrix";
  std::string folder_path = get_selfpath();
  std::string map_matrix_path = folder_path + "/" + filename;
  std::string nodeCountMap_path = folder_path + "/CountMaps/CountMap_30minutes";

  GraphAlgs myGraph(map_matrix_path);

  myGraph.loadNodeCountMapFromFile(nodeCountMap_path);

  myGraph.stdevFromExpectedVisits(); //prints a matrix




}



std::string get_selfpath() {
  char buff[2048];
  ssize_t len = ::readlink("/proc/self/exe", buff, sizeof(buff)-1);
  if (len != -1) {
    buff[len] = '\0';
    std::string path(buff);   ///Here the executable name is still in
    std::string::size_type t = path.find_last_of("/");   // Here we find the last "/"
    path = path.substr(0,t);                             // and remove the rest (exe name)
    return path;
  } else {
    printf("Cannot determine file path!\n");
  }
}
