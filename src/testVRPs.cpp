//	Copyright (c) 2014, Francesco Wanderlingh. 					//
//	All rights reserved.										//
//	License: BSD (http://opensource.org/licenses/BSD-3-Clause)	//

/*
 * testVRPs.cpp
 *
 *  Created on: Jun 12, 2014
 *      Author: francescow
 */



#include "ros/ros.h"
#include "kernelNode.h"
#include <cstring>
#include <sstream>
#include <unistd.h>
#include <ctime>


using std::cout;
using std::endl;
using std::vector;


std::string get_selfpath(void);


int main(int argc, char **argv)
{
  /// argv[1] contains the size of the swarm, e.g. the number of robots to be controlled
  if(argc<2){
    printf("%s** argv[1] is empty! Provide size of Map! **%s\n", TC_RED, TC_NONE);
    exit(EXIT_FAILURE);
  }
  int mapDim = strtol(argv[1], NULL, 0);


  std::string folder_path = get_selfpath();
  //std::string filename = "access_mat_subs";

  std::string map_dim(argv[1]);
  std::string sizeTag = map_dim + "x" + map_dim;
  std::string filename = "free_mat_" + sizeTag;
  std::string map_matrix_path = folder_path + "/" + filename;

  std::string save_path = folder_path + "/Results/VRP_Results_Astar_" + sizeTag;


/*
  std::ofstream map_file;
  map_file.open ( map_matrix_path.c_str());

  for(int i=0; i<mapDim; i++){
    for(int j=0; j<mapDim; j++){
      map_file << "0";
      if(j != (mapDim-1) ) map_file << " ";
    }
    map_file << "\n";
  }

  map_file.close();
*/

  for(int i=1; i<=10; i++){
    int num_robots = i;
    /// Constructor inputs are (mapToExplore, numOfAgents) ///
    //VrpGreedy myVrp(map_matrix_path, num_robots);
    VrpGreedyAstar myVrp(map_matrix_path, num_robots);

    struct timespec requestStart, requestEnd;

    clock_gettime(CLOCK_REALTIME, &requestStart);
    myVrp.solve();
    clock_gettime(CLOCK_REALTIME, &requestEnd);
    double timeElapsed = ( requestEnd.tv_sec - requestStart.tv_sec )
                           + ( requestEnd.tv_nsec - requestStart.tv_nsec ) / 1E9;

    performanceIndexes(myVrp, save_path, timeElapsed*1E3); /// We multiply the time to get it in milliseconds

    //std::cin.get();
  }

  //savePathsToFile(myVrp, folder_path);


  return 0;
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




