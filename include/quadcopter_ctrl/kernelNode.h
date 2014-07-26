/*
 * kernelNode.h
 *
 *  Created on: Jun 12, 2014
 *      Author: francescow
 */

#ifndef KERNELNODE_H_
#define KERNELNODE_H_

#include <iostream>
#include <fstream>
#include <vector>
#include "graphStructs.h"
#include "termColors.h"
#include "VrpGreedyFW.h"
#include "VrpGreedyFW2.h"
#include "VrpGreedyAstar.h"
#include "CoverAnalysis.h"


#define INTSTRSIZE ((CHAR_BIT * sizeof(int) - 1) / 3 + 2)

using std::cout;
using std::endl;
using std::vector;


template<typename T>
void performanceIndexes(T VRP, std::string save_path, double execTime){

  int numRobots = VRP.getNumRobots();
  int numFreeNodes = VRP.getNumFreeNodes();
  std::vector< vector<int> > Paths = VRP.getPaths();

  printf("Coverage input args: [ numRobots=%d, numFreeNodes=%d ]\n", numRobots, numFreeNodes);
  CoverAnalysis myCoverage(Paths, numRobots, numFreeNodes);

  int longest = myCoverage.getLongestPath();
  int total = myCoverage.getTotalLength();
  double st_dev = myCoverage.getStDev();
  printf("%sMax Path Length: %d%s\n", TC_MAGENTA, longest, TC_NONE);
  printf("%sTotal Paths Length: %d%s\n", TC_MAGENTA, total, TC_NONE);
  printf("%sPaths length standard deviation: %f%s\n", TC_MAGENTA, st_dev, TC_NONE);
  printf("%sAlgorithm execution time: %f ms%s\n", TC_MAGENTA, execTime, TC_NONE);

  std::ofstream results_file;
  //cout << "save_path" << save_path << endl;

  results_file.open ( save_path.c_str(), std::fstream::app );

  if( results_file.is_open() ) {

    results_file << numRobots << "\t" << numFreeNodes + 1
        << "\t" << longest << "\t" << total << "\t" << st_dev << "\t" << execTime << "\n";
    results_file.close();

  }else{
    printf("%s ** ERROR writing results file! **%s\n", TC_RED, TC_NONE);
  }


}


template<typename T>
void savePathsToFile(T VRP, std::string folder_save_path){
  ///  SAVE PATHS TO FILE  ///

  vector< vector<int> > Paths = VRP.getPaths();
  vector<graphNode> graphNodes = VRP.getGraphNodes();


  for (std::vector< vector<int> >::iterator itr = Paths.begin(); itr != Paths.end(); ++itr){

    std::ofstream pathfile;

    /** Getting path index from iterator and converting it into string : **/
    char pathIndex[INTSTRSIZE];
    sprintf(pathIndex, "%d", (int)(itr - Paths.begin()) );
    std::string pathfileName(pathIndex);
    pathfileName = folder_save_path + "/path_"  + pathfileName;

    pathfile.open ( pathfileName.c_str() );

    if( pathfile.is_open() ) {

      for (std::vector<int>::iterator itc = itr->begin(); itc != itr->end(); ++itc){
        pathfile << graphNodes.at(*itc).posx << ' ' << graphNodes.at(*itc).posy;
        //cout << graphNodes.at(*itc).posx << ' ' << graphNodes.at(*itc).posy;
        pathfile << '\n';
        //cout << endl;
      }
      //cout << endl;
      pathfile.close();
      printf("%sPath_%s file created...%s\n", TC_GREEN, pathIndex, TC_NONE);

    }else{
      printf("%s ** ERROR writing path_%s file! **%s\n", TC_RED, pathIndex, TC_NONE);
    }

  }
}



#endif /* KERNELNODE_H_ */
