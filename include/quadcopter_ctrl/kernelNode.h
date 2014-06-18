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
#include "VrpGreedyAstar.h"
#include "CoverAnalysis.h"


#define INTSTRSIZE ((CHAR_BIT * sizeof(int) - 1) / 3 + 2)

using std::cout;
using std::endl;
using std::vector;


template<typename T>
void performanceIndexes(T VRP, std::string folder_save_path, double execTime){

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

  std::string resultsFileName = folder_save_path + "/Results/VRP_Results";

  results_file.open ( resultsFileName.c_str(), std::fstream::app );

  results_file << numRobots << "\t" << numFreeNodes
      << "\t" << longest << "\t" << total << "\t" << st_dev << "\t" << execTime << "\n";

  results_file.close();


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
    for (std::vector<int>::iterator itc = itr->begin(); itc != itr->end(); ++itc){
      pathfile << graphNodes.at(*itc).posx << ' ' << graphNodes.at(*itc).posy;
      //cout << graphNodes.at(*itc).posx << ' ' << graphNodes.at(*itc).posy;
      pathfile << '\n';
      //cout << endl;
    }
    //cout << endl;
    pathfile.close();

  }
  std::cout << "\"Paths\" files created!" << endl;
}



#endif /* KERNELNODE_H_ */
