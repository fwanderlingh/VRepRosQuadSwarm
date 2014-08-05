//	Copyright (c) 2014, Francesco Wanderlingh. 			//
//	All rights reserved.						//				//
//	License: BSD (http://opensource.org/licenses/BSD-3-Clause)	//

/*
 * OnlineSearchListener.cpp
 *
 *  Created on: Jun 1, 2014
 *      Author: francescow
 */


#include "ros/ros.h"
#include "quadcopter_ctrl/OSmsg.h"
#include <quadcopter_ctrl/CoverAnalysis.h>
#include "termColors.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <ctime>
#include <sstream>

///Used API Services
#include "vrep_common/simRosStartSimulation.h"
#include "vrep_common/simRosStopSimulation.h"

#define DEF_LOOP_RATE 2     // Loop frequency of node in hertz

using std::cout;
using std::endl;
using std::vector;

quadcopter_ctrl::OSmsg osInfo;
int numOfCollected = 0;

std::string testName;
int numFreeNodes;
vector<vector<int> > finalPaths;

/// This class must be subscribed to a topic like "completedPaths" where the online
/// searches are publishing, so that after all the quadcopter performing the online
/// search are done it computes the CoverAnalysis!

std::string get_selfpath();

void pathFromQuads(const quadcopter_ctrl::OSmsg::ConstPtr& onlineSearchInfo){

  cout << "Listener Says: Collected Path [" << onlineSearchInfo->ID << "]" << endl;
  numFreeNodes = onlineSearchInfo->numNodes;
  finalPaths.push_back(onlineSearchInfo->path);
  testName = onlineSearchInfo->fileName;
  numOfCollected++;
}


int main(int argc, char **argv)
{
  /// argv[1] contains the size of the swarm, e.g. the number of robots to be controlled
  if(argc<2){
    printf("%s** argv[1] is empty! Provide size of swarm! **%s\n", TC_RED, TC_NONE);
    exit(EXIT_FAILURE);
  }
  const int num_robots = strtol(argv[1], NULL, 0);

  std::time_t elapsed;

  std::string folder_path = get_selfpath();

  ros::init(argc, argv, "OnlineSearchListener");
  ros::NodeHandle n;

  ros::Subscriber completed_sub = n.subscribe("completedPath", 100, pathFromQuads);

  ros::Rate loop_rate(DEF_LOOP_RATE); //Loop at DEF_LOOP_RATE

  bool firstLoopRun = true;


  while (ros::ok())
  {
    if(firstLoopRun){
      firstLoopRun = false;

      /** STARRT SIMULATION IN V-REP **/
      ros::ServiceClient client_startSimulation =
           n.serviceClient<vrep_common::simRosStartSimulation>("/vrep/simRosStartSimulation");
      vrep_common::simRosStartSimulation srv_startSimulation;
      client_startSimulation.call(srv_startSimulation);

      elapsed = time(NULL);
    }

    if(numOfCollected == num_robots){

      elapsed = time(NULL) - elapsed;

      /** STOP SIMULATION IN V-REP **/
      ros::ServiceClient client_stopSimulation =
           n.serviceClient<vrep_common::simRosStopSimulation>("/vrep/simRosStopSimulation");
      vrep_common::simRosStopSimulation srv_stopSimulation;
      client_stopSimulation.call(srv_stopSimulation);

      printf("%sAll %d paths collected!%s\n", TC_GREEN, num_robots, TC_NONE);

      CoverAnalysis myCoverage(finalPaths, num_robots, numFreeNodes);
      int longest = myCoverage.getLongestPath();
      int total = myCoverage.getTotalLength();
      double st_dev = myCoverage.getStDev();
      printf("%sMax Path Length: %d%s\n", TC_MAGENTA, longest, TC_NONE);
      printf("%sTotal Paths Length: %d%s\n", TC_MAGENTA, total, TC_NONE);
      printf("%sPaths length standard deviation: %f%s\n", TC_MAGENTA, st_dev, TC_NONE);

      /** SAVING RESULTS TO FILE **/
      std::ostringstream oss_n;
      oss_n << num_robots;
      std::string resultsFileName = folder_path + "/Results/" + testName + "_" + oss_n.str() + "_Results";
      std::ofstream results_file;
      cout << resultsFileName;



      results_file.open ( resultsFileName.c_str(), std::fstream::app );
      results_file << num_robots << "\t" << numFreeNodes
          << "\t" << longest << "\t" << total << "\t" << st_dev << "\t" << elapsed << endl;
/*
      results_file << "Quadcopters paths:" << endl;
      for (int i=0; i<finalPaths.size(); ++i){
        results_file << "#" << i << ": ";
        for (int j=0; j<finalPaths.at(i).size(); ++j){
          results_file << finalPaths[i][j] << ' ';
        }
        results_file << endl;
      }
      results_file << endl;
*/
      results_file.close();

      ros::shutdown();

    }

    ros::spinOnce();
    loop_rate.sleep();
  }

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

