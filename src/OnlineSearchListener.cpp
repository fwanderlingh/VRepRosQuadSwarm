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
#include <vector>

#define DEF_LOOP_RATE 2      // Loop frequency of node in hertz

using std::cout;
using std::endl;
using std::vector;

quadcopter_ctrl::OSmsg osInfo;
int numOfCollected = 0;

int numFreeNodes;
vector<vector<int> > finalPaths;

/// This class must be subscribed to a topic like "completedPaths" where the online
/// searches are publishing, so that after all the quadcopter performing the online
/// search are done it computes the CoverAnalysis!


void pathFromQuads(const quadcopter_ctrl::OSmsg::ConstPtr& onlineSearchInfo){

  cout << "Listener Says: Collected Path [" << onlineSearchInfo->ID << "]" << endl;
  numFreeNodes = onlineSearchInfo->numNodes;
  finalPaths.push_back(onlineSearchInfo->path);
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

  ros::init(argc, argv, "OnlineSearchListener");
  ros::NodeHandle n;

  ros::Subscriber completed_sub = n.subscribe("completedPath", 100, pathFromQuads);

  ros::Rate loop_rate(DEF_LOOP_RATE); //Loop at DEF_LOOP_RATE


  while (ros::ok())
  {

    if(numOfCollected == num_robots){

      printf("%sAll %d paths collected!%s\n", TC_GREEN, num_robots, TC_NONE);

      CoverAnalysis myCoverage(finalPaths, num_robots, numFreeNodes);
      int longest = myCoverage.getLongestPath();
      int total = myCoverage.getTotalLength();
      printf("%sMax Path Length: %d%s\n", TC_MAGENTA, longest, TC_NONE);
      printf("%sTotal Paths Length: %d%s\n", TC_MAGENTA, total, TC_NONE);

      ros::shutdown();

    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;

}

