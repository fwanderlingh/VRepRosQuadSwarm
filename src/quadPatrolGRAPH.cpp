//	Copyright (c) 2014, Francesco Wanderlingh. 					//
//	All rights reserved.										//
//	License: BSD (http://opensource.org/licenses/BSD-3-Clause)	//

/*
 * quadPatrolGRAPH.cpp
 *
 *  Created on: May 6, 2014
 *      Author: francescow
 */


#include "quadcopterRosCtrl.h"
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "quadcopter_ctrl/PGmsg.h"
#include "quadcopter_ctrl/OSmsg.h"
#include <cstring>
#include <sstream>
#include "termColors.h"
#include "PathPlanningAlg.h"
#include "PatrolGRAPH.h"
#include <fstream>
#include <vector>
#include <ctime>

#define INTSTRSIZE ((CHAR_BIT * sizeof(int) - 1) / 3 + 2)

using std::cout;
using std::endl;
using std::vector;

/** Patrol graph msg:
int32 currNode
bool isCurrVisited
int32[2] chosenEdge */
quadcopter_ctrl::PGmsg PGinfo;
quadcopter_ctrl::OSmsg osInfo;
geometry_msgs::PoseStamped quadPos;
geometry_msgs::PoseStamped targetPos;
geometry_msgs::PoseStamped subTarget;

PatrolGRAPH myPG;

int quadPosAcquired = 0;
float zHeight = 0;

///FUNCTIONS
std::string get_selfpath(void);
std::string add_argv(std::string str, char* argvalue);
void updateTarget(ros::Publisher& countPub);
void publishSubTarget(ros::Publisher& posPub);


void quadPosFromVrep(const geometry_msgs::PoseStamped::ConstPtr& pubQuadPose)
{
  quadPos.pose.position.x = pubQuadPose->pose.position.x;
  quadPos.pose.position.y = pubQuadPose->pose.position.y;
  quadPos.pose.position.z = pubQuadPose->pose.position.z;
  quadPosAcquired = 1;
}


void updateCount(const quadcopter_ctrl::PGmsg::ConstPtr& PGinfo){

  myPG.incrCount(PGinfo->currNode, PGinfo->isCurrVisited, PGinfo->chosenEdge);
}


int main(int argc, char **argv)
{

  std::time_t start = time(NULL);
  /// argv[1] contains the ID number of the robot to be controlled (0,1,2...)
  if(argc<2){
    printf("%s** argv[1] is empty! Provide quadcopter # to control! **%s\n", TC_RED, TC_NONE);
    exit(EXIT_FAILURE);
  }

  // In this way each robot flight at a slightly different height
  zHeight =  (float)(strtol(argv[1], NULL, 0)) *0.6 + 5;

  std::ifstream access_matrix;
  //std::string filename = "access_mat_subs";
  std::string filename = "free_mat_3x3";

  std::string folder_path = get_selfpath();
  std::string file_path = folder_path + "/" + filename;

  access_matrix.open( file_path.c_str() );
  if( !access_matrix.is_open() ){
    printf("%sFile not found! (sure is the executable folder?)%s\n", TC_RED, TC_NONE);
    exit(EXIT_FAILURE);
  }

  myPG.initGraph(access_matrix);    //Constructor inputs is (mapToExplore)


  /* The following strings are used to concatenate the topic name to the argument passed to
   * the node (the argv[1]), so to name each node with a different name and send signals to
   *  different topics.
   * (e.g. if argv[1] = 0 the node will be named quadcopterRosCtrl_0, publish to
   *  vrep/targetObjPos_0, etc...)
   */
  std::string nodeName = add_argv("quadPatrolGRAPH", argv[1]);
  std::string targetObjPosName = add_argv("vrep/targetObjPos", argv[1]);
  std::string quadcopPosName = add_argv("vrep/quadcopPos", argv[1]);

  ros::init(argc, argv, nodeName);
  ros::NodeHandle n;

  ros::Publisher targetObjPos_pub = n.advertise<geometry_msgs::PoseStamped>(targetObjPosName, 100);
  ros::Subscriber quadcopPos_sub = n.subscribe(quadcopPosName, 100, quadPosFromVrep);

  ros::Publisher updateCount_pub = n.advertise<quadcopter_ctrl::PGmsg>("updatePGCount", 100);
  ros::Subscriber updateCount_sub = n.subscribe("updatePGCount", 100, updateCount);

  ros::Publisher completed_pub = n.advertise<quadcopter_ctrl::OSmsg>("completedPath", 100);

  ros::Rate loop_rate(DEF_LOOP_RATE); //Loop at DEF_LOOP_RATE

  /*            *** PoseStamped structure: ***
  //            std_msgs/Header header
  //              uint32 seq
  //              time stamp
  //              string frame_id
  //            geometry_msgs/Pose pose
  //              geometry_msgs/Point position
  //                    float64 x
  //                    float64 y
  //                    float64 z
  //              geometry_msgs/Quaternion orientation
  //                    float64 x
  //                    float64 y
  //                    float64 z
  //                    float64 w
   */

  cout << "[" << argv[1] << "] Waiting for start....    " << endl;

  int running = 1;
  int inSubPath = 0;

  double dist = 0;
  /// How much the quadcopter has to be near to the
  /// green sphere (target) before the target moves:
  double treshold = 0.3;
  int loaded = 0;

  //if(clock()) cout << "Clock is available\n";
  //else{cout << "Clock not available\n";}


  while (ros::ok())
  {
    if(myPG.isCompleted() == false ||  inSubPath==1 || ((time(NULL) - start) < (30*60))  ){

      if(quadPosAcquired){
        quadPosAcquired = 0;
        running = 1;

        if (loaded == 0){
          loaded = 1;
          updateTarget(updateCount_pub);
        }

        // Calculating current l^2-norm between target and quadcopter (Euclidean distance)
        dist = fabs( PathPlanningAlg::Distance(&quadPos, &targetPos) );
        //cout << "Distance to target = " << dist << " m" << endl;

        if(inSubPath == 0){
          if( dist > CRITICAL_DIST ){
            inSubPath = 1;
            publishSubTarget(targetObjPos_pub);
            //std::cout << "First subTarget Published!" << std::endl;
          }else if(dist < treshold){

            /// Here we save the index of the node to which we arrived since after executing
            /// the findNext algorithm the "currentNode" variable gets modified, containing
            /// the index of the next vertex to be visited.
            PGinfo.currNode = myPG.getCurrentNode();
            PGinfo.isCurrVisited = myPG.getCurrentType();
            myPG.findNext();
            PGinfo.chosenEdge = myPG.getChosenEdge();
            updateTarget(updateCount_pub);

            //In the following if, "dist" is calculated again since updateTarget changed targetPos
            if( fabs(PathPlanningAlg::Distance(&quadPos, &targetPos)) < CRITICAL_DIST ){
              targetObjPos_pub.publish(targetPos);
              //std::cout << "Target #" << wpIndex << " reached!" << std::endl;
            }
          }
        }else{
          double sub_dist = fabs( (PathPlanningAlg::Distance(&quadPos, &subTarget)) );
          if(dist < treshold){
            inSubPath = 0;
            targetObjPos_pub.publish(targetPos);
          }else if(sub_dist < treshold){
            publishSubTarget(targetObjPos_pub);
            //std::cout << "subTarget Published!" << std::endl;
          }

          /*if(sub_dist < treshold && dist > treshold){
            publishSubTarget(targetObjPos_pub);
            //std::cout << "subTarget Published!" << std::endl;
          }else if (dist < treshold){
            inSubPath = 0;
          }*/
        }
      }else{              /// THIS PART IS EXECUTED IF VREP SIMULATION IS NOT RUNNING

        if (running == 1) {
          printf("%s[%s] ** No incoming vrep/quadcopPos! (waiting...) **%s\n", TC_YELLOW, argv[1], TC_NONE);
        }
        running = 0;
      }

    }else{
      printf("%s[%s] ** Area coverage completed! **%s\n", TC_GREEN, argv[1], TC_NONE);
      printf("Minutes elapsed: %d\n", (int)(time(NULL) - start)/60);

      osInfo.ID = strtol(argv[1], NULL, 0);
      osInfo.numNodes = myPG.getNumFreeNodes();
      osInfo.path = myPG.getFinalPath();
      completed_pub.publish(osInfo);

      int gridSizeX = myPG.getGridSizeX();
      int gridSizeY = myPG.getGridSizeY();
      vector<graphNode> graphNodes = myPG.getGraphNodes();

      ///Dump counts map on file
      char process_id[INTSTRSIZE];
      sprintf(process_id, "%d", (int)getpid() );
      std::string resultsFileName(process_id);
      resultsFileName = folder_path + "/CountMaps/CountMap_" + resultsFileName;

      std::ofstream nodeCountMap;
      nodeCountMap.open(resultsFileName.c_str());
      if(nodeCountMap.is_open() ){
        for(int i=0; i<gridSizeX; i++){
          for(int j=0; j<gridSizeY; j++){
            nodeCountMap << (int)graphNodes.at((i*gridSizeY) + j).nodeCount << " ";
          }
          nodeCountMap << endl;
        }
        nodeCountMap.close();

      }else{ cout << "Error writing counts on file" << endl; }



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


std::string add_argv(std::string str, char* argvalue){

  std::string suffix(argvalue);
  str = str + "_" + suffix;

  return str;

}


void updateTarget(ros::Publisher& countPub){

  ///Position is multiplied by 2 since the access map is sub-sampled
  targetPos.pose.position.x = myPG.getCurrentCoord('x')*2.0 - VREP_X0;     /// The constant VREP_** is added due to the
  targetPos.pose.position.y = myPG.getCurrentCoord('y')*2.0 - VREP_Y0;     /// different origin of the GRF used in Vrep
  targetPos.pose.position.z = zHeight;

  /// Here we inform all the other quadcopters of our choice
  countPub.publish(PGinfo);
}


void publishSubTarget(ros::Publisher& posPub){
  double dSubWP[3];
  PathPlanningAlg::InterpNewPoint(&quadPos, &targetPos, dSubWP);
  subTarget.pose.position.x = quadPos.pose.position.x + dSubWP[X];
  subTarget.pose.position.y = quadPos.pose.position.y + dSubWP[Y];
  subTarget.pose.position.z = quadPos.pose.position.z + dSubWP[Z];
  posPub.publish(subTarget);
}

