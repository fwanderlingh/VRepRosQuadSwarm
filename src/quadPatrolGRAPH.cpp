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
#include <string>
#include <sstream>
#include "termColors.h"
#include "PathPlanningAlg.h"
#include "PatrolGRAPH.h"
#include <fstream>
#include <vector>
#include <ctime>
#include "Utils.h"

//#define INTSTRSIZE ((CHAR_BIT * sizeof(int) - 1) / 3 + 2)

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

double zHeight = 0;
double MAP_SCALE, OFS_X, OFS_Y, WP_STEP, CRIT_DIST, threshold;
int quadPosAcquired = 0;

///FUNCTIONS
std::string get_selfpath(void);
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
  if(argc<6){
    printf("%s** ERROR **\n"
        "argv[1]: Quadcopter # to control\n"
        "argv[2]: Input file\n"
        "argv[3]: zHeight of flight\n"
        "argv[4]: Control Mode ('sim' or 'asctec')\n"
        "argv[5]: STARTNODE, index of first node%s\n"
        "argv[6]: minVisit%s\n",
        "argv[7]: optimized (0=false, 1=true) ", TC_RED, TC_NONE);
    exit(EXIT_FAILURE);
  }

  /// In this way each robot flies at a different height
  zHeight = static_cast<double>(strtod(argv[3], NULL));
  printf("%s[%s] My zHeight is: %f%s\n", TC_YELLOW, argv[1], zHeight, TC_NONE);

  int startNode = strtol(argv[5], NULL, 0);


  std::string filename(argv[2]);
  std::string folder_path = get_selfpath();
  std::string file_path = folder_path + "/Input/Grids/" + filename;

  std::ifstream access_matrix;
  access_matrix.open( file_path.c_str() );
  if( !access_matrix.is_open() ){
    printf("%sAccess matrix not found! (sure is the executable folder?)%s\n", TC_RED, TC_NONE);
    exit(EXIT_FAILURE);
  }

 /* std::ifstream PTM_matrix;
  std::string PTM_filename = "PTM_" + filename + ".txt";
  std::string PTM_file_path = folder_path + "/Input/PTMs/" + PTM_filename;
  PTM_matrix.open( PTM_file_path.c_str() );
  if( !PTM_matrix.is_open() ){
    printf("%sPTM matrix not found!%s\n", TC_RED, TC_NONE);
    exit(EXIT_FAILURE);
  }


  std::string posV_filename = "posV_" + filename;
  std::string posV_file_path = folder_path + "/Input/PosV/" + posV_filename;
  std::ifstream pos_Vec;
  pos_Vec.open( posV_file_path.c_str() );
  if( !pos_Vec.is_open() ){
    printf("%sPos_Vec matrix not found!%s\n", TC_RED, TC_NONE);
    exit(EXIT_FAILURE);
  }*/

  int min_visit = strtol(argv[6], NULL, 0);
  bool optimized = static_cast<bool>(strtol(argv[7], NULL, 0));
  std::string type;

  if(optimized){
    /// Run with offline optimisation

    type = "PG_";
    //myPG.init_acc_ptm(access_matrix, PTM_matrix, startNode, min_visit);
    //myPG.init_graph_pos_ptm(access_matrix, pos_Vec, PTM_matrix, startNode, min_visit);
  }else{
    /// Run withOut offline optimisation

    type = "EC_";
    myPG.init_acc(access_matrix, startNode, min_visit);
    //myPG.init_graph_pos(access_matrix, pos_Vec, startNode, min_visit);
  }

  int minVisit = strtol(argv[6], NULL, 0);

  std::string controlMode(argv[4]);
  printf("%s[%s] Control Mode: %s%s\n", TC_YELLOW, argv[1], controlMode.c_str(), TC_NONE);
  if(PathPlanningAlg::LoadParams(controlMode, MAP_SCALE, OFS_X, OFS_Y, WP_STEP, CRIT_DIST, threshold)){
    printf("%s** INCORRECT CONTROL MODE **%s\n", TC_RED, TC_NONE);
    exit(EXIT_FAILURE);
  }

  double dist;
  int running = 1;
  int inSubPath = 0;
  bool loaded = 0;

  /// ROS NETWORK CONFIGURATION ///
  /* The following strings are used to concatenate the topic name to the argument passed to
   * the node (the argv[1]), so to name each node with a different name and send signals to
   *  different topics.
   * (e.g. if argv[1] = 0 the node will be named quadcopterRosCtrl_0, publish to
   *  vrep/targetObjPos_0, etc...)
   */
  std::string nodeName = Utils::add_argv("quadPatrolGRAPH", argv[1]);
  std::string targetObjPosName = Utils::add_argv("targetObjPos", argv[1]);
  std::string quadcopPosName = Utils::add_argv("quadcopPos", argv[1]);

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


  // || ((time(NULL) - start) < (60 * #))  <-- # minutes

  while (ros::ok())
  {
    if(myPG.isCompleted() == false || inSubPath == 1){

      if(quadPosAcquired){
        quadPosAcquired = 0;
        //cout << "."; cout.flush();
        if(running == 0){
          printf("%s[%s] On my way Sir Captain!%s\n", TC_YELLOW, argv[1], TC_NONE);
          running = 1;
        }

        if (loaded == 0){
          loaded = 1;
          targetPos.pose.position.x = myPG.getCurrentCoord('x')*MAP_SCALE - OFS_X;     /// The constant VREP_** is added due to the
          targetPos.pose.position.y = myPG.getCurrentCoord('y')*MAP_SCALE - OFS_Y;     /// different origin of the GRF used in Vrep
          targetPos.pose.position.z = zHeight;
          if (controlMode == "asctec") {
            targetObjPos_pub.publish(targetPos);
            printf("%s[%s] 1st TARGET PUBLISHED!%s\n", TC_YELLOW, argv[1], TC_NONE);
          }
        }

        // Calculating current l^2-norm between target and quadcopter (Euclidean distance)
        dist = fabs( PathPlanningAlg::Distance(&quadPos, &targetPos) );
        //cout << "Distance to target = " << dist << " m" << endl;

        if(controlMode == "asctec"){
          if(dist < threshold){
            cout << "TARGET REACHED!" << endl;
            //sleep(5);
            PGinfo.currNode = myPG.getCurrentNode();
            PGinfo.isCurrVisited = myPG.getCurrentType();
            myPG.findNext();
            PGinfo.chosenEdge = myPG.getChosenEdge();
            updateTarget(updateCount_pub);
            targetObjPos_pub.publish(targetPos);
          }
        }else{
          if(inSubPath == 0){
            if( dist > CRIT_DIST ){
              inSubPath = 1;
              publishSubTarget(targetObjPos_pub);
              //std::cout << "First subTarget Published!" << std::endl;
            }else if(dist < threshold){

              /// Here we save the index of the node to which we arrived since after executing
              /// the findNext algorithm the "currentNode" variable gets modified, containing
              /// the index of the next vertex to be visited.
              PGinfo.currNode = myPG.getCurrentNode();
              PGinfo.isCurrVisited = myPG.getCurrentType();
              myPG.findNext();
              PGinfo.chosenEdge = myPG.getChosenEdge();
              updateTarget(updateCount_pub);

              //In the following if, "dist" is calculated again since updateTarget changed targetPos
              if( fabs(PathPlanningAlg::Distance(&quadPos, &targetPos)) < CRIT_DIST ){
                targetObjPos_pub.publish(targetPos);
                //std::cout << "Target #" << wpIndex << " reached!" << std::endl;
              }
            }
          }else{
            double sub_dist = fabs( (PathPlanningAlg::Distance(&quadPos, &subTarget)) );
            if(dist < threshold){
              inSubPath = 0;
              targetObjPos_pub.publish(targetPos);
            }else if(sub_dist < threshold){
              publishSubTarget(targetObjPos_pub);
              //std::cout << "subTarget Published!" << std::endl;
            }
          }
        }
      }else{              /// THIS PART IS EXECUTED IF VREP SIMULATION IS NOT RUNNING

        if (running == 1) {
          printf("%s[%s] ** No incoming vrep/quadcopPos! (waiting...) **%s\n", TC_YELLOW, argv[1], TC_NONE);
        }
        running = 0;
      }

    }else{
      printf("%s[%s] ** Area coverage completed! **%s\n", TC_GREEN, argv[1], TC_NONE);
      //printf("Minutes elapsed: %d\n", (int)(time(NULL) - start)/60);

      osInfo.ID = strtol(argv[1], NULL, 0);
      osInfo.numNodes = myPG.getNumFreeNodes();
      osInfo.path = myPG.getFinalPath();
      //filename.resize(filename.size()-2); /// XXX REMEBER TO DELETE THIS LINE FIXME
      osInfo.fileName = type + filename;
      completed_pub.publish(osInfo);

/*
      ///Dump counts map on file
      int gridSizeX = myPG.getGridSizeX();
      int gridSizeY = myPG.getGridSizeY();
      vector<graphNode> graphNodes = myPG.getGraphNodes();

      //std::ostringstream process_id;
      //process_id << (int)getpid();
      std::string resultsName = folder_path + "/CountMaps/" + type + Utils::add_argv("CountMap", argv[1]) +
          "_" + filename + "_" + Utils::GetCurrentDateFormatted();

      std::ofstream nodeCountMap;
      nodeCountMap.open(resultsName.c_str());
      if(nodeCountMap.is_open() ){
        for(int i=0; i<gridSizeX; i++){
          for(int j=0; j<gridSizeY; j++){
            nodeCountMap << (int)graphNodes.at((i*gridSizeY) + j).nodeCount << " ";
          }
          nodeCountMap << endl;
        }
        nodeCountMap.close();

      }else{ cout << "Error writing Count Map on file" << endl; }
*/
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



void updateTarget(ros::Publisher& countPub){

  ///Position is multiplied by 2 since the access map is sub-sampled
  targetPos.pose.position.x = myPG.getCurrentCoord('x')*MAP_SCALE - OFS_X;     /// The constant VREP_** is added due to the
  targetPos.pose.position.y = myPG.getCurrentCoord('y')*MAP_SCALE - OFS_Y;     /// different origin of the GRF used in Vrep
  targetPos.pose.position.z = zHeight;

  /// Here we inform all the other quadcopters of our choice
  countPub.publish(PGinfo);
}


void publishSubTarget(ros::Publisher& posPub){
  double dSubWP[3] = {WP_STEP, WP_STEP, WP_STEP};
  PathPlanningAlg::InterpNewPoint(&quadPos, &targetPos, dSubWP);
  subTarget.pose.position.x = quadPos.pose.position.x + dSubWP[X];
  subTarget.pose.position.y = quadPos.pose.position.y + dSubWP[Y];
  subTarget.pose.position.z = quadPos.pose.position.z + dSubWP[Z];
  posPub.publish(subTarget);
}
