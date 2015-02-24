//	Copyright (c) 2015, Francesco Wanderlingh.                 	//
//	License: BSD (http://opensource.org/licenses/BSD-3-Clause)	//

/*
 * quadNodeCountLoss.cpp
 *
 *  Created on: Feb 22, 2015
 *      Author: francescow
 */


#include "quadcopterRosCtrl.h"
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "quadcopter_ctrl/NCmsg.h"
#include "quadcopter_ctrl/OSmsg.h"
#include <string>
#include <sstream>
#include "termColors.h"
#include "PathPlanningAlg.h"
#include "NodeCountingLoss.h"
#include <fstream>
#include <vector>
#include "Utils.h"


using std::cout;
using std::endl;
using std::vector;

int quadID(0);

quadcopter_ctrl::NCmsg ncInfo;
quadcopter_ctrl::OSmsg osInfo;
geometry_msgs::PoseStamped quadPos;
geometry_msgs::PoseStamped targetPos;
geometry_msgs::PoseStamped subTarget;

NodeCountingLoss myNodeCount;

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


void updateCount(const quadcopter_ctrl::NCmsg::ConstPtr& nodeCountInfo){

  /** We update our count only if we weren't the one publishing the message **/
  if(nodeCountInfo->robotId != quadID){
    myNodeCount.incrCount(nodeCountInfo->node, nodeCountInfo->isVisited);
  }

}



int main(int argc, char **argv)
{
  /// argv[1] contains the ID number of the robot to be controlled (0,1,2...)
  quadID = strtol(argv[1], NULL, 0);
  ncInfo.robotId = quadID;

  if(argc<7){
    printf("%s** ERROR **\n"
        "argv[1]: Quadcopter # to control\n"
        "argv[2]: Input file\n"
        "argv[3]: zHeight of flight\n"
        "argv[4]: Control Mode ('sim' or 'asctec')\n"
        "argv[5]: STARTNODE, index of first node\n"
        "argv[6]: Min #visits for each node (1=\"simple coverage\")%s\n", TC_RED, TC_NONE);
    exit(EXIT_FAILURE);
  }

  std::string type = "NC_";

  //std::string inputMapType(argv[7]);


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

  std::string posV_filename = "posV_" + filename;
  std::string posV_file_path = folder_path + "/Input/PosV/" + posV_filename;
  std::ifstream pos_Vec;

//  if(inputMapType == "graph"){
    pos_Vec.open( posV_file_path.c_str() );
    if( !pos_Vec.is_open() ){
      printf("%sPos_Vec matrix not found!%s\n", TC_RED, TC_NONE);
      exit(EXIT_FAILURE);
    }
//  }


  int min_visit = strtol(argv[6], NULL, 0);
  cout << "min_visit: " << min_visit << endl;

  //myNodeCount.init_acc(access_matrix, startNode, min_visit);    //Constructor inputs is (mapToExplore)
  myNodeCount.init_graph_pos(access_matrix, pos_Vec, startNode, min_visit);



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
  std::string nodeName = Utils::add_argv("quadNodeCount", argv[1]);
  std::string targetObjPosName = Utils::add_argv("targetObjPos", argv[1]);
  std::string quadcopPosName = Utils::add_argv("quadcopPos", argv[1]);


  ros::init(argc, argv, nodeName);
  ros::NodeHandle n;

  ros::Publisher targetObjPos_pub = n.advertise<geometry_msgs::PoseStamped>(targetObjPosName, 100);
  ros::Subscriber quadcopPos_sub = n.subscribe(quadcopPosName, 100, quadPosFromVrep);

  ros::Publisher updateCount_pub = n.advertise<quadcopter_ctrl::NCmsg>("updateNodeCount", 100);
  ros::Subscriber updateCount_sub = n.subscribe("updateNodeCount", 100, updateCount);

  ros::Publisher completed_pub = n.advertise<quadcopter_ctrl::OSmsg>("completedPath", 100);

  ros::Rate loop_rate(DEF_LOOP_RATE); //Loop at DEF_LOOP_RATE


  while (ros::ok())
  {
    if(myNodeCount.isCompleted() == false || inSubPath == 1){

      if(quadPosAcquired){
        quadPosAcquired = 0;
        //printf("%s[%s]. %s", TC_YELLOW, argv[1], TC_NONE);
        if(running == 0){
          printf("%s[%s] On my way Sir Captain!%s\n", TC_YELLOW, argv[1], TC_NONE);
          running = 1;
        }

        if (loaded == 0){
          loaded = 1;
          updateTarget(updateCount_pub);
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
            printf("\n%s[%s] TARGET REACHED!%s\n", TC_GREEN, argv[1], TC_NONE);
            //sleep(5);
            myNodeCount.findNext();
            updateTarget(updateCount_pub);
            targetObjPos_pub.publish(targetPos);
            printf("\n%s[%s] NEXT TARGET SENT!%s\n", TC_GREEN, argv[1], TC_NONE);
          }
        }else{
          if(inSubPath == 0){
            if( dist > CRIT_DIST ){
              inSubPath = 1;
              publishSubTarget(targetObjPos_pub);
              //std::cout << "First subTarget Published!" << std::endl;
            }else if(dist < threshold){

              //cout << "Finding next node:" << endl;
              myNodeCount.findNext();
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
          printf("%s[%s] ** No incoming /quadcopPos! (waiting...) **%s\n", TC_YELLOW, argv[1], TC_NONE);
        }
        running = 0;              //
      }

    }else{
      printf("\n%s[%s] ** Area coverage completed! **%s\n", TC_GREEN, argv[1], TC_NONE);

      osInfo.ID = strtol(argv[1], NULL, 0);
      osInfo.numNodes = myNodeCount.getNumFreeNodes();
      osInfo.path = myNodeCount.getFinalPath();
      filename.resize(filename.size()-2); /// XXX REMEBER TO DELETE THIS LINE FIXME
      osInfo.fileName = type + filename;
      completed_pub.publish(osInfo);
/*
      ///Dump counts map on file (for now only valid for occGrid)
      int gridSizeX = myNodeCount.getGridSizeX();
      int gridSizeY = myNodeCount.getGridSizeY();
      vector<graphNode> graphNodes = myNodeCount.getGraphNodes();

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
    std::string path(buff);                              // Here the executable name is still in buff
    std::string::size_type t = path.find_last_of("/");   // Here we find the last "/"
    path = path.substr(0,t);                             // and remove the rest (exe name)
    return path;
  } else {
    printf("Cannot determine file path!\n");
  }
}



void updateTarget(ros::Publisher& countPub){
  targetPos.pose.position.x = myNodeCount.getCurrentCoord('x')*MAP_SCALE - OFS_X;     /// The constant is added due to the
  targetPos.pose.position.y = myNodeCount.getCurrentCoord('y')*MAP_SCALE - OFS_Y;     /// different origin of the GRF used in Vrep
  targetPos.pose.position.z = zHeight;

  ncInfo.node = myNodeCount.getCurrentIndex();
  ncInfo.isVisited = myNodeCount.getCurrentType();
  countPub.publish(ncInfo);

}


void publishSubTarget(ros::Publisher& posPub){
  double dSubWP[3] = {WP_STEP, WP_STEP, WP_STEP};
  PathPlanningAlg::InterpNewPoint(&quadPos, &targetPos, dSubWP);
  subTarget.pose.position.x = quadPos.pose.position.x + dSubWP[X];
  subTarget.pose.position.y = quadPos.pose.position.y + dSubWP[Y];
  subTarget.pose.position.z = quadPos.pose.position.z + dSubWP[Z];
  posPub.publish(subTarget);
}

