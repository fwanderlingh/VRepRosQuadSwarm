//	Copyright (c) 2014, Francesco Wanderlingh. 					//
//	All rights reserved.										//
//	License: BSD (http://opensource.org/licenses/BSD-3-Clause)	//

/*
 * quadNodeCount.cpp
 *
 *  Created on: May 6, 2014
 *      Author: francescow
 */


#include "asctecRosCtrl.h"
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "quadcopter_ctrl/NCmsg.h"
#include "quadcopter_ctrl/OSmsg.h"
#include <cstring>
#include <sstream>
#include "termColors.h"
#include "PathPlanningAlg.h"
#include "NodeCounting.h"
#include <fstream>
#include <vector>


using std::cout;
using std::endl;
using std::vector;

quadcopter_ctrl::NCmsg ncInfo;
quadcopter_ctrl::OSmsg osInfo;
geometry_msgs::PoseStamped quadPos;
geometry_msgs::PoseStamped targetPos;
geometry_msgs::PoseStamped subTarget;

NodeCounting myNodeCount;

int quadPosAcquired = 0;
double zHeight;

///FUNCTIONS
std::string get_selfpath(void);
std::string add_argv(std::string str, char* argvalue);
void updateTarget(ros::Publisher& countPub);


void quadPosFromEthnos(const geometry_msgs::PoseStamped::ConstPtr& pubQuadPose)
{
  quadPos.pose.position.x = pubQuadPose->pose.position.x;
  quadPos.pose.position.y = pubQuadPose->pose.position.y;
  quadPos.pose.position.z = pubQuadPose->pose.position.z;
  quadPosAcquired = 1;
}


void updateCount(const quadcopter_ctrl::NCmsg::ConstPtr& nodeCountInfo){

  myNodeCount.incrCount(nodeCountInfo->node, nodeCountInfo->isVisited);

}



int main(int argc, char **argv)
{
  /// argv[1] contains the ID number of the robot to be controlled (0,1,2...)
  if(argc<4){
    printf("%s** ERROR **\n"
        "argv[1]: Quadcopter # to control\n"
        "argv[2]: Input file\n"
        "argv[3]: zHeight of flight%s\n", TC_RED, TC_NONE);
    exit(EXIT_FAILURE);
  }

  /// In this way each robot flies at a slightly different height
  zHeight = static_cast<double>(strtod(argv[3], NULL));
  cout << "zHeight:" << zHeight << endl;

  std::string filename(argv[2]);
  std::string folder_path = get_selfpath();
  std::string file_path = folder_path + "/" + filename;

  std::ifstream access_matrix;
  access_matrix.open( file_path.c_str() );
  if( !access_matrix.is_open() ){
    printf("%sAccess matrix not found! (sure is the executable folder?)%s\n", TC_RED, TC_NONE);
    exit(EXIT_FAILURE);
  }


  myNodeCount.init_acc(access_matrix);    //Constructor inputs is (mapToExplore)
  //myNodeCount.init_graph_pos(access_matrix, pos_Vec);

  /* The following strings are used to concatenate the topic name to the argument passed to
   * the node (the argv[1]), so to name each node with a different name and send signals to
   *  different topics.
   * (e.g. if argv[1] = 0 the node will be named quadcopterRosCtrl_0, publish to
   *  vrep/targetObjPos_0, etc...)
   */
  std::string nodeName = add_argv("quadNodeCount", argv[1]);
  std::string targetObjPosName = add_argv("targetObjPos", argv[1]);
  std::string quadcopPosName = add_argv("quadcopPos", argv[1]);


  ros::init(argc, argv, nodeName);
  ros::NodeHandle n;

  ros::Publisher targetObjPos_pub = n.advertise<geometry_msgs::PoseStamped>(targetObjPosName, 100);
  ros::Subscriber quadcopPos_sub = n.subscribe(quadcopPosName, 100, quadPosFromEthnos);

  ros::Publisher nodeCount_pub = n.advertise<quadcopter_ctrl::NCmsg>("updateNodeCount", 100);
  ros::Subscriber nodeCount_sub = n.subscribe("updateNodeCount", 100, updateCount);

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

  double dist;
  /// How much the quadcopter has to be near
  /// to the green sphere (target) before the target moves
  double threshold = 1.0;
  int loaded = 0;

  while (ros::ok())
  {

    if(myNodeCount.isCompleted() == false){

      if(quadPosAcquired){
        cout << ".";
        fflush(stdout);
        quadPosAcquired = 0;
        if(running == 0){
          printf("%s[%s] On my way Sir Captain!%s\n", TC_YELLOW, argv[1], TC_NONE);
          running = 1;
        }

        if (loaded == 0){
          loaded = 1;
          //cout << "First Target Published!" << endl;
          updateTarget(nodeCount_pub);
          targetObjPos_pub.publish(targetPos);

        }

        /// Calculating current l^2-norm between target and quadcopter (Euclidean distance)
        dist = fabs( PathPlanningAlg::Distance(&quadPos, &targetPos) );
        //cout << "Distance to target = " << dist << " m" << endl;
        if( dist < threshold){

          cout << "TARGET REACHED!" << endl;
          sleep(5);
          myNodeCount.findNext();
          updateTarget(nodeCount_pub);
          targetObjPos_pub.publish(targetPos);


        }
      }else{ /// THIS PART IS EXECUTED IF VREP SIMULATION IS NOT RUNNING

        if (running == 1) {
          printf("%s[%s] ** No incoming /quadcopPos! (waiting...) **%s\n", TC_YELLOW, argv[1], TC_NONE);
          running = 0;
        }
        //
      }

    }else{
      printf("%s[%s] ** Area coverage completed! **%s\n", TC_GREEN, argv[1], TC_NONE);

      osInfo.ID = strtol(argv[1], NULL, 0);
      osInfo.numNodes = myNodeCount.getNumFreeNodes();
      osInfo.path = myNodeCount.getFinalPath();
      osInfo.fileName = "NC_" + filename;
      completed_pub.publish(osInfo);

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
  targetPos.pose.position.x = myNodeCount.getCurrentCoord('x')*MAP_SCALE;     /// The constant is added due to the
  targetPos.pose.position.y = myNodeCount.getCurrentCoord('y')*MAP_SCALE;     /// different origin of the GRF used in Vrep
  targetPos.pose.position.z = zHeight;

  ncInfo.node = myNodeCount.getCurrentIndex();
  ncInfo.isVisited = myNodeCount.getCurrentType();
  countPub.publish(ncInfo);
  //posPub.publish(targetPos);


}


