//	Copyright (c) 2014, Francesco Wanderlingh. 			        //
//	All rights reserved.						                //
//	License: BSD (http://opensource.org/licenses/BSD-3-Clause)  //

//‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾//
// *Control of a Quadcopter simulated in VREP*	//
//______________________________________________//

#include "quadcopterRosCtrl.h"
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Int64.h"
#include <cstring>
#include <sstream>
#include "termColors.h"
#include "PathPlanningAlg.h"
#include <fstream>
#include <unistd.h>
#include <vector>


using std::cout;
using std::endl;
using std::vector;

std_msgs::Int64 controlSignal;
std_msgs::Int64 completedPath;
geometry_msgs::PoseStamped quadPos;
geometry_msgs::PoseStamped targetPos;
geometry_msgs::PoseStamped subTarget;

int quadPosAcquired = 0;

std::string get_selfpath(void);
void loadPathFromFile(std::ifstream &file, vector< vector<double> > &robotPath);
void loadPath(vector< vector<double> > &robotPath, char * argvalue);
std::string add_argv(std::string str, char* argvalue);
void updateTarget(int index,  vector< vector<double> > &path, char * argvalue);
void publishSubTarget(ros::Publisher& pub);


void quadPosFromVrep(const geometry_msgs::PoseStamped::ConstPtr& pubQuadPose)
{
  quadPos.pose.position.x = pubQuadPose->pose.position.x;
  quadPos.pose.position.y = pubQuadPose->pose.position.y;
  quadPos.pose.position.z = pubQuadPose->pose.position.z;
  quadPosAcquired = 1;
}


void kernelCtrlSignal(const std_msgs::Int64::ConstPtr& control)
{
  controlSignal.data = control->data;
}


int main(int argc, char **argv)
{
  /// argv[1] contains the ID number of the robot to be controlled (0,1,2...)
  if(argc<2){
    printf("%s** argv[1] is empty! Provide quadcopter # to control! **%s\n", TC_RED, TC_NONE);
    exit(EXIT_FAILURE);
  }

  vector< vector<double> > robotPathVec;

  /* The following strings are used to concatenate the topic name to the argument passed to
   * the node (the argv[1]), so to name each node with a different name and send signals to
   *  different topics.
   * (e.g. if argv[1] = 0 the node will be named quadcopterRosCtrl_0, publish to
   *  vrep/targetObjPos_0, etc...)
   */
  std::string nodeName = add_argv("quadcopterRosCtrl", argv[1]);
  std::string targetObjPosName = add_argv("vrep/targetObjPos", argv[1]);
  std::string quadcopPosName = add_argv("vrep/quadcopPos", argv[1]);

  ros::init(argc, argv, nodeName);
  ros::NodeHandle n;

  ros::Publisher targetObjPos_pub = n.advertise<geometry_msgs::PoseStamped>(targetObjPosName, 100);
  ros::Publisher completedPath_pub = n.advertise<std_msgs::Int64>("pathCompleted", 10);
  ros::Subscriber quadcopPos_sub = n.subscribe(quadcopPosName, 100, quadPosFromVrep);
  ros::Subscriber ctrlSignal_sub = n.subscribe("quadCtrlSignal", 10, kernelCtrlSignal);

  ros::Rate loop_rate(DEF_LOOP_RATE); //Loop at DEF_LOOP_RATE


  double dist;
  /// How much the quadcopter has to be near to the green sphere before the target moves
  double threshold = 0.3;

  int wpIndex = 0;     // Waypoint index, used to navigate through the robotPath vector

  double subDist = 0;
  int inSubPath = 0;

  int running = 1;

  /*		*** PoseStamped structure: ***
  //		std_msgs/Header header
  //		  uint32 seq
  //		  time stamp
  //		  string frame_id
  //		geometry_msgs/Pose pose
  //		  geometry_msgs/Point position
  //			double64 x
  //			double64 y
  //			double64 z
  //		  geometry_msgs/Quaternion orientation
  //			double64 x
  //			double64 y
  //			double64 z
  //			double64 w
  */

  cout << "[" << argv[1] << "] Waiting for start....    " << endl;

  controlSignal.data = 0;
  completedPath.data = 0;
  int loaded = 0;

  while (ros::ok())
  {
    if( ((controlSignal.data != 0) && (completedPath.data == 0)) || (inSubPath == 1) ){

      if(quadPosAcquired){
        quadPosAcquired = 0;
        if(running == 0){
          printf("%s[%s] On my way Sir Captain!%s\n", TC_YELLOW, argv[1], TC_NONE);
          running = 1;
        }
        running = 1;

        if (loaded == 0){
          loadPath(robotPathVec, argv[1]);
          loaded = 1;
          wpIndex = 0;
          updateTarget(wpIndex, robotPathVec, argv[1]);
        }

        // Calculating current l^2-norm between target and quadcopter (Euclidean distance)
        dist = fabs( PathPlanningAlg::Distance(&quadPos, &targetPos) );
        //std::cout << "Distance to target = " << dist << " m" << std::endl;
        if(inSubPath == 0){
          if( dist > CRITICAL_DIST ){
            inSubPath = 1;
            publishSubTarget(targetObjPos_pub);
            //std::cout << "First subTarget Published!" << std::endl;
          }else if(dist < threshold){

            ++wpIndex;
            if(wpIndex == (int)robotPathVec.size() ){
              completedPath.data = 1;
            }
            updateTarget(wpIndex, robotPathVec, argv[1]);

            if( fabs(PathPlanningAlg::Distance(&quadPos, &targetPos)) < CRITICAL_DIST ){
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
      }else{              /// THIS PART IS EXECUTED IF VREP IS NOT RUNNING
        if (running == 1) {
          printf("%s[%s] ** No incoming vrep/quadcopPos! (waiting...) **%s\n", TC_YELLOW, argv[1], TC_NONE);
        }
        running = 0;              //
      }
    }

    if(completedPath.data == 1){
      completedPath_pub.publish(completedPath);
      completedPath.data = 2;   /// completedPath = 2 defines the end state, after completing the path
      printf("[%s] ** Path %s covered! **\n", argv[1], argv[1]);
      ros::shutdown();
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}


void loadPathFromFile(std::ifstream &file, vector< vector<double> > &robotPath, char * argvalue){

  robotPath.push_back( vector<double>() );

  int pathVecIndex = 0;
  if( file.is_open() ) {
    double val;
    while( file >> val ){
      /// Here x and y coordinates are inserted in the path vector
      robotPath[pathVecIndex].push_back( val );

      if(file.peek() == '\n'){
        /// Here the z coordinate is inserted and it's value depends on the robot number
        /// so that every quadcopter flies at a different height
        robotPath[pathVecIndex].push_back( (double)(*argvalue - '0') *0.75 + 5 );
        robotPath.push_back( vector<double>() );
        ++pathVecIndex;
      }
    }
    robotPath.pop_back();
    file.close();
  }else{
    printf("%s[%s] ** ERROR reading path file! **%s\n", TC_RED, argvalue, TC_NONE);
  }

/*
  std::cout << "The contents of Path are:" << endl;
  for (std::vector< vector<double> >::iterator itr = robotPath.begin(); itr != robotPath.end(); ++itr){
    for (std::vector<double>::iterator itc = itr->begin(); itc != itr->end(); ++itc){
        std::cout << *itc << ' ';
    }
    std::cout << '\n';
  }
*/

}


void loadPath(vector< vector<double> > &robotPath, char * argvalue){

    std::ifstream robotPathFile;
    std::string filename = "path";

    std::string folder_path = get_selfpath();
    std::string file_path = folder_path + "/" + add_argv(filename, argvalue);

    robotPathFile.open( file_path.c_str() );


    loadPathFromFile(robotPathFile, robotPath, argvalue);
/*
    std::cout << "The contents of Path are:" << endl;
    for (std::vector< vector<double> >::iterator itr = robotPath.begin(); itr != robotPath.end(); ++itr){
      for (std::vector<double>::iterator itc = itr->begin(); itc != itr->end(); ++itc){
          std::cout << *itc << ' ';
      }
      std::cout << '\n';
    }
    */
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


void updateTarget(int index, vector< vector<double> > &path, char * argvalue){
  if(index < (int)path.size()){
  targetPos.pose.position.x = path[index][X]*MAP_SCALE - VREP_X0;     /// The constant is added due to the
  targetPos.pose.position.y = path[index][Y]*MAP_SCALE - VREP_Y0;     /// different origin of the GRF used in Vrep
  targetPos.pose.position.z = path[index][Z];
  }else{
    printf("ERROR: Path index out of range");
  }
  //std::cout << "[" << argvalue << "] New target: "
  //          << targetPos.pose.position.x << "  " << targetPos.pose.position.y << "  " << targetPos.pose.position.z << std::endl;

}


void publishSubTarget(ros::Publisher& pub){

  double dSubWP[3];
  PathPlanningAlg::InterpNewPoint(&quadPos, &targetPos, dSubWP);
  subTarget.pose.position.x = quadPos.pose.position.x + dSubWP[X];
  subTarget.pose.position.y = quadPos.pose.position.y + dSubWP[Y];
  subTarget.pose.position.z = quadPos.pose.position.z + dSubWP[Z];
  pub.publish(subTarget);            // PUBLISH NEW SUBTARGET POSITION
}

