//      Copyright (c) 2014, Francesco Wanderlingh.                              //
//      All rights reserved.                                                    //
//      License: BSD (http://opensource.org/licenses/BSD-3-Clause)              //

//‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾//
// *Control of a Quadcopter simulated in VREP*  //
//______________________________________________//

/*
 * kernelNode.cpp
 */


#include "kernelNode.h"
#include "ros/ros.h"
#include "std_msgs/Int64.h"
#include <cstring>
#include <sstream>
#include <unistd.h>
#include <ctime>

///Used API Services
#include "vrep_common/simRosStartSimulation.h"
#include "vrep_common/simRosStopSimulation.h"


using std::cout;
using std::endl;
using std::vector;


std::string get_selfpath(void);

int completed = 0;

void completedSignal(const std_msgs::Int64::ConstPtr& control)
{
  if(control->data == 1) completed++; /// A node controller sends one when it finishes the path

}


int main(int argc, char **argv)
{

  if(argc<3){
    printf("%s** ERROR **\n"
              "argv[1]: Input Map\n"
              "argv[2]: Number of quadcopters%s\n", TC_RED, TC_NONE);
    exit(EXIT_FAILURE);
  }


  std::string filename(argv[1]);
  std::string folder_path = get_selfpath();

  std::string file_path = folder_path + "/Input/" + filename;
  //cout << file_path << endl;
  std::ifstream access_matrix;
  access_matrix.open( file_path.c_str() );
  if( !access_matrix.is_open() ){
    printf("%sAccess matrix not found! (sure is the executable folder?)%s\n", TC_RED, TC_NONE);
    exit(EXIT_FAILURE);
  }

/*
  std::string posV_filename = "posV_" + filename;
  std::string posV_file_path = folder_path + "/Input/PosV/" + posV_filename;
  std::ifstream pos_Vec;
  pos_Vec.open( posV_file_path.c_str() );
  if( !pos_Vec.is_open() ){
    printf("%sPos_Vec matrix not found!%s\n", TC_RED, TC_NONE);
    exit(EXIT_FAILURE);
  }
*/
  std::string save_path = folder_path + "/Results/VRP_FW_" + filename + "_Results";


  int num_robots = static_cast<int>(strtol(argv[2], NULL, 0));
  cout << "\nNum robots: " << num_robots << endl;
  VrpGreedy2 myVrp;
  myVrp.init_acc(access_matrix, num_robots);
  //myVrp.init_graph_pos(access_matrix, pos_Vec, num_robots);

  struct timespec requestStart, requestEnd;

  clock_gettime(CLOCK_REALTIME, &requestStart);
  myVrp.solve();
  clock_gettime(CLOCK_REALTIME, &requestEnd);
  double timeElapsed = ( requestEnd.tv_sec - requestStart.tv_sec )
               + ( requestEnd.tv_nsec - requestStart.tv_nsec ) / 1E9;

  performanceIndexes(myVrp, save_path, timeElapsed*1E3); ///We multiply the time to get it in milliseconds

  savePathsToFile(myVrp, folder_path);



  ros::init(argc, argv, "kernelNode");
  ros::NodeHandle n;

  ros::Publisher ctrlSignal_pub = n.advertise<std_msgs::Int64>("quadCtrlSignal", 100);
  ros::Subscriber pathCompleted_sub = n.subscribe("pathCompleted", 10, completedSignal);

  ros::Rate loop_rate(0.5); //0.5 Hertz

  std_msgs::Int64 controlSignal;
  controlSignal.data = 1;

  int firstrun = 1;


  while (ros::ok())
  {
    if(firstrun){
      ctrlSignal_pub.publish(controlSignal);
      cout << "Start control triggered!" << endl;
      firstrun = 0;

      /** STARRT SIMULATION IN V-REP **/
      ros::ServiceClient client_startSimulation =
           n.serviceClient<vrep_common::simRosStartSimulation>("/vrep/simRosStartSimulation");
      vrep_common::simRosStartSimulation srv_startSimulation;
      client_startSimulation.call(srv_startSimulation);
    }

    if( completed < num_robots){
      ctrlSignal_pub.publish(controlSignal);

    }else{
      /** STOP SIMULATION IN V-REP **/
      ros::ServiceClient client_stopSimulation =
           n.serviceClient<vrep_common::simRosStopSimulation>("/vrep/simRosStopSimulation");
      vrep_common::simRosStopSimulation srv_stopSimulation;
      client_stopSimulation.call(srv_stopSimulation);

      printf("%sAll paths covered!%s\n", TC_GREEN, TC_NONE);
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




