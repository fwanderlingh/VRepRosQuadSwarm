//      Copyright (c) 2014, Francesco Wanderlingh.                              //
//      All rights reserved.                                                    //
//      License: BSD (http://opensource.org/licenses/BSD-3-Clause)              //

//‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾//
// *Control of a Quadcopter simulated in VREP*  //
//______________________________________________//

/*
 * fatherNode.cpp
 */


#include "ros/ros.h"
#include "std_msgs/Int64.h"
#include <cstring>
#include <sstream>
#include "termColors.h"
#include "VrpGreedyFW.h"
#include "VrpGreedyAstar.h"
#include <fstream>
#include <unistd.h>

#define INTSTRSIZE ((CHAR_BIT * sizeof(int) - 1) / 3 + 2)

using std::cout;
using std::endl;
using std::vector;


std::string get_selfpath(void);
void savePathsToFile(std::vector< vector<int> > &pathVec, vector<graphNode> &graph, std::string folder_save_path);

int completed = 0;

void completedSignal(const std_msgs::Int64::ConstPtr& control)
{
	if(control->data == 1) completed++; /// A node controller sends one when it finishes the path

}


int main(int argc, char **argv)
{

	/// argv[1] contains the size of the swarm, e.g. the number of robots to be controlled
	if(argc<2){
		printf("%s** argv[1] is empty! Provide size of swarm! **%s\n", TC_RED, TC_NONE);
		exit(EXIT_FAILURE);
	}
	int num_robots = strtol(argv[1], NULL, 0);

	std::string filename = "free_mat_8x9";
	//std::string filename = "access_mat_subs";
	//std::string filename = "little_matrix";
	std::string folder_path = get_selfpath();
	std::string acc_matrix_path = folder_path + "/" + filename;

	/// Constructor inputs are (mapToExplore, numOfAgents) ///
	//VrpGreedy myVrp(acc_matrix_path, num_robots);
	VrpGreedyAstar myVrp(acc_matrix_path, num_robots);

	myVrp.solve();
	//myVrpAstar.solve();

	vector< vector<int> > pathVec;
	vector<graphNode> graph;
	myVrp.copyPathsTo(pathVec);
	myVrp.copyGraphTo(graph);

	savePathsToFile(pathVec, graph, folder_path);

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
		}

		if( completed < num_robots){
			ctrlSignal_pub.publish(controlSignal);

		}else{
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

void savePathsToFile(std::vector< vector<int> > &Paths,  vector<graphNode> &graphNodes, std::string folder_save_path){
	///  SAVE PATHS TO FILE  ///


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



