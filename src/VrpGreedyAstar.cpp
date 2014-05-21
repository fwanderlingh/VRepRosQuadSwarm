//	Copyright (c) 2014, Francesco Wanderlingh. 					//
//	All rights reserved.										//
//	License: BSD (http://opensource.org/licenses/BSD-3-Clause)	//

/*
 * VrpGreedyAstar.cpp
 *
 *  Created on: Apr 29, 2014
 *      Author: francescow
 */

#include "VrpGreedyAstar.h"
#include <iostream>
#include <cfloat>
#include <cassert>
#include <cmath>
#include <ctime>


#define STARTNODE 5
#define MAX_FOV 2
#define SQRT2 1.4143 /// Ceiled to avoid failure in check condition in "solve()"
/// due to small decimal errors.

#define DEF_GRID_X 4
#define DEF_GRID_Y 4
#define DEF_NUM_ROB 3

using std::cout;
using std::endl;
using std::vector;



VrpGreedyAstar::VrpGreedyAstar() :
    		gridSizeX(DEF_GRID_X),
    		gridSizeY(DEF_GRID_Y),
    		numAgents(DEF_NUM_ROB),  // Default Initialisation
    		minDist(FLT_MAX),
    		v(0),
    		neighbourAvailable(false)
{
	cout << "Default configuration:" << endl;
	cout << "- Default grid is empty" << endl;
	cout << "- " << numAgents << " Robots" << endl;

	access_vec.resize(gridSizeX*gridSizeY,0);


	// Parameters initialisation
	deltaBest = FLT_MAX;
	deltavip = FLT_MAX;
	bigL = -FLT_MAX;
	bigLTent = -FLT_MAX;
	liMin = FLT_MAX;

}


VrpGreedyAstar::VrpGreedyAstar(std::string acc_matrix_path) :
    		numAgents(DEF_NUM_ROB),
    		minDist(FLT_MAX),
    		v(0),
    		neighbourAvailable(false)
{
	loadMatrixFile(acc_matrix_path);
}


VrpGreedyAstar::VrpGreedyAstar(std::string acc_matrix_path, int agents) :
    		minDist(FLT_MAX),
    		v(0),
    		neighbourAvailable(false)
{
	loadMatrixFile(acc_matrix_path);
	numAgents = agents;
}


VrpGreedyAstar::~VrpGreedyAstar()
{
	// TODO Auto-generated destructor stub
}


float VrpGreedyAstar::pathLength(vector<int> &path){


	float length = FLT_MIN;
	for(vector<graphNode>::size_type i = 0; i < (path.size() - 1); i++){
		length = length + sqrt(pow((graphNodes[path[i]].posx - graphNodes[path[i+1]].posx),2) +
				pow((graphNodes[path[i]].posy - graphNodes[path[i+1]].posy),2) +
				pow((graphNodes[path[i]].posz - graphNodes[path[i+1]].posz),2) );
	}
	return length;
}


void VrpGreedyAstar::loadMatrixFile(std::string acc_matrix_path){
	/**
	 * Here gridSizeX and gridSizeY are deducted from the size of the input matrix file
	 */

	std::ifstream access_mat;
	access_mat.open( acc_matrix_path.c_str() );

	if( access_mat.is_open() ) {
		int val;
		int num_nl = 0;
		while( access_mat >> val ){
			if(access_mat.peek() == '\n') num_nl++;
			access_vec.push_back( val );
		}


		gridSizeX = num_nl;
		gridSizeY = access_vec.size()/num_nl;

		access_mat.close();

	}
	else{ cout << "Error reading file!" << endl; }



}


void VrpGreedyAstar::init(){
	/** Fill the graph and the A* grid with the information from the
	 * access matrix (saved in access_vec for more consistent indexing)
	 * and initialize all the variables.
	 */

	srand(time(NULL));    ///Here just because of the Astar class

	cout << "Matrix size is: " << gridSizeX << "x" << gridSizeY << endl;

	graphNodes.resize(gridSizeX*gridSizeY);
	unvisitedNodes.reserve( graphNodes.size() );


	/// Graph initialisation - to every node is assigned a position
	for(int i=0; i<gridSizeX; i++){
		for(int j=0; j<gridSizeY; j++){
			graphNodes.at((i*gridSizeY) + j).setPos((float)i*2, (float)j*2);
			graphNodes.at((i*gridSizeY) + j).occupied = access_vec.at((i*gridSizeY) + j);
			//cout << (int)graphNodes.at((i*gridSizeY) + j).occupied << " ";
			if (graphNodes.at((i*gridSizeY)+ j).occupied == 0 && ((i*gridSizeY)+ j) != STARTNODE){
				unvisitedNodes.push_back((i*gridSizeY) + j); //Adding the free nodes to the list of unvisited
			}
		}
		//cout << endl;
	}


	minDist = (dist(graphNodes.at(0), graphNodes.at(1)) + FLT_MIN)*SQRT2;
	cout << "minDist=" << minDist << endl;

	// Path initialisation
	path.push_back(STARTNODE);       // Every path initially is just 2 nodes: Start + End(=start)
	path.push_back(STARTNODE);

	for(int i = 0; i < numAgents; i++){
		Paths.push_back(path); // Define a path for every agent
	}

	Paths.reserve( graphNodes.size() * numAgents);

	createCycles();

}


void VrpGreedyAstar::createCycles(){

	/** /////////////////// Create initial cycles /////////////////////////////////
	 * To understand the need for the initial cycle: consider that the initial path
	 * is (0,0), the next best path for the Greedy solver cannot be (0,1,0) which has
	 * length=2, but (0,0,1) which has length=1. So it will not produce a circular
	 * solution. By inserting the first element in the middle (see **), we solve this problem.
	 */

	int insertIndex;

	for (itr = Paths.begin(); itr != Paths.end(); ++itr){            // On every path i

		// Delta increments initialisation
		deltaBest = FLT_MAX;
		deltavip = FLT_MAX;
		liMin = FLT_MAX;
		itc = (itr->begin()+1);        // Between Start and End
		for (v = 0; v < unvisitedNodes.size(); v++ ){   // For every Node v
			insertIndex = (int)(itc - itr->begin());        // This is necessary to avoid modifying the current path
			// vector (which is NOT allowed since we're iterating
			// inside it), and keep track of the insertion index.
			pathTentative = *itr;
			vector<int>::iterator iTent = pathTentative.begin() + insertIndex;
			float tentPathLength;
			pathTentative.insert(iTent, unvisitedNodes[v]);
			tentPathLength = pathLength(pathTentative);

			if(tentPathLength < liMin){
				liMin = tentPathLength;
				choice.set_vip(v, itr, itc);
			}

		} // END V
		choice.i->insert(choice.p, unvisitedNodes[choice.v]);
		vector<int>::iterator insertedNode = unvisitedNodes.begin() + choice.v;
		unvisitedNodes.erase(insertedNode);
	} // END I

}


void VrpGreedyAstar::solve(){

	init();
	pathTentative.reserve(gridSizeX*gridSizeY);

	/// Here we adapt out access_vec to the format of the Astar class:
	/// a wall is represented with a "9" instead of a "1" as we do.
	/// For more info see the Astar class "NOTES ON THE INPUT MAP".
	std::vector<int> astar_vec(access_vec);
	std::transform(astar_vec.begin(), astar_vec.end(),
			astar_vec.begin(), std::bind1st(std::multiplies<int>(),9));

	Astar astar(astar_vec, gridSizeX, gridSizeY);


	astar.printMap();
	int start1, start2, target;


	while(unvisitedNodes.size() > 0){

		/*
    std::cout << "The contents of Paths are:" << endl;
    for (itr = Paths.begin(); itr != Paths.end(); ++itr){
      cout << "#" << itr - Paths.begin() << ": ";
      for (itc = itr->begin(); itc != itr->end(); ++itc){
          std::cout << *itc << ' ';
      }
      std::cout << '\n';
    }
		 */
		// Delta increments initialisation
		deltaBest = FLT_MAX;
		deltavip = FLT_MAX;
		bigL = -FLT_MAX;
		bigLTent = -FLT_MAX;
		liMin = FLT_MAX;

		/// Initialise bigL as the MAX path length among all the current paths
		for(it = Paths.begin(); it!=Paths.end(); ++it){
			if( pathLength(*it) > bigL ) bigL = pathLength(*it);
		}

		///*** MAIN NESTED LOOPS ***///

		for (v = 0; v < unvisitedNodes.size(); v++ ){                      // For every Node v
			for (itr = Paths.begin(); itr != Paths.end(); ++itr){            // On every path i
				liMin = FLT_MAX;
				for (itc = (itr->begin()+1); itc != itr->end(); ++itc){        // In every position p (reversed)


					pathTentative = *itr;
					vector<int>::iterator iTent = pathTentative.begin() + (itc - itr->begin());


					//            cout << (dist(graphNodes[*itc], graphNodes[unvisitedNodes[v]])) << endl;
					//            cout << (dist(graphNodes[*itc-1], graphNodes[unvisitedNodes[v]])) << endl;
					//            std::cin.get();


					/// Following if: If node is adjacent to path just insert it ///
					if( ( dist(graphNodes[*itc], graphNodes[unvisitedNodes[v]]) ) <= minDist   &&
							( dist(graphNodes[*(itc - 1)], graphNodes[unvisitedNodes[v]]) ) <= minDist ){

						pathTentative.insert(iTent, unvisitedNodes[v]);

						checkBest(1);

					}

					/// Otherwise: calculate the shortest traversable path to "get there and go back" ///
					else{

						//cout << "A* in progress! :)" <<  endl;
						//cout << "_ Insert position = " << (itc - itr->begin()) << " _" << endl;

						/// In the first astar.run() we calculate the "way there" path,
						///  while in the second run() we calculate the "way back".


						target = unvisitedNodes[v];
						astarTent.clear();


						if((dist(graphNodes[*(itc - 1)], graphNodes[unvisitedNodes[v]])) > minDist){

							start1 = *(itc-1);
							//cout << "Target:" << target << " start_1:" << start1 << endl;


							astar.run(start1, target);
							//cout << "way there: ";
							for(int i=1; i<astar.path.size()-1; i++){
								//cout << astar.path[i] << " ";
								astarTent.push_back(astar.path[i]);
							}
							//cout << endl;
						}

						//cout << "Inserting target node " <<  unvisitedNodes[v] << endl;
						astarTent.push_back(unvisitedNodes[v]);


						if((dist(graphNodes[*itc], graphNodes[unvisitedNodes[v]])) > minDist){

							start2 = *itc;
							//cout << "Target:" << target << " start_2:" << start2 << endl;

							astar.run(target, start2);
							//cout << "way back: ";
							for(int i=1; i<astar.path.size()-1; i++){
								//cout << astar.path[i] << " ";
								astarTent.push_back(astar.path[i]);
							}
							//cout << endl;
						}

						pathTentative.insert(iTent, astarTent.begin(), astarTent.end());
						checkBest(0);

					}

				} // END P(positions)
			} // END I (robots)
		} // END V (nodes)


		if(choice.neighb == 1){
			/// Inserting chosen best node v, in position p, in path of robot i
			choice.i->insert(choice.p, unvisitedNodes[choice.v]);
		}else{
			/// Inserting chosen best node v, along with all the path to reach it
			choice.i->insert(choice.p, choice.astarPath.begin(), choice.astarPath.end());
		}

		unvisitedNodes.erase(unvisitedNodes.begin() + choice.v);     // Delete it from list of unvisited

		cout << "unvisitedNodes.size() = " << (int)unvisitedNodes.size() << "       " << '\r';
		cout.flush();

	} // END WHILE


	std::cout << '\n';

	std::cout << "The contents of Paths are:" << endl;
	for (itr = Paths.begin(); itr != Paths.end(); ++itr){
		cout << "#" << itr - Paths.begin() << ": ";
		for (itc = itr->begin(); itc != itr->end(); ++itc){
			std::cout << *itc << ' ';
		}
		std::cout << '\n';
	}

}




float VrpGreedyAstar::dist(graphNode &a, graphNode &b){

	float dist = sqrt ( pow(a.posx - b.posx, 2.0) +
			pow(a.posy - b.posy, 2.0) );

	//cout << "Dist=" << dist << endl;
	return dist;
}

void VrpGreedyAstar::checkBest(bool isNeighbour){

	int tentPathLenght = pathLength(pathTentative);

	if(tentPathLenght < liMin){
		liMin = tentPathLenght;

		// * Objective Function * //
		deltavip = liMin - bigL;

		if(deltavip < deltaBest){
			deltaBest = deltavip;
			choice.set_vipn(v, itr, itc, isNeighbour);
			if(!isNeighbour){
				choice.astarPath = astarTent;
			}
		}//End Check Global Best
	}//End Check Local Best

}

void VrpGreedyAstar::printTentative(){
	cout << "Path Tentative: ";
	for(int i=0; i<pathTentative.size();i++){
		cout << pathTentative.at(i) << " ";

	}
	cout << endl;

}

void VrpGreedyAstar::copyPathsTo(vector< vector<int> > &destination){
	destination = Paths;
}

void VrpGreedyAstar::copyGraphTo(vector< graphNode > &destination){
	destination = graphNodes;
}
