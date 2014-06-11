#ifndef ASTAR_H_
#define ASTAR_H_

#include "stlastar.h" // See header for copyright and usage information
#include <iostream>
#include <stdio.h>
#include <MapSearchNode.h>


/*******************   NOTES ON THE INPUT MAP   **************************

   Each element contains an integer from 0 to 5 which indicates the cost 
   of travel across the terrain. Zero means the least possible difficulty 
   in travelling (think ice rink if you can skate) whilst 5 represents the 
   most difficult. 9 indicates that we cannot pass.

 **************************************************************************/

std::vector<int> MapSearchNode::map;
int MapSearchNode::MAP_WIDTH;
int MapSearchNode::MAP_HEIGHT;


class Astar
{

  int gridSizeX;
  int gridSizeY;

  AStarSearch<MapSearchNode> astarsearch;
  std::vector<int> access_map;
  int start, target;

public:
  Astar():
    gridSizeX(0),
    gridSizeY(0),
    start(0),
    target(0)
{ }
  Astar(std::vector<int> a_map, int sizeX, int sizeY):
    access_map(a_map),
    gridSizeX(sizeX),
    gridSizeY(sizeY),
    start(0),
    target(0)
  {
    MapSearchNode::map = a_map;
    MapSearchNode::MAP_WIDTH = sizeX;
    MapSearchNode::MAP_HEIGHT = sizeY;
  }
  virtual ~Astar(){}

  std::vector<int> path;
  void run(int v_start, int v_target);
  void printMap();
  void printSolution();

};



void Astar::run(int v_start, int v_target){

  start = v_start;
  target = v_target;
  path.clear();

  MapSearchNode nodeStart;
  nodeStart.x = start/gridSizeY;
  nodeStart.y = start%gridSizeY;

  MapSearchNode nodeEnd;
  nodeEnd.x = target/gridSizeY;
  nodeEnd.y = target%gridSizeY;

  // Set Start and goal states
  astarsearch.SetStartAndGoalStates( nodeStart, nodeEnd );

  unsigned int SearchState;

  do{
    SearchState = astarsearch.SearchStep();
  }
  while( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SEARCHING );

  if( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SUCCEEDED )
  {

    MapSearchNode *node = astarsearch.GetSolutionStart();

    /// Saving solution in path vector
    do{
      path.push_back(node->x * gridSizeY+node->y);
      access_map.at(node->x * gridSizeY+node->y) = -1;
      node = astarsearch.GetSolutionNext();

    }while( node != NULL );

    // Once you're done with the solution you can free the nodes up
    astarsearch.FreeSolutionNodes();

  }
  else if( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_FAILED )
  {
    cout << "Search terminated. No traversable path found.\n";
  }

  //astarsearch.EnsureMemoryFreed();

}

void Astar::printSolution(){

  cout << "Path from " << start << " to " << target << " (path length=" << path.size() << "):" << endl;

  for(int i=0; i<path.size(); i++){
    cout << path.at(i) << " ";
  }
  cout << endl;

  for(int i=0; i<access_map.size(); i++){
    if(i%gridSizeY == 0) printf("\n");
    if(i == start) printf("S ");
    else if(i == target) printf("T ");
    else if(access_map.at(i) >=0 && access_map.at(i) <= 5) printf(". ");
    else if(access_map.at(i) == 9) printf("■ ");
    else if(access_map.at(i) == -1) printf("X ");

  }
  cout << endl;
}

void Astar::printMap(){

  cout << "Map:";

  for(int i=0; i<access_map.size(); i++){
    if(i%gridSizeY == 0) printf("\n");
    if(access_map.at(i) >=0 && access_map.at(i) <= 5) printf(". ");
    else if(access_map.at(i) == 9) printf("■ ");
  }
  cout << endl;
}

#endif

