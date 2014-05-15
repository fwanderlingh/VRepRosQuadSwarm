//	Copyright (c) 2014, Francesco Wanderlingh. 			        //
//	All rights reserved.						                //
//	License: BSD (http://opensource.org/licenses/BSD-3-Clause)  //

/*
 * PathPlanningAlg.cpp
 *
 *  Created on: Apr 7, 2014
 *      Author: francescow
 */

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "PathPlanningAlg.h"
#include "robotInfo.h"

PathPlanningAlg::PathPlanningAlg()
{
  gridWidth = 0;
  gridHeight = 0;

}

PathPlanningAlg::~PathPlanningAlg()
{
  // TODO Auto-generated destructor stub
}

void PathPlanningAlg::CreateGrid(int width, int height)
{
  /*While creating the grid I am assuming a standard size of the robot
  * in order to decide the size of the grid. The input (width, height) are
  * the sizes of the rectangle i want to explore.
  */
  gridWidth = ceil(width/ROBOT_SQUARE_SIZE);
  gridHeight = ceil(height/ROBOT_SQUARE_SIZE);

  envGrid.set_size(gridWidth, gridHeight);

}

void PathPlanningAlg::CreatePath()
{

}

void PathPlanningAlg::InterpNewPoint(geometry_msgs::PoseStamped* quadPos,
                                          geometry_msgs::PoseStamped* targetPos,
                                          float dSubWP[3])
{
  /* interpNewPoint() interpolates the next point in the direction of the target
   * with a distance from the quadcopter that is less than CRITICAL_DIST
   * As a first step we compute the vector d = (quadPos -> targetPos),
   * then we compute its norm to get the unit vector and then we will
   * multiply the unit vector for a given proportionality constant to
   * get the infinitesimal dSubWP movement.
   */

   float distVec[3] = { (targetPos->pose.position.x - quadPos->pose.position.x),
                        (targetPos->pose.position.y - quadPos->pose.position.y),
                        (targetPos->pose.position.z - quadPos->pose.position.z) };
   float distVecNorm = Distance(quadPos, targetPos);
   //std::cout << "Distance = " <<  distVecNorm << " m" << std::endl;
   dSubWP[X] = (distVec[X]/distVecNorm)*CRITICAL_DIST;
   dSubWP[Y] = (distVec[Y]/distVecNorm)*CRITICAL_DIST;
   dSubWP[Z] = (distVec[Z]/distVecNorm)*CRITICAL_DIST;

}

float PathPlanningAlg::Distance(geometry_msgs::PoseStamped* quadPos,
                                   geometry_msgs::PoseStamped* targetPos){

  float dist = sqrt (pow(quadPos->pose.position.x - targetPos->pose.position.x, 2.0) +
                      pow(quadPos->pose.position.y - targetPos->pose.position.y, 2.0) +
                      pow(quadPos->pose.position.z - targetPos->pose.position.z, 2.0) );
  return dist;
}


