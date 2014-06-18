/*
 * PathPlanningAlg.h
 *
 *  Created on: Apr 7, 2014
 *      Author: francescow
 */

#ifndef PATHPLANNINGALG_H_
#define PATHPLANNINGALG_H_

#include <vector>
#include "armadillo"

class PathPlanningAlg
{
  std::vector<std::vector<int> > grid;
  int gridWidth;
  int gridHeight;
  arma::mat envGrid;

public:
  PathPlanningAlg();
  virtual ~PathPlanningAlg();

  static void InterpNewPoint(geometry_msgs::PoseStamped* quadPos,
                                geometry_msgs::PoseStamped* targetPos,
                                float dSubWP[3]);
  static float Distance(geometry_msgs::PoseStamped* quadPos,
                         geometry_msgs::PoseStamped* targetPos);


};

#endif /* PATHPLANNINGALG_H_ */
