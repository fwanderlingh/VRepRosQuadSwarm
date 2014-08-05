/*
 * PathPlanningAlg.h
 *
 *  Created on: Apr 7, 2014
 *      Author: francescow
 */

#ifndef PATHPLANNINGALG_H_
#define PATHPLANNINGALG_H_

#include <vector>

class PathPlanningAlg
{
  std::vector<std::vector<int> > grid;
  int gridWidth;
  int gridHeight;

public:
  PathPlanningAlg();
  virtual ~PathPlanningAlg();

  static void InterpNewPoint(geometry_msgs::PoseStamped* quadPos,
                                geometry_msgs::PoseStamped* targetPos,
                                double dSubWP[3]);

  static double Distance(geometry_msgs::PoseStamped* quadPos,
                         geometry_msgs::PoseStamped* targetPos);

  static int LoadParams(std::string controlMode,
                         double &scale, double &ofs_x, double &ofs_y,
                         double &wpStep, double &critDist, double &threshold);


};

#endif /* PATHPLANNINGALG_H_ */
