/*
 * robotInfo.h
 *
 *  Created on: Apr 8, 2014
 *      Author: francescow
 */

#ifndef ROBOTINFO_H_
#define ROBOTINFO_H_

#define ROBOT_SQUARE_SIZE 0.5   //This is the robot size taken from the VREP simulator
#define SENSOR_DEPTH 8.0

#define CRITICAL_DIST (float) 0.8       /// Distance above which automatic quadcopter control by
                                            /// VREP is badly handled.

enum{X, Y, Z};

#endif /* ROBOTINFO_H_ */
