#ifndef ASCTECROSCTRL_H_
#define ASCTECROSCTRL_H_

#define DEF_LOOP_RATE 2      // Loop frequency of node in hertz

// This variable defines the period of the position update:
// 1 = update every node loop
// n = update every n loops
#define STEP_PERIOD 1

#define VREP_X0 10.7
#define VREP_Y0 9.43

#define ROBOT_SQUARE_SIZE 0.5   //This is the quadcopter size taken from the VREP simulator
#define SENSOR_DEPTH 8.0

#define WP_STEP (double) 5

#define MAP_SCALE 5
//#define CRITICAL_DIST (double) 0.8       /// Distance above which automatic quadcopter control by



#endif /* ASCTECROSCTRL_H_ */

