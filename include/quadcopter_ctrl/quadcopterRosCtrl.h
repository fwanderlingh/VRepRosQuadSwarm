//‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾//
// *Control of a Quadcopter simulated in VREP*  //
//______________________________________________//

#ifndef QUADCOPTERROSCTRL_H_
#define QUADCOPTERROSCTRL_H_

#define DEF_LOOP_RATE 2      // Loop frequency of node in hertz

// This variable defines the period of the position update:
// 1 = update every node loop
// n = update every n loops
#define STEP_PERIOD 1

#define VREP_X0 10.7
#define VREP_Y0 9.43

#define ROBOT_SQUARE_SIZE 0.5   //This is the quadcopter size taken from the VREP simulator
#define SENSOR_DEPTH 8.0

/*
#define CRITICAL_DIST (double) 0.65       /// Distance above which automatic quadcopter control by
/// VREP is badly handled.
#define WP_STEP (double) 0.65

#define MAP_SCALE 2
*/


enum coord {X, Y, Z};



#endif /* QUADCOPTERROSCTRL_H_ */

