//‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾//
// *Control of a Quadcopter simulated in VREP*  //
//______________________________________________//

#ifndef QUADCOPTERROSCTRL_H_
#define QUADCOPTERROSCTRL_H_

#define DEF_LOOP_RATE 2      // Loop frequency of node in hertz
#define STEP_PERIOD 1        // This variable defines the period of the position update:
                                // 1 = update every node loop
                                // n = update every n loops
#define VREP_X0 11
#define VREP_Y0 9.5

#define ROBOT_SQUARE_SIZE 0.5   //This is the robot size taken from the VREP simulator
#define SENSOR_DEPTH 8.0

#define CRITICAL_DIST (float) 0.8       /// Distance above which automatic quadcopter control by
                                            /// VREP is badly handled.

enum{X, Y, Z};



#endif /* QUADCOPTERROSCTRL_H_ */

