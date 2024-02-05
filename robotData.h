/*
File name: robotData.h
File description: 
*/

#ifndef ROBOTDATA_H
#define ROBOTDATA_H

#include "Arduino.h"
#include <Servo.h>

#define NB_JOINTS 5
#define NB_SERVOS 6

#define ROBOT_BASE 2
#define ROBOT_JOINT_1 3
#define ROBOT_JOINT_2 4
#define ROBOT_JOINT_3 5
#define ROBOT_JOINT_4 6
#define ROBOT_GRIPPER 7

#define HOMING_DELAY_MS 3000
#define GRIPPER_DELAY_MS 1000

class DIY_Robot {
  private:
    Servo robotServos[6];

    int jointValues[5];
    int jointOffsets[5];

    int jointsLimits[2][5] = { // 1st row for minimum, 2nd row for maximum. Non changeable
      {0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0}
    }; 
    int homingAngles[5] = {0, 0, 0, 0, 0}; // Homing joint values. Non changeable

    int gripperAngles[2] = {0, 0}; // Gripper angles (1st: open, 2nd: closed). Non changeable
    enum openCloseState {
      opened = 0,
      closed = 1
    } gripperState;

  public:
    // Constructor
    DIY_Robot();

    // Set functions
    void setJointValues(int jointValues_[5]);
    void setJointOffsets(int jointOffsets_[5]);

    // Methods
    void homing();
    void move();
    void openGripper();
    void closeGripper();
    void toggleGripper();
};

enum robotState {
  initialState = 0,
  powerON = 1,
  powerOFF = 2,
  movementSelection = 3,
  linearMovement = 4,
  jointMovement = 5,
  homing = 6,
  calibration = 7
};



#endif