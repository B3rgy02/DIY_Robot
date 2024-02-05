/*
File name: robotData.cpp
File description: 
*/

/*
Add-ons:
- Add LED light control on functions
*/

#include "robotData.h"
#include "Arduino.h"

// Robot Constructor
DIY_Robot::DIY_Robot(){
  // Link Arduino pins to servos
  robotServos[0].attach(ROBOT_BASE);
  robotServos[1].attach(ROBOT_JOINT_1);
  robotServos[2].attach(ROBOT_JOINT_2);
  robotServos[3].attach(ROBOT_JOINT_3);
  robotServos[4].attach(ROBOT_JOINT_4);
  robotServos[5].attach(ROBOT_GRIPPER);

  for (int i = 0; i < NB_JOINTS; i++) {
    jointValues[i] = 0;
    jointOffsets[i] = 0;
  }
}

// Robot Set functions
void DIY_Robot::setJointValues(int jointValues_[5]){
  for (int i = 0; i < NB_JOINTS; i++){
    jointValues[i] = jointValues_[i];
  }

  return;
}

void DIY_Robot::setJointOffsets(int jointOffsets_[5]){
  for (int i = 0; i < NB_JOINTS; i++){
    jointOffsets[i] = jointOffsets_[i];
  }

  return;
}

// Robot methods
// Homing function
void DIY_Robot::homing(){
  // Open gripper
  robotServos[5].write(gripperAngles[0]);
  gripperState = opened;
  delay(GRIPPER_DELAY_MS);

  // Move joints to homing position
  for (int i = 0; i < NB_JOINTS; i++){
    robotServos[i].write(homingAngles[i]);
  }
  delay(HOMING_DELAY_MS);

  // Close gripper
  robotServos[5].write(gripperAngles[1]);
  gripperState = closed;
  delay(GRIPPER_DELAY_MS);

  return;
}

// Move function
void DIY_Robot::move(){
  // Move joints to position (with offset)
  for (int i = 0; i < NB_JOINTS; i++){
    robotServos[i].write(jointValues[i] + jointOffsets[i]);
  }
  delay(HOMING_DELAY_MS);

  return;
}

// Open gripper function
void DIY_Robot::openGripper(){
  // Open gripper
  robotServos[5].write(gripperAngles[0]);
  gripperState = opened;
  delay(GRIPPER_DELAY_MS);

  return;
}

// Close gripper function
void DIY_Robot::closeGripper(){
  // Open gripper
  robotServos[5].write(gripperAngles[1]);
  gripperState = opened;
  delay(GRIPPER_DELAY_MS);

  return;
}

void DIY_Robot::toggleGripper(){
  if (gripperState == opened){
    closeGripper();
  } else {
    openGripper();
  }

  return;
}
















