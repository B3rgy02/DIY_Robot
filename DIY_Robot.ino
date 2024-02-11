/*
File name: DIY_Robot.ino
File description: Main file to be run on Arduino Mega 2560 board. Finite state machine is run here
*/

#include "controller.h"
#include "robotData.h"
#include "Arduino.h"

// Global objects
DIY_Robot myRobot;
LiquidCrystal lcd(LCD_RS_PIN, LCD_ENABLE_PIN, LCD_D0_PIN, LCD_D1_PIN, LCD_D2_PIN, LCD_D3_PIN, LCD_D4_PIN, LCD_D5_PIN, LCD_D6_PIN, LCD_D7_PIN);

// Global variables
robotState currentState; // current FSM State
robotState previousState; // previous FSM State

void setup() {
  // Controller setup
  controllerSetup(lcd);

  // FSM setup
  currentState = initialState;
  previousState = initialState;
}

void loop() {
  // FSM only here
  switch (currentState) {
    case initialState:
      // Initialisation state
      if (checkPwrSwitch()){
        // Ready to power ON
        currentState = powerON;
        previousState = initialState;
      }

      break;
    
    case powerON:
      // Power ON state
      powerUpSequence();

      // Change FSM state to movement selection
      currentState = movementSelection;
      previousState = powerON;
      break;

    case powerOFF:
      // Power OFF state
      powerDownSequence();

      // Change state to initial state
      currentState = initialState;
      previousState = powerOFF;
      break;

    case movementSelection:
      // Movement selection state
      // Check if power switch is still on
      if (checkPwrSwitch()){
        currentState = selectionSeq(currentState, previousState);
        previousState = movementSelection;
      } else {
        // Power OFF State
        currentState = powerOFF;
        previousState = movementSelection;
      }

      break;
    
    case linearMovement:
      // Linear movement state

      break;

    case jointMovement:
      // Joint movement state
      // Check if power switch is still on
      if (checkPwrSwitch()){
        currentState = jointMove(currentState, previousState);
        previousState = jointMovement;
      } else {
        // Power OFF State
        currentState = powerOFF;
        previousState = jointMovement;
      }

      break;

    case homing:
      // Homing state

      break;
    
    case calibration:
      // Calibration state

      break;

    default:
      // Error: Flash LED

      break;
  }
}

// Initialisation state
bool checkPwrSwitch(){
  /*
  Wait for power switch to be activated
  */

  return (digitalRead(SWITCH_POWER) == HIGH);
}

// Power Up Sequence
void powerUpSequence(){
  /* 
  Sequence:
  1. Turn ON Power ON LED
  2. Turn ON Display
  3. Display message for power on
  4. If not at home position, move robot to home
  5. Display sequence complete
  6. Ready LED ON and Busy LED OFF
  */
  
  // Turn ON power ON LED
  digitalWrite(POWER_LED, HIGH);

  // Turn ON display
  lcd.display();

  // Display message for power on
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print("Powering ON...");

  // If not at home position, move robot to home position
  if (myRobot.getHomingState() == false){
    myRobot.homing();
  }

  // Display sequence complete
  lcd.clear();
  lcd.setCursor(0, 4);
  lcd.print("Power ON");
  lcd.setCursor(1, 4);
  lcd.print("Complete");

  // Turn ON ready LED
  digitalWrite(ROBOT_READY_LED, HIGH);
  digitalWrite(ROBOT_BUSY_LED, LOW);

  return;
}

// Power OFF Sequence
void powerDownSequence(){
  /* 
  Sequence:
  1. Display message for power OFF
  2. Ready LED OFF and Busy LED ON
  3. If not at home position, move robot to home
  4. Turn OFF LCD
  5. Turn OFF all LED 
  */

  // Display message for power OFF
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Powering OFF...");

  // Ready LED OFF and Busy LED ON
  digitalWrite(ROBOT_READY_LED, LOW);
  digitalWrite(ROBOT_BUSY_LED, HIGH);

  // If not at home position, move robot to home
  if (myRobot.getHomingState() == false){
    myRobot.homing();
  }

  // Turn OFF LCD
  lcd.noDisplay();

  // Turn OFF all LED
  digitalWrite(POWER_LED, LOW);
  digitalWrite(ROBOT_READY_LED, LOW);
  digitalWrite(ROBOT_BUSY_LED, LOW);
  digitalWrite(GRIPPER_OPEN_LED, LOW);
  digitalWrite(GRIPPER_CLOSED_LED, LOW);

  return;
}

robotState selectionSeq(robotState currentState_, robotState previousState_){
  /*
  Movement sequence:
  1. If not already displayed, display on LCD menu selection
  2. Read inputs buttons (Select, back, up, down) - NB: what to do with multiple button press?
  3. Change selection depending on button pressed
  */
  robotState tempoState = currentState_;


  // Check if first time in movement selection sequence
  if (currentState_ != previousState_){
    // Display menu selection. Default selection: Linear
    String textLCD = "Linear";

    lcd.clear();
    lcd.setCursor(0, 1);
    lcd.print("Select Control:");
    lcd.setCursor(1, 0);
    lcd.print(textLCD);
  }

  //Read Select button
  if (areButtonPressed() && (digitalRead(BUTTON_SELECT) == HIGH)){
    // Button is pressed. Change FSM State to whatever is on screen
    if (textLCD == "Linear"){ tempoState = linearMovement; }
    if (textLCD == "Joint"){ tempoState = jointMovement; }
    if (textLCD == "Calibration"){ tempoState = calibration; }
    if (textLCD == "Homing"){ tempoState = homing; }

    // Change state of select button
    isSelectPressed = true;
  }

  // Read Back button
  if (areButtonPressed() && (digitalRead(BUTTON_BACK) == HIGH)){
    // Back button is pressed. Nothing is happening. Only change state of back button
    isBackPressed = true;
  }

  // Read Up button
  if (areButtonPressed() && (digitalRead(BUTTON_UP) == HIGH)){
    // Up button pressed. Change display text.
    /* Text order: 
    1. Linear
    2. Joint
    3. Homing
    4. Calibration
    */
    if (textLCD == "Linear"){ textLCD = "Joint"; }
    if (textLCD == "Joint"){ textLCD = "Homing"; }
    if (textLCD == "Homing"){ textLCD = "Calibration"; }
    if (textLCD == "Calibration"){ textLCD = "Linear"; }

    // Display
    lcd.clear();
    lcd.setCursor(0, 1);
    lcd.print("Select Control:");
    lcd.setCursor(1, 0);
    lcd.print(textLCD);

    // Change Up button state
    isUpPressed = true;
  }

  // Read Down button
  if (areButtonPressed() && (digitalRead(BUTTON_DOWN) == HIGH)){
    // Down button pressed. Change display text.
    /* Text order: 
    1. Linear
    2. Joint
    3. Homing
    4. Calibration
    */
    if (textLCD == "Linear"){ textLCD = "Calibration"; }
    if (textLCD == "Joint"){ textLCD = "Linear"; }
    if (textLCD == "Homing"){ textLCD = "Joint"; }
    if (textLCD == "Calibration"){ textLCD = "Homing"; }
    
    // Display
    lcd.clear();
    lcd.setCursor(0, 1);
    lcd.print("Select Control:");
    lcd.setCursor(1, 0);
    lcd.print(textLCD);

    // Change Down button state
    isDownPressed = true;
  }

  // Reset button states if not pressed anymore
  resetButtons();

  return tempoState;
}

robotState jointMove(robotState currentState_, robotState previousState_){
  /*
  Joint movement sequence. 2 main parts of this sequence:
  1. Joint selection. Control via buttons
  2. Joint movement. Control via joystick

  Note: 2 joints can be controlled at the same time
  */

  // Part 1: Joint selection. Control via buttons
  robotState tempoState = currentState_;

  String jointNames[5] = {"Base", "Joint 1", "Joint 2", "Joint 3", "Joint 4"};

  // Check if first time in movement selection sequence
  if (currentState_ != previousState_){
    // Display menu selection. Default selection: Base and Joint 1
    lcd.clear();
    lcd.setCursor(0, 1);
    lcd.print("Select Control:");
    lcd.setCursor(1, 0);
    lcd.print(jointNames[myRobot.getJointControl(0)]);
    lcd.setCursor(1, 8);
    lcd.print(jointNames[myRobot.getJointControl(1)]);
  }

  // Read Select button
  if (areButtonPressed() && (digitalRead(BUTTON_SELECT) == HIGH)){
    // Button is pressed. Change control index
    myRobot.setjointControlIndex((myRobot.getjointControlIndex() + 1) % 2);

    // Change button state
    isSelectPressed = true;
  }

  // Read Back button
  if (areButtonPressed() && (digitalRead(BUTTON_BACK) == HIGH)){
    // Back button is pressed. Go back to movement selection screen
    tempoState = movementSelection;

    // Change button state
    isBackPressed = true;
  }

  // Read Up button
  if (areButtonPressed() && (digitalRead(BUTTON_UP) == HIGH)){
    // Up button pressed. Change moving joint.
    changeJointControl(1);

    // Display changes
    lcd.clear();
    lcd.setCursor(0, 1);
    lcd.print("Select Control:");
    lcd.setCursor(1, 0);
    lcd.print(jointNames[myRobot.getJointControl(0)]);
    lcd.setCursor(1, 8);
    lcd.print(jointNames[myRobot.getJointControl(1)]);

    // Change Up button state
    isUpPressed = true;
  }

  // Read Down button
  if (areButtonPressed() && (digitalRead(BUTTON_UP) == HIGH)){
    // Down button pressed. Change moving joint.
    changeJointControl(-1);

    // Display changes
    lcd.clear();
    lcd.setCursor(0, 1);
    lcd.print("Select Control:");
    lcd.setCursor(1, 0);
    lcd.print(jointNames[myRobot.getJointControl(0)]);
    lcd.setCursor(1, 8);
    lcd.print(jointNames[myRobot.getJointControl(1)]);

    // Change Down button state
    isDownPressed = true;
  }

  // Reset button states if not pressed anymore
  resetButtons();

  // Part 2: Joint movement. Control via joystick. To do only when changes are applied
  /*
  Moving sequence:
  1. Read joytstick analog values
  2. Convert to angle
  3. Read joystick digital value
  4. Toggle gripper if applicable 
  5. Move to angles
  */

  // Read joystick analog values
  int rawJoystickX = analogRead(JOYSTICK_X);
  int rawJoystickY = analogRead(JOYSTICK_Y);

  // If out of deadzone, proceed to move
  if ((abs(rawJoystickX - 512) <= JOYSTICK_X_DEADZONE) || (abs(rawJoystickY - 512) <= JOYSTICK_Y_DEADZONE)) {
    // Get all joints values and controlled joints
    int tempJointVals[5];
    int tempControlledJointsX = myRobot.getJointControl(0);
    int tempControlledJointsY = myRobot.getJointControl(1);

    for (int i = 0; i < 5; i++){
      tempJointVals[i] = myRobot.getJointAngle(i);
    }

    // Limits mapping angles
    int newMapLowX = tempJointVals[tempControlledJointsX] - JOYSTICK_SENSITIVITY;
    int newMapHighX = tempJointVals[tempControlledJointsX] + JOYSTICK_SENSITIVITY;
    int newMapLowY = tempJointVals[tempControlledJointsY] - JOYSTICK_SENSITIVITY;
    int newMapHighY = tempJointVals[tempControlledJointsY] + JOYSTICK_SENSITIVITY;
    
    if (newMapLowX < 0) { newMapLowX = 0; }
    if (newMapHighX < 0) { newMapHighX = 180; }
    if (newMapLowY < 0) { newMapLowY = 0; }
    if (newMapHighY < 0) { newMapHighY = 180; }

    // Convert to angle
    int angleX = map(rawJoystickX, JOYSTICK_X_MIN, JOYSTICK_X_MAX, newMapLowX, newMapHighX);
    int angleY = map(rawJoystickY, JOYSTICK_Y_MIN, JOYSTICK_Y_MAX, newMapLowY, newMapHighY);

    // Read joystick digital value and toggle if applicable
    if ((digitalRead(JOYSTICK_Z) == HIGH) && (!isJoystickPressed)){
      // Joystick pressed. Toggle gripper
      myRobot.toggleGripper();

      // Change joystick press state
      isJoystickPressed = true;
    }

    // Reset joystick pressing state if applicable
    resetJoystickPress();

    // Move joints to angles
    tempJointVals[tempControlledJointsX] = angleX;
    tempJointVals[tempControlledJointsY] = angleY;

    myRobot.setJointValues(tempJointVals);
    myRobot.move();
  }

  return tempoState;
}

bool areButtonPressed(){
  // Check if all 4 buttons are non pressed
  return ((!isSelectPressed) && (!isBackPressed) && (!isUpPressed) && (!isDownPressed));
}

void resetButtons(){
  // Reset button states if not pressed anymore
  if ((isSelectPressed == true) && (digitalRead(BUTTON_SELECT) == LOW)){ isSelectPressed = false; }
  if ((isBackPressed == true) && (digitalRead(BUTTON_BACK) == LOW)){ isBackPressed = false; }
  if ((isUpPressed == true) && (digitalRead(BUTTON_UP) == LOW)){ isUpPressed = false; }
  if ((isDownPressed == true) && (digitalRead(BUTTON_DOWN) == LOW)){ isDownPressed = false; }

  return;
}

void changeJointControl(int plusMinus){
  // Get current controlled joints and control index
  int controlledJoints[2];
  controlledJoints[0] = myRobot.getJointControl(0);
  controlledJoints[1] = myRobot.getJointControl(1);
  int controlJointIndex = myRobot.getjointControlIndex();

  // Create temporary array containing all possibilities of controllable joints
  int tempArr[4];
  int tempCounter = 0;
  int tempIndex;

  for (int i = 0; i < 4; i++){
    if (i != controlledJoints[(controlJointIndex + 1) % 2]){
      if (i == controlledJoints[controlJointIndex]){
        tempIndex = tempCounter;
      }

      tempArr[tempCounter] = i;
      tempCounter++;
    }
  }

  // Change controlled joint
  myRobot.setJointsControl(controlJointIndex, tempArr[(tempIndex + 4 + plusMinus) % 4]);

  return;
}

void resetJoystickPress(){
  if ((isJoystickPressed == true) && (digitalRead(JOYSTICK_Z) == LOW)){ isJoystickPressed = false; }
  return;
}




















