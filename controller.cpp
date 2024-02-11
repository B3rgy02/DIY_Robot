/*
File name: controller.cpp
File description: 
*/

#include <LiquidCrystal.h>
#include "controller.h"
#include "Arduino.h"

// LCD variable text (2nd line of the LCD)
String textLCD;

// Button pressing state
bool isSelectPressed;
bool isBackPressed;
bool isUpPressed;
bool isDownPressed;

// Joystick pressing state
bool isJoystickPressed;

void controllerSetup(LiquidCrystal lcd_){
  /* 
  Joystick setup
  Only digital pins need pin association
  */
  pinMode(JOYSTICK_Z, INPUT);
  isJoystickPressed = false;

  // LCD setup
  lcd_.begin(16,2);
  lcd_.clear();
  lcd_.noCursor();
  lcd_.noDisplay();
  
  /* 
  Button setup
  1st button: Select button for LCD display
  2nd button: Back button for LCD display
  3rd button: Up button in menu for LCD display
  4th button: Down button in menu for LCD display
  */
  pinMode(BUTTON_SELECT, INPUT);
  pinMode(BUTTON_BACK, INPUT);
  pinMode(BUTTON_UP, INPUT);
  pinMode(BUTTON_DOWN, INPUT);

  // Set button pressing states
  isSelectPressed = false;
  isBackPressed = false;
  isUpPressed = false;
  isDownPressed = false;

  // Switch setup for power ON/OFF mode
  pinMode(SWITCH_POWER, INPUT);

  /* 
  LED setup
  1st LED: Power ON/OFF state (Green)
  2nd LED: Robot ready to move (Green)
  3rd LED: Robot busy (Red)
  4th LED: Gripper opened (Green)
  5th LED: Gripper closed (Red)
  */
  pinMode(POWER_LED, OUTPUT);
  pinMode(ROBOT_READY_LED, OUTPUT);
  pinMode(ROBOT_BUSY_LED, OUTPUT);
  pinMode(GRIPPER_OPEN_LED, OUTPUT);
  pinMode(GRIPPER_CLOSED_LED, OUTPUT);

  return;
}



























