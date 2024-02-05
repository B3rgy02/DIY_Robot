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
robotState robotFSM;

void setup() {
  // Controller setup
  controllerSetup(lcd);

  // FSM setup
  robotFSM = initialState;
}

void loop() {
  // FSM only here
  switch (robotFSM) {
    case initialState:
      // Initialisation state
      
      break;
    
    case powerON:
      // Power ON state

      break;

    case powerOFF:
      // Power OFF state

      break;

    case movementSelection:
      // Movement selection state

      break;
    
    case linearMovement:
      // Linear movement state

      break;

    case jointMovement:
      // Joint movement state

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





















