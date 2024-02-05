/*
File name: controller.h
File description: 
*/

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "Arduino.h"
#include <LiquidCrystal.h>

// Joystick pins
#define JOYSTICK_X A0
#define JOYSTICK_Y A1
#define JOYSTICK_Z 8

// Button pins
#define BUTTON_SELECT 9
#define BUTTON_BACK 10
#define BUTTON_UP 11
#define BUTTON_DOWN 12

// Switch pin
#define SWITCH_POWER 13

// LED pins
#define POWER_LED 14
#define ROBOT_READY_LED 15
#define ROBOT_BUSY_LED 16
#define GRIPPER_OPEN_LED 17
#define GRIPPER_CLOSED_LED 18

// LCD pins
#define LCD_RS_PIN 22
#define LCD_ENABLE_PIN 23
#define LCD_D0_PIN 24
#define LCD_D1_PIN 25
#define LCD_D2_PIN 26
#define LCD_D3_PIN 27
#define LCD_D4_PIN 28
#define LCD_D5_PIN 29
#define LCD_D6_PIN 30
#define LCD_D7_PIN 31

// Function definition
void controllerSetup(LiquidCrystal lcd_);

#endif
























