#include <cmath>
#include "Arduino.h"
#include "Tank.h"
#include "Enes100.h"
// Note: This code still needs debugging for traveling along the boundary, along with handling boundary limits.
// The code may also need to handle wall boundaries as well.
// --- Function Prototypes ---
void driveSetup();
void driveLoop();
void moveStraight();
void moveLeft();
void moveRight();
void alongObstacle();

// --- Setup for the drive system (Tank only) ---
void setup() {
  Enes100.begin("Material Mermaids", MATERIAL, 3, 1116, 8, 9); 
  Tank.begin();   // initialize tank motors and sensors
}

// --- Loop for the drive logic ---
void loop() {
  float x = Enes100.getX();
  float y = Enes100.getY();
      moveRight();
      delay(7100);
      Tank.turnOffMotors();
      delay(1000);
      moveRight();
      delay(7100);
      Tank.turnOffMotors();
      delay(1000);
      moveRight();
      delay(7000);
      Tank.turnOffMotors();
      delay(1000);
      Tank.turnOffMotors();
      while (true);
}

void moveRight() {
  float theta = Enes100.getTheta();

  if (theta >= -1.6) {
    Tank.setRightMotorPWM(-25);
    Tank.setLeftMotorPWM(25);
  } else if (theta < -1.6) {
    Tank.setRightMotorPWM(25);
    Tank.setLeftMotorPWM(-25);
  } else {
    Tank.setRightMotorPWM(25);
    Tank.setLeftMotorPWM(25);
  }
}

void moveLeft() {
  float theta = Enes100.getTheta();

  if (theta <= 1.6) {
    Tank.setRightMotorPWM(25);
    Tank.setLeftMotorPWM(-25);
  } else if (theta > 1.6) {
    Tank.setLeftMotorPWM(25);
    Tank.setRightMotorPWM(-25);
  } else {
    Tank.setRightMotorPWM(25);
    Tank.setLeftMotorPWM(25);
  }
}
