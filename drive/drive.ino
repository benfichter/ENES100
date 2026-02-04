#include <cmath>
#include "Arduino.h"
#include "Tank.h"
#include "Enes100.h"
// Note: This code still needs debugging for traveling along the boundary.
// --- Function Prototypes ---
void driveSetup();
void driveLoop();
void moveStraight();
void moveLeft();
void moveRight();
void alongObstacle();

/*   ** DONT DELETE FOR SIMULATION PURPOSES
void setup() {
  // Enes100.begin("Material Mermaids", MATERIAL, 3, 1116, 8, 9); *DONT DELETE
  Tank.begin();   // initialize tank motors and sensors
}*/ 

// --- Setup for the drive system (Tank only) ---
void driveSetup() {
  Tank.begin();   // initialize tank motors and sensors
}

// --- Loop for the drive logic ---
void driveLoop() { // change to void loop() for simulation ** DONT DELETE **
  float x = Enes100.getX();
  float y = Enes100.getY();
  float distance = Tank.readDistanceSensor(1);
   if (x >= 3.1){ // stop condition
      Tank.turnOffMotors();
      while (true);
  }
  if (distance == -1 || distance >= 0.25) {
    moveStraight();
  } else if (distance < 0.25) {
    if (y >= 1) { // upper half
      moveRight();
      delay(7100);
      alongObstacle();
    } else {
      moveLeft();
      delay(7100);
      alongObstacle();
    }
  }
}

void alongObstacle() {
  float distance = Tank.readDistanceSensor(1);
  int clear = 0;

  while (true) {
    distance = Tank.readDistanceSensor(1);
    float theta = Enes100.getTheta();
    float y = Enes100.getY();

    if (y >= 1) {
      if (theta > -1.5) {
        Tank.setLeftMotorPWM(10);
        Tank.setRightMotorPWM(0);
      } else if (theta < -1.7) {
        Tank.setLeftMotorPWM(0);
        Tank.setRightMotorPWM(10);
      } else {
        Tank.setRightMotorPWM(25);
        Tank.setLeftMotorPWM(25);
      }
    } else {
      if (theta > 1.7) {
        Tank.setLeftMotorPWM(10);
        Tank.setRightMotorPWM(0);
      } else if (theta < 1.5) {
        Tank.setLeftMotorPWM(0);
        Tank.setRightMotorPWM(10);
      } else {
        Tank.setRightMotorPWM(25);
        Tank.setLeftMotorPWM(25);
      }
    }

    delay(100);
    Enes100.print(distance);

    if (distance > 0.78 && distance != -1) {
      clear++;
    } else {
      clear = 0;
    }

    if (clear == 2) {
      Tank.setLeftMotorPWM(0);
      Tank.setRightMotorPWM(0);
      break;
    }
  }
  delay(300);
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

void moveStraight() {
  float theta = Enes100.getTheta();
  int baseSpeed = 25;
  int maxCorrection = 25;
  int leftSpeed, rightSpeed;

  if (theta > 0.05) {
    leftSpeed = baseSpeed;
    rightSpeed = baseSpeed - maxCorrection;
  } else if (theta < -0.05) {
    leftSpeed = baseSpeed - maxCorrection;
    rightSpeed = baseSpeed;
  } else {
    leftSpeed = baseSpeed;
    rightSpeed = baseSpeed;
  }

  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  Tank.setLeftMotorPWM(leftSpeed);
  Tank.setRightMotorPWM(rightSpeed);
}
