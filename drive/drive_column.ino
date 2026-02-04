#include <Arduino.h>
#include "Enes100.h"
#include <SoftwareSerial.h>

// Motor pins (same mapping as drive.ino)
#define ENA1 5
#define IN1 22
#define IN2 23
#define ENB1 6
#define IN3 24
#define IN4 25
#define ENA2 7
#define IN5 26
#define IN6 27
#define ENB2 8
#define IN7 28
#define IN8 29

// Ultrasonic pins
#define TRIG_FRONT_LEFT   52
#define ECHO_FRONT_LEFT   53
#define TRIG_FRONT_RIGHT  30
#define ECHO_FRONT_RIGHT  31
#define TRIG_LEFT         34
#define ECHO_LEFT         35
#define TRIG_RIGHT        36
#define ECHO_RIGHT        37

// Config
const float FRONT_THRESH = 0.4;
const float SIDE_CLEAR   = 0.8;
const float SIDE_CLOSE   = 0.30;
const float SIDE_DETECT  = 0.35; // not used in simplified flow
const float FOLLOW_DIST  = 0.18;
const float GOAL_X       = 3.4;
const float GOAL_Y       = 1.5;
const float Y_TOL        = 0.05;
const float COL1_X       = 0.9;
const float COL2_X       = 1.7;
const float OPEN_ZONE_X  = 2.8;
const float CLEAR_ADV_X  = 0.25;
const float SWEEP_TOP    = 1.7;
const float SWEEP_BOTTOM = 0.3;
const int   BASE_SPEED   = 150;
const float HEADING_TOL  = 0.08;
const unsigned long TURN_DELAY_MS = 300;
const unsigned long CLEAR_ROLL_MS = 200;
const float FOLLOW_KP    = 25;   // distance correction
const float HEADING_KP   = 80;   // heading correction

enum State { TO_CENTER_Y, TURN_EAST, COLUMN1_DESCEND, COLUMN2_DESCEND, OPEN_ZONE, GOAL_REACHED };
State state = TO_CENTER_Y;

// Follow context
float currentColumnX = 0.0; // tracked column target

// Helpers
float readDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long d = pulseIn(echoPin, HIGH, 30000);
  if (d == 0) return -1;
  return d * 0.000343 / 2.0; // meters
}

float getFrontDistance() {
  float dL = readDistance(TRIG_FRONT_LEFT, ECHO_FRONT_LEFT);
  float dR = readDistance(TRIG_FRONT_RIGHT, ECHO_FRONT_RIGHT);
  if (dL == -1 && dR == -1) return -1;
  if (dL == -1) return dR;
  if (dR == -1) return dL;
  return min(dL, dR);
}

float getSideDistance(bool left) {
  return left ? readDistance(TRIG_LEFT, ECHO_LEFT)
              : readDistance(TRIG_RIGHT, ECHO_RIGHT);
}

void setMotorPWM(int leftPWM, int rightPWM) {
  leftPWM  = constrain(leftPWM, -255, 255);
  rightPWM = constrain(rightPWM, -255, 255);

  digitalWrite(IN1, leftPWM > 0);
  digitalWrite(IN2, leftPWM < 0);
  digitalWrite(IN5, leftPWM > 0);
  digitalWrite(IN6, leftPWM < 0);
  analogWrite(ENA1, abs(leftPWM));
  analogWrite(ENA2, abs(leftPWM));

  // Right side inverted wiring
  digitalWrite(IN3, rightPWM < 0);
  digitalWrite(IN4, rightPWM > 0);
  digitalWrite(IN7, rightPWM < 0);
  digitalWrite(IN8, rightPWM > 0);
  analogWrite(ENB1, abs(rightPWM));
  analogWrite(ENB2, abs(rightPWM));
}

void stopMotors() { setMotorPWM(0, 0); }
void moveStraight() { setMotorPWM(BASE_SPEED, BASE_SPEED); }

float angleDiff(float target, float current) { return target - current; }

void turnTo(float targetAngle, float tol = HEADING_TOL) {
  stopMotors();
  delay(TURN_DELAY_MS);
  while (true) {
    if (!Enes100.isVisible()) { stopMotors(); return; }
    float th = Enes100.getTheta();
    float err = angleDiff(targetAngle, th);
    if (fabs(err) < tol) {
      stopMotors();
      delay(20);
      float th2 = Enes100.getTheta();
      if (fabs(angleDiff(targetAngle, th2)) < tol) {
        delay(TURN_DELAY_MS);
        return;
      }
    }
    setMotorPWM(BASE_SPEED, -BASE_SPEED);
    delay(5);
  }
}

void setup() {
  int motorPins[] = {IN1,IN2,IN3,IN4,IN5,IN6,IN7,IN8,ENA1,ENB1,ENA2,ENB2};
  for (int i=0;i<12;i++) pinMode(motorPins[i], OUTPUT);
  int trigPins[] = {TRIG_FRONT_LEFT, TRIG_FRONT_RIGHT, TRIG_LEFT, TRIG_RIGHT};
  int echoPins[] = {ECHO_FRONT_LEFT, ECHO_FRONT_RIGHT, ECHO_LEFT, ECHO_RIGHT};
  for (int i=0;i<4;i++){ pinMode(trigPins[i], OUTPUT); pinMode(echoPins[i], INPUT); }
  Serial.begin(9600);
  Enes100.begin("Material Mermaids", MATERIAL, 500, 1120, 10, 11);
  Enes100.println("Drive simple column init");
}

void loop() {
  if (!Enes100.isVisible()) { stopMotors(); return; }
  float x = Enes100.getX();
  float y = Enes100.getY();
  float theta = Enes100.getTheta();
  float front = getFrontDistance();

  if (state == TO_CENTER_Y) {
    float err = GOAL_Y - y;
    if (fabs(err) < Y_TOL) {
      stopMotors();
      state = TURN_EAST;
      currentColumnX = COL1_X;
      return;
    }
    // drive north/south to y=1.5
    setMotorPWM(BASE_SPEED - 30*err, BASE_SPEED + 30*err);
    return;
  }

  if (state == TURN_EAST) {
    turnTo(0);
    state = COLUMN1_DESCEND;
    currentColumnX = COL1_X;
    return;
  }

  // Goal
  if (x >= GOAL_X && fabs(y - GOAL_Y) < 0.1) { stopMotors(); state = GOAL_REACHED; return; }
  if (state == GOAL_REACHED) { stopMotors(); return; }

  // Open zone y-correction
  if (state == OPEN_ZONE || x >= OPEN_ZONE_X) {
    float err = GOAL_Y - y;
    setMotorPWM(BASE_SPEED - 30*err, BASE_SPEED + 30*err);
    if (x >= GOAL_X && fabs(y-GOAL_Y)<0.1) { stopMotors(); state = GOAL_REACHED; }
    return;
  }

  // Column progression with downward scan
  if (state == COLUMN1_DESCEND || state == COLUMN2_DESCEND) {
    float targetCol = (state == COLUMN1_DESCEND) ? COL1_X : COL2_X;
    if (x < targetCol - 0.05) { moveStraight(); return; }

    // Move to top (north) if not there yet
    if (y < SWEEP_TOP - Y_TOL) {
      turnTo(PI/2);
      setMotorPWM(BASE_SPEED, BASE_SPEED); // drive north
      return;
    }

    // Ensure facing south to descend
    turnTo(-PI/2);

    // Descend while side indicates obstacle
    float side = getSideDistance(true); // left sensor while facing south
    if (side == -1) side = 0.0; // treat unknown as obstacle present

    if (side < SIDE_CLEAR && y > SWEEP_BOTTOM + Y_TOL) {
      // keep descending
      setMotorPWM(-BASE_SPEED, -BASE_SPEED);
      return;
    }

    // Clear: either side is clear or bottom reached
    moveStraight();
    delay(CLEAR_ROLL_MS);
    turnTo(0);
    if (state == COLUMN1_DESCEND) {
      state = COLUMN2_DESCEND;
      currentColumnX = COL2_X;
    } else {
      state = OPEN_ZONE;
    }
    return;
  }

}
