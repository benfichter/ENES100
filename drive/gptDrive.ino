#include <Arduino.h>
#include "Enes100.h"
#include <SoftwareSerial.h>

//  PIN DEFINITIONS 

// Front H-Bridge 
#define ENA1 5     
#define IN1 22
#define IN2 23
#define ENB1 6     
#define IN3 24
#define IN4 25

// --- Rear H-Bridge ---
#define ENA2 7    
#define IN5 26
#define IN6 27
#define ENB2 8   
#define IN7 28
#define IN8 29

// --- Ultrasonic Sensors ---
#define TRIG_FRONT_LEFT   30
#define ECHO_FRONT_LEFT   31
#define TRIG_FRONT_RIGHT  32
#define ECHO_FRONT_RIGHT  33
#define TRIG_LEFT         34
#define ECHO_LEFT         35
#define TRIG_RIGHT        36
#define ECHO_RIGHT        37

// Enes100 Serial (TX/RX)
#define ENES_TX 10
#define ENES_RX 11
SoftwareSerial enesSerial(ENES_TX, ENES_RX);

// Constants
const float FRONT_THRESH = 0.25;     
const float SIDE_CLEAR_DIST = 1.0;   
const float SIDE_CLOSE_DIST = 0.3;   
const float FOLLOW_DIST = 0.18;      
const int BASE_SPEED = 30;
const float Kp = 62.5;              

// STATE DEFINITIONS
enum Orientation { EAST, NORTH, SOUTH };
enum DriveState  { FORWARD, ALONG_OBSTACLE, TURN_AROUND_OBSTACLE, OPEN_ZONE };

Orientation facing = EAST;
DriveState state = FORWARD;

// FUNCTION PROTOTYPES 
void driveSetup();
void driveLoop();
void moveStraight();
void setMotorPWM(int leftPWM, int rightPWM);
void stopMotors();
void turnLeft90();
void turnRight90();
void turn180();
void turnToEast();
float readDistance(int trigPin, int echoPin);
float getFrontDistance();
float getSideDistance(bool left);
void followUsingSide(bool leftSensor);

// SETUP
void driveSetup() {
  int pins[] = {IN1, IN2, IN3, IN4, IN5, IN6, IN7, IN8, ENA1, ENB1, ENA2, ENB2};
  for (int i = 0; i < 12; i++) pinMode(pins[i], OUTPUT);

  int trigPins[] = {TRIG_FRONT_LEFT, TRIG_FRONT_RIGHT, TRIG_LEFT, TRIG_RIGHT};
  int echoPins[] = {ECHO_FRONT_LEFT, ECHO_FRONT_RIGHT, ECHO_LEFT, ECHO_RIGHT};
  for (int i = 0; i < 4; i++) {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
  }

  Serial.begin(9600);
  enesSerial.begin(9600);

  Enes100.begin("Material Mermaids", MATERIAL, 424, 1116, 12, 13);
  Enes100.println("Drive system initialized");

  Serial.println("Drive setup complete");
}

// LOOP
void driveLoop() {
  float x = Enes100.getX();
  float y = Enes100.getY();

  Serial.print("Position: x=");
  Serial.print(x);
  Serial.print(" y=");
  Serial.println(y);

  if (x >= 3.4 && fabs(y - 1.5) < 0.2) {
    stopMotors();
    Enes100.println("Goal reached!");
    Serial.println("Goal reached. Motors stopped");
    return;
  }

  if (x >= 2.8) {
    float targetY = 1.5;
    float errorY = targetY - y;
    float correction = Kp * errorY;
    setMotorPWM(BASE_SPEED - correction, BASE_SPEED + correction);
    Serial.println("Open zone override reached");
    return;
  }

  float frontDist = getFrontDistance();
  float leftDist  = getSideDistance(true);
  float rightDist = getSideDistance(false);

  Serial.print("Sensor distance (m): front=");
  Serial.print(frontDist);
  Serial.print(" left=");
  Serial.print(leftDist);
  Serial.print(" right=");
  Serial.println(rightDist);

  switch (state) {

    case FORWARD:
      Serial.println("STATE: FORWARD");
      if (frontDist > FRONT_THRESH) {
        moveStraight();
      } else {
        Serial.println("Obstacle detected ahead");
        if (y < 1.0) {
          facing = NORTH;
          turnLeft90();
        } else {
          facing = SOUTH;
          turnRight90();
        }
        state = ALONG_OBSTACLE;
        Serial.println("New state: ALONG_OBSTACLE");
      }
      break;

    case ALONG_OBSTACLE: {
      Serial.println("State: ALONG_OBSTACLE");

      bool usingLeft = (facing == SOUTH);
      bool usingRight = (facing == NORTH);

      float sideDist = usingLeft ? leftDist : rightDist;

      followUsingSide(usingLeft);

      if (frontDist < FRONT_THRESH && sideDist < SIDE_CLOSE_DIST) {
        Enes100.println("Trap detected: turning 180");
        Serial.println("Performing 180 degree turn");
        turn180();
        facing = (facing == NORTH) ? SOUTH : NORTH;
      }

      if (sideDist > SIDE_CLEAR_DIST) {
        Enes100.println("Obstacle cleared, turning east");
        Serial.println("Side distance cleared. Turning to EAST");
        turnToEast();
        facing = EAST;
        state = FORWARD;
      }
      break;
    }

    default:
      moveStraight();
      break;
  }
}

void moveStraight() {
  Serial.println("MOVING STRAIGHT");
  setMotorPWM(BASE_SPEED, BASE_SPEED);
}

void setMotorPWM(int leftPWM, int rightPWM) {
  leftPWM = constrain(leftPWM, -255, 255);
  rightPWM = constrain(rightPWM, -255, 255);

  Serial.print("Motor PWM set: L=");
  Serial.print(leftPWM);
  Serial.print(" R=");
  Serial.println(rightPWM);

  digitalWrite(IN1, leftPWM > 0);
  digitalWrite(IN2, leftPWM < 0);
  digitalWrite(IN5, leftPWM > 0);
  digitalWrite(IN6, leftPWM < 0);
  analogWrite(ENA1, abs(leftPWM));
  analogWrite(ENA2, abs(leftPWM));

  digitalWrite(IN3, rightPWM > 0);
  digitalWrite(IN4, rightPWM < 0);
  digitalWrite(IN7, rightPWM > 0);
  digitalWrite(IN8, rightPWM < 0);
  analogWrite(ENB1, abs(rightPWM));
  analogWrite(ENB2, abs(rightPWM));
}

void stopMotors() {
  Serial.println("Stopping motors");
  setMotorPWM(0, 0);
}

void turnLeft90() {
  Serial.println("Turning left 90 degrees");
  Enes100.println("Turning left 90°");
  setMotorPWM(-BASE_SPEED, BASE_SPEED);
  delay(700);
  stopMotors();
}

void turnRight90() {
  Serial.println("Turning right 90 degrees");
  Enes100.println("Turning right 90°");
  setMotorPWM(BASE_SPEED, -BASE_SPEED);
  delay(700);
  stopMotors();
}

void turn180() {
  Enes100.println("Turning 180°");
  Serial.println("Turning 180 degrees");
  setMotorPWM(BASE_SPEED, -BASE_SPEED);
  delay(1400);
  stopMotors();
}

void turnToEast() {
  Enes100.println("Turning back east");
  Serial.println("Turning east");
  if (facing == NORTH) turnRight90();
  else if (facing == SOUTH) turnLeft90();
  facing = EAST;
}

// FIXED readDistance()
float readDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);
  float dist = (duration == 0) ? -1 : duration * 0.000343 / 2.0;

  Serial.print("Ultrasonic ");
  Serial.print(echoPin);
  Serial.print(" dist = ");
  Serial.println(dist);

  delay(20);
  return dist;
}

float getFrontDistance() {
  float dL = readDistance(TRIG_FRONT_LEFT, ECHO_FRONT_LEFT);
  float dR = readDistance(TRIG_FRONT_RIGHT, ECHO_FRONT_RIGHT);
  if (dL == -1 && dR == -1) return -1;
  else if (dL == -1) return dR;
  else if (dR == -1) return dL;
  return min(dL, dR);
}

float getSideDistance(bool left) {
  return left ? readDistance(TRIG_LEFT, ECHO_LEFT)
              : readDistance(TRIG_RIGHT, ECHO_RIGHT);
}

void followUsingSide(bool leftSensor) {
  float dist = leftSensor ? getSideDistance(true) : getSideDistance(false);

  if (dist == -1) {
    moveStraight();
    return;
  }

  float error = dist - FOLLOW_DIST;
  float correction = Kp * error;

  int leftPWM = BASE_SPEED - correction;
  int rightPWM = BASE_SPEED + correction;

  Serial.print("Following side: dist=");
  Serial.println(dist);

  setMotorPWM(leftPWM, rightPWM);
}

void setup() {
  driveSetup();
}

void loop() {
  driveLoop();
}
