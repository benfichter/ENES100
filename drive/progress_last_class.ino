 #include <Arduino.h>
 #include "Enes100.h"
 #include <math.h>
 
 // ---------------- MOTOR PINS ----------------
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
 
 // ---------------- ULTRASONIC PINS ----------------
 #define TRIG_FRONT_LEFT 52
 #define ECHO_FRONT_LEFT 53
 #define TRIG_FRONT_RIGHT 32
 #define ECHO_FRONT_RIGHT 33
 #define TRIG_LEFT_SIDE 40
 #define ECHO_LEFT_SIDE 41
 #define TRIG_RIGHT_SIDE 42
 #define ECHO_RIGHT_SIDE 43
 
 // ---------------- PARAMETERS ----------------
 const int BASE_SPEED = 120;
 const int TURN_SPEED = 120;
 
 const float TARGET_THETA = 0.0;
 const float THETA_TOLERANCE = 0.05;
 
 const float OBSTACLE_THRESHOLD = 0.25; // front sensor detection
 const float OBSTACLE_CLEAR_DISTANCE = 0.20; // side sensor clearance
 const float MIN_VALID_READING = 0.05;
 
 // ---------------- STATE ----------------
 int obstacleFrontCounter = 0;
 float desiredTheta = 0.0;
 boolean turn_parallel = false;
 
 // ---------------- FSM ----------------
 enum ObstacleState {
 NORMAL_DRIVING,
 TURN_TO_PARALLEL,
 FOLLOW_OBSTACLE
 };
 ObstacleState obsState = NORMAL_DRIVING;
 
 // ---------------- MOTOR CONTROL ----------------
 void setMotorPWM(int leftPWM, int rightPWM, bool leftForward=true, bool rightForward=true) {
 leftPWM = constrain(leftPWM, 0, 255);
 rightPWM = constrain(rightPWM, 0, 255);
 
 digitalWrite(IN1, leftForward ? HIGH : LOW);
 digitalWrite(IN2, leftForward ? LOW : HIGH);
 digitalWrite(IN5, leftForward ? HIGH : LOW);
 digitalWrite(IN6, leftForward ? LOW : HIGH);
 analogWrite(ENA1, leftPWM);
 analogWrite(ENA2, leftPWM);
 
 digitalWrite(IN3, rightForward ? LOW : HIGH);
 digitalWrite(IN4, rightForward ? HIGH : LOW);
 digitalWrite(IN7, rightForward ? LOW : HIGH);
 digitalWrite(IN8, rightForward ? HIGH : LOW);
 analogWrite(ENB1, rightPWM);
 analogWrite(ENB2, rightPWM);
 }
 
 void stopMotors() {
 setMotorPWM(0, 0);
 }
 
 void pivotInPlace(float thetaError) {
 if (thetaError > 0)
 setMotorPWM(TURN_SPEED, TURN_SPEED, false, true); // turn left
 else
 setMotorPWM(TURN_SPEED, TURN_SPEED, true, false); // turn right
 }
 
 void moveForward() {
 setMotorPWM(BASE_SPEED, BASE_SPEED, true, true);
 }
 
 // ---------------- ANGLE FUNCTIONS ----------------
 float normalizeAngle(float angle) {
 while (angle > M_PI) angle -= 2 * M_PI;
 while (angle < -M_PI) angle += 2 * M_PI;
 return angle;
 }
 
 float getThetaErrorTo(float target) {
 return normalizeAngle(target - Enes100.getTheta());
 }
 
 bool isThetaAcceptable(float err, float tolerance) {
 return fabs(err) <= tolerance;
 }
 
 // ---------------- ULTRASONIC FUNCTIONS ----------------
 float readUltrasonicDistance(int trigPin, int echoPin) {
 digitalWrite(trigPin, LOW); delayMicroseconds(2);
 digitalWrite(trigPin, HIGH); delayMicroseconds(10);
 digitalWrite(trigPin, LOW);
 
 long duration = pulseIn(echoPin, HIGH, 30000);
 if (duration == 0) return -1.0;
 
 return (duration * 0.0343 / 2.0) / 100.0; // meters
 }
 
 float getFrontDistance() {
 float dL = readUltrasonicDistance(TRIG_FRONT_LEFT, ECHO_FRONT_LEFT);
 float dR = readUltrasonicDistance(TRIG_FRONT_RIGHT, ECHO_FRONT_RIGHT);
 
 if (dL > 0 && dL < MIN_VALID_READING) dL = -1;
 if (dR > 0 && dR < MIN_VALID_READING) dR = -1;
 
 if (dL < 0 && dR < 0) return -1;
 if (dL < 0) return dR;
 if (dR < 0) return dL;
 
 return min(dL, dR);
 }
 
 float getLeftSideDistance() { return readUltrasonicDistance(TRIG_LEFT_SIDE, ECHO_LEFT_SIDE); }
 float getRightSideDistance() { return readUltrasonicDistance(TRIG_RIGHT_SIDE, ECHO_RIGHT_SIDE); }
 
 // ---------------- SETUP ----------------
 void setup() {
 int pins[] = {IN1,IN2,IN3,IN4,IN5,IN6,IN7,IN8,ENA1,ENA2,ENB1,ENB2};
 for(int i=0;i<12;i++) pinMode(pins[i], OUTPUT);
 
 pinMode(TRIG_FRONT_LEFT, OUTPUT); pinMode(ECHO_FRONT_LEFT, INPUT);
 pinMode(TRIG_FRONT_RIGHT, OUTPUT); pinMode(ECHO_FRONT_RIGHT, INPUT);
 pinMode(TRIG_LEFT_SIDE, OUTPUT); pinMode(ECHO_LEFT_SIDE, INPUT);
 pinMode(TRIG_RIGHT_SIDE, OUTPUT); pinMode(ECHO_RIGHT_SIDE, INPUT);
 
 Serial.begin(9600);
 
 Enes100.begin("Material_Mermaids", MATERIAL, 500, 1116, 50, 51);
 while(!Enes100.isVisible()) delay(50);
 
 stopMotors();
 }
 
 // ---------------- LOOP ----------------
 void loop() {
 if(!Enes100.isVisible()){
 stopMotors();
 return;
 }
 
 float distFront = getFrontDistance();
 float x = Enes100.getX();
 
 switch(obsState) {
 
 // ---------------- NORMAL DRIVING ----------------
 case NORMAL_DRIVING: {
 float err = getThetaErrorTo(TARGET_THETA);
 
 if(distFront > 0 && distFront <= OBSTACLE_THRESHOLD)
 obstacleFrontCounter++;
 else
 obstacleFrontCounter = 0;
 
 if(obstacleFrontCounter >= 3){
 // Quadrant-based turn
 if(x <= 1.0f) desiredTheta = -M_PI/2; // left → turn right
 else desiredTheta = M_PI/2; // right → turn left
 
 obsState = TURN_TO_PARALLEL;
 break;
 }
 
 if(!isThetaAcceptable(err, 0.2)) pivotInPlace(err);
 else moveForward();
 } break;
 
 // ---------------- TURN TO PARALLEL ----------------
 case TURN_TO_PARALLEL: {
 desiredTheta = (x <= 1.0f) ? -M_PI/2 : M_PI/2; // quadrant-based
 setMotorPWM(TURN_SPEED, TURN_SPEED, desiredTheta > 0, desiredTheta < 0); // spin one way
 delay(1000); // pivot for fixed time
 turn_parallel = true;
 obsState = FOLLOW_OBSTACLE; // then start moving
 } break;
 
 // ---------------- FOLLOW OBSTACLE ----------------
 case FOLLOW_OBSTACLE: {
 if (turn_parallel){
 moveForward(); // go straight along obstacle
 
 float sideDist = (desiredTheta > 0) ? getRightSideDistance() : getLeftSideDistance();
 if(sideDist > OBSTACLE_CLEAR_DISTANCE || sideDist < 0){
 obstacleFrontCounter = 0;
 obsState = NORMAL_DRIVING;
 }
 } else {
    obsState = NORMAL_DRIVING;
 }
 } break;
 }
 
 delay(10);
 }
