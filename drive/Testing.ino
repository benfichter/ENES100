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
#define TRIG_FRONT_LEFT   52
#define ECHO_FRONT_LEFT   53
#define TRIG_FRONT_RIGHT  32
#define ECHO_FRONT_RIGHT  33
#define TRIG_LEFT_SIDE    40
#define ECHO_LEFT_SIDE    41
#define TRIG_RIGHT_SIDE   42
#define ECHO_RIGHT_SIDE   43

// ---------------- PARAMETERS ----------------
const int BASE_SPEED = 150;
const int TURN_SPEED = 140;

const float TARGET_THETA = 0.0;
const float THETA_TOLERANCE = 0.05;
const float TURN_TOLERANCE  = 0.15;

const float OBSTACLE_THRESHOLD = 0.30;
const float CLEAR_SIDE_THRESHOLD = 0.30;  // side sensor > 30cm → obstacle cleared
const float MIN_VALID_READING = 0.05;

// ---------------- STATE MACHINE ----------------
enum RobotState {
    NORMAL_DRIVING,
    TURN_TO_PARALLEL,
    DRIVE_ALONG_OBSTACLE,
    TURN_BACK_TO_ZERO
};
RobotState state = NORMAL_DRIVING;

float desiredTheta = 0.0;

// ---------------- MOTOR FUNCTIONS ----------------
void setMotorPWM(int leftPWM, int rightPWM, bool leftForward = true, bool rightForward = true) {
    leftPWM  = constrain(leftPWM, 0, 255);
    rightPWM = constrain(rightPWM, 0, 255);

    digitalWrite(IN1, leftForward ? HIGH : LOW);
    digitalWrite(IN2, leftForward ? LOW  : HIGH);
    digitalWrite(IN5, leftForward ? HIGH : LOW);
    digitalWrite(IN6, leftForward ? LOW  : HIGH);
    analogWrite(ENA1, leftPWM);
    analogWrite(ENA2, leftPWM);

    digitalWrite(IN3, rightForward ? LOW  : HIGH);
    digitalWrite(IN4, rightForward ? HIGH : LOW );
    digitalWrite(IN7, rightForward ? LOW  : HIGH);
    digitalWrite(IN8, rightForward ? HIGH : LOW );
    analogWrite(ENB1, rightPWM);
    analogWrite(ENB2, rightPWM);
}

void stopMotors() {
    setMotorPWM(0, 0);
}

void moveForward() {
    setMotorPWM(BASE_SPEED, BASE_SPEED, true, true);
}

void pivotInPlace(float error) {
    if (error > 0)
        setMotorPWM(TURN_SPEED, TURN_SPEED, false, true);  // turn left
    else
        setMotorPWM(TURN_SPEED, TURN_SPEED, true, false);  // turn right
}

// ---------------- ANGLE HELPERS ----------------
float normalizeAngle(float a) {
    while (a >  M_PI) a -= 2 * M_PI;
    while (a < -M_PI) a += 2 * M_PI;
    return a;
}

float getThetaError(float target) {
    return normalizeAngle(target - Enes100.getTheta());
}

bool angleOK(float err, float tol) {
    return fabs(err) <= tol;
}

// ---------------- ULTRASONIC ----------------
float readUltrasonicDistance(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW); delayMicroseconds(2);
    digitalWrite(trigPin, HIGH); delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH, 30000);
    if (duration == 0) return -1.0;

    return (duration * 0.0343 / 2.0) / 100.0;
}

float getFrontDistance() {
    float dL = readUltrasonicDistance(TRIG_FRONT_LEFT,  ECHO_FRONT_LEFT);
    float dR = readUltrasonicDistance(TRIG_FRONT_RIGHT, ECHO_FRONT_RIGHT);

    if (dL > 0 && dL < MIN_VALID_READING) dL = -1;
    if (dR > 0 && dR < MIN_VALID_READING) dR = -1;

    if (dL < 0 && dR < 0) return -1;
    if (dL < 0) return dR;
    if (dR < 0) return dL;

    return min(dL, dR);
}

// ---------------- SETUP ----------------
void setup() {
    int pins[] = {IN1,IN2,IN3,IN4,IN5,IN6,IN7,IN8,ENA1,ENA2,ENB1,ENB2};
    for (int i = 0; i < 12; i++) pinMode(pins[i], OUTPUT);

    pinMode(TRIG_FRONT_LEFT, OUTPUT);  pinMode(ECHO_FRONT_LEFT, INPUT);
    pinMode(TRIG_FRONT_RIGHT, OUTPUT); pinMode(ECHO_FRONT_RIGHT, INPUT);

    pinMode(TRIG_LEFT_SIDE, OUTPUT);   pinMode(ECHO_LEFT_SIDE, INPUT);
    pinMode(TRIG_RIGHT_SIDE, OUTPUT);  pinMode(ECHO_RIGHT_SIDE, INPUT);

    Serial.begin(9600);

    Enes100.begin("Material_Mermaids", MATERIAL, 500, 1116, 50, 51);
    while (!Enes100.isVisible()) delay(50);

    stopMotors();
}

// ---------------- LOOP ----------------
void loop() {

    if (!Enes100.isVisible()) {
        stopMotors();
        return;
    }

    float distFront = getFrontDistance();
    float x = Enes100.getX();
    float err;

    switch (state) {

    // -------------------------------------------------
    // NORMAL DRIVING
    // -------------------------------------------------
    case NORMAL_DRIVING: {
        err = getThetaError(TARGET_THETA);

        if (distFront > 0 && distFront <= OBSTACLE_THRESHOLD) {
            stopMotors();

            if (x > 1.0) desiredTheta =  M_PI / 2;  // TURN LEFT
            else         desiredTheta = -M_PI / 2;  // TURN RIGHT

            desiredTheta = normalizeAngle(desiredTheta);
            state = TURN_TO_PARALLEL;
            break;
        }

        if (!angleOK(err, THETA_TOLERANCE))
            pivotInPlace(err);
        else
            moveForward();

    } break;

    // -------------------------------------------------
    // TURN UNTIL ±π/2
    // -------------------------------------------------
    case TURN_TO_PARALLEL: {
        err = getThetaError(desiredTheta);

        if (!angleOK(err, TURN_TOLERANCE))
            pivotInPlace(err);
        else {
            stopMotors();
            state = DRIVE_ALONG_OBSTACLE;
        }

    } break;

    // -------------------------------------------------
    // DRIVE STRAIGHT ALONG OBSTACLE (SIMPLE VERSION)
    // -------------------------------------------------
    case DRIVE_ALONG_OBSTACLE: {

        moveForward();  // SIMPLE VERSION — no angle correction

        float sideDist;

        // If robot turned LEFT (+π/2), use RIGHT side sensor
        if (desiredTheta > 0)
            sideDist = readUltrasonicDistance(TRIG_RIGHT_SIDE, ECHO_RIGHT_SIDE);
        else
            sideDist = readUltrasonicDistance(TRIG_LEFT_SIDE, ECHO_LEFT_SIDE);

        if (sideDist > CLEAR_SIDE_THRESHOLD && sideDist > 0) {
            stopMotors();
            state = TURN_BACK_TO_ZERO;
        }

    } break;

    // -------------------------------------------------
    // TURN BACK TO HEADING 0
    // -------------------------------------------------
    case TURN_BACK_TO_ZERO: {
        err = getThetaError(0.0);

        if (!angleOK(err, TURN_TOLERANCE))
            pivotInPlace(err);
        else {
            stopMotors();
            state = NORMAL_DRIVING;
        }

    } break;
    }

    delay(10);
}
