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

// ---------------- PARAMETERS ----------------
const int BASE_SPEED = 150;
const int TURN_SPEED = 125;               // reduced pivot speed for finer turning
const float TARGET_THETA = M_PI / 2;     // 90 degrees
const float THETA_TOLERANCE = 0.05;      // ~2.8 deg tolerance
const int REQUIRED_CONSECUTIVE_OK = 2;   // must be within tolerance this many loops
const float MOVE_DISTANCE = 0.1;        // forward displacement in meters

// ---------------- STATE ----------------
float startX = 0.0;
float targetX = 0.0;
bool reachedX = false;
int thetaOKCounter = 0;

// ---------------- MOTOR CONTROL ----------------
void setMotorPWM(int leftPWM, int rightPWM, bool leftForward=true, bool rightForward=true) {
    leftPWM  = constrain(leftPWM, 0, 255);
    rightPWM = constrain(rightPWM, 0, 255);

    // LEFT MOTORS
    digitalWrite(IN1, leftForward ? HIGH : LOW);
    digitalWrite(IN2, leftForward ? LOW : HIGH);
    digitalWrite(IN5, leftForward ? HIGH : LOW);
    digitalWrite(IN6, leftForward ? LOW : HIGH);
    analogWrite(ENA1, leftPWM);
    analogWrite(ENA2, leftPWM);

    // RIGHT MOTORS
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

// ---------------- ANGLE CONTROL ----------------
float normalizeAngle(float angle) {
    while (angle >  M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

float getThetaError() {
    return normalizeAngle(TARGET_THETA - Enes100.getTheta());
}

bool isThetaAcceptable(float thetaError) {
    return fabs(thetaError) <= THETA_TOLERANCE;
}

// Pivot in place around center
void pivotInPlace(float thetaError) {
    if (thetaError > 0) {
        // Left backward, right forward → rotate left
        setMotorPWM(TURN_SPEED, TURN_SPEED, false, true);
    } else {
        // Left forward, right backward → rotate right
        setMotorPWM(TURN_SPEED, TURN_SPEED, true, false);
    }
}

// ---------------- FORWARD MOTION ----------------
void moveForward() {
    setMotorPWM(BASE_SPEED, BASE_SPEED, true, true);
}

void driveDistance() {
    if (!Enes100.isVisible()) {
        stopMotors();
        return;
    }

    float x = Enes100.getX();
    if (x <= targetX) {
        stopMotors();
        reachedX = true;
        Serial.println("Reached distance goal.");
        return;
    }

    float thetaError = getThetaError();
    if (!isThetaAcceptable(thetaError)) {
        thetaOKCounter = 0;
        pivotInPlace(thetaError);  // correct heading while moving
    } else {
        moveForward();
    }
}

// ---------------- SETUP & LOOP ----------------
void setup() {
    int motorPins[] = {IN1, IN2, IN3, IN4, IN5, IN6, IN7, IN8, ENA1, ENB1, ENA2, ENB2};
    for (int i = 0; i < 12; i++) pinMode(motorPins[i], OUTPUT);

    Serial.begin(9600);
    Enes100.begin("Material_Mermaids", MATERIAL, 500, 1116, 50, 51);

    // Wait until visible to read starting X
    while (!Enes100.isVisible()) {
        Serial.println("Waiting for visibility...");
        delay(50);
    }

    startX = Enes100.getX();
    targetX = startX - MOVE_DISTANCE;
    Serial.print("Start X = "); Serial.println(startX);
    Serial.print("Target X = "); Serial.println(targetX);
    Serial.println("READY.");
}

void loop() {
    if (reachedX) {
        stopMotors();
        Serial.println("DONE (holding).");
        delay(50);
        return;
    }

    if (!Enes100.isVisible()) {
        stopMotors();
        Serial.println("Not visible → stopping.");
        delay(50);
        return;
    }

    float thetaError = getThetaError();

    if (isThetaAcceptable(thetaError)) {
        thetaOKCounter++;
        if (thetaOKCounter >= REQUIRED_CONSECUTIVE_OK) {
            driveDistance();
        } else {
            stopMotors(); // wait until heading is stable
        }
    } else {
        thetaOKCounter = 0;
        pivotInPlace(thetaError);
    }

    delay(2); // very short delay for precise rotation
}
