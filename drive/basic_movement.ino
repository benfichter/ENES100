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
const int BASE_SPEED = 150;                // forward motion speed
const int TURN_SPEED = 130;                // prev speed 140
const float TARGET_THETA = 0.0;           // target orientation in radians
const float THETA_TOLERANCE = 0.05;       // acceptable range (~2.8°)
const int REQUIRED_CONSECUTIVE_OK = 2;    // must meet tolerance this many loops

// ---------------- STATE ----------------
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

// Pivot around center
void pivotInPlace(float thetaError) {
    if (thetaError > 0) {
        // rotate left
        setMotorPWM(TURN_SPEED, TURN_SPEED, false, true);
    } else {
        // rotate right
        setMotorPWM(TURN_SPEED, TURN_SPEED, true, false);
    }
}

// Forward motion
void moveForward() {
    setMotorPWM(BASE_SPEED, BASE_SPEED, true, true);
}

// ---------------- SETUP & LOOP ----------------
void setup() {
    int motorPins[] = {IN1, IN2, IN3, IN4, IN5, IN6, IN7, IN8, ENA1, ENB1, ENA2, ENB2};
    for (int i = 0; i < 12; i++) pinMode(motorPins[i], OUTPUT);

    Serial.begin(9600);
    Enes100.begin("Material_Mermaids", MATERIAL, 500, 1116, 50, 51);

    // wait for visibility
    while (!Enes100.isVisible()) {
        Serial.println("Waiting for visibility...");
        delay(50);
    }

    Serial.println("Starting orientation correction...");
}

void loop() {
    if (!Enes100.isVisible()) {
        stopMotors();
        Serial.println("Not visible → stopping!");
        delay(50);
        return;
    }

    float thetaError = getThetaError();

    if (isThetaAcceptable(thetaError)) {
        thetaOKCounter++;
        moveForward();  // move forward while orientation is okay

        if (thetaOKCounter >= REQUIRED_CONSECUTIVE_OK) {
            Serial.println("Orientation stable, moving forward...");
        }
    } else {
        thetaOKCounter = 0;
        pivotInPlace(thetaError);  // correct orientation before moving
    }

    delay(1);  // short delay for precise pivoting
}
