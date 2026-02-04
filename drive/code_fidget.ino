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
const int TURN_SPEED = 140;
float TARGET_THETA = 0.0;                     // set after reading start Y
const float THETA_TOLERANCE = 0.05;           // for self-correction
const int REQUIRED_CONSECUTIVE_OK = 2;

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

// ---------------- TURN UNTIL FUNCTION ----------------
void turnUntil(float targetTheta) {
    const float TOL = 0.15;   // ~8.6 degrees, lenient tolerance
    int turnPWM = TURN_SPEED;

    while (true) {
        float theta = Enes100.getTheta();
        float error = targetTheta - theta;

        // Normalize error into [-π, π]
        while (error >  M_PI) error -= 2 * M_PI;
        while (error < -M_PI) error += 2 * M_PI;

        // Stop if within tolerance
        if (fabs(error) <= TOL)
            break;

        // Decide turning direction
        if (error > 0)
            setMotorPWM(turnPWM, turnPWM, false, true);   // turn left
        else
            setMotorPWM(turnPWM, turnPWM, true, false);   // turn right

        delay(10);
    }

    stopMotors();
    delay(100);
}

// ---------------- ANGLE FUNCTIONS ----------------
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

// ---------------- SETUP ----------------
void setup() {
    int motorPins[] = {
        IN1, IN2, IN3, IN4, IN5, IN6, IN7, IN8,
        ENA1, ENB1, ENA2, ENB2
    };
    for (int i = 0; i < 12; i++) pinMode(motorPins[i], OUTPUT);

    Serial.begin(9600);
    Enes100.begin("Material_Mermaids", MATERIAL, 500, 1116, 50, 51);

    while (!Enes100.isVisible()) {
        Serial.println("Waiting for visibility...");
        delay(50);
    }

    Serial.println("READY.");

    // ---------- STARTUP TURN BASED ON INITIAL Y ----------
    float startY = Enes100.getY();
    Serial.print("Starting Y = "); Serial.println(startY);

    if (startY > 1.0) {
        Serial.println("Start Y > 1 → TURN SOUTH (-π/2)");
        TARGET_THETA = -M_PI / 2.0;
        turnUntil(TARGET_THETA);
    } else {
        Serial.println("Start Y < 1 → TURN NORTH (+π/2)");
        TARGET_THETA = M_PI / 2.0;
        turnUntil(TARGET_THETA);
    }

    // Immediately start driving forward
    setMotorPWM(BASE_SPEED, BASE_SPEED, true, true);
    delay(200);
}

// ---------------- LOOP ----------------
void loop() {
    if (!Enes100.isVisible()) {
        stopMotors();
        Serial.println("Not visible — stopping");
        delay(50);
        return;
    }

    float thetaError = getThetaError();
    Serial.print("ThetaErr: "); Serial.println(thetaError);

    // ----------- ORIENTATION SELF-CORRECTION + FORWARD -----------
    if (!isThetaAcceptable(thetaError)) {
        if (thetaError > 0)
            setMotorPWM(TURN_SPEED, TURN_SPEED, false, true);   // turn left
        else
            setMotorPWM(TURN_SPEED, TURN_SPEED, true, false);   // turn right
    } else {
        setMotorPWM(BASE_SPEED, BASE_SPEED, true, true);        // drive forward
    }

    delay(10);
}
