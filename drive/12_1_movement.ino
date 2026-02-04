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
const int TURN_SPEED = 140;                // pivot speed
const float TARGET_THETA = 0.0;           // target orientation in radians
const float THETA_TOLERANCE = 0.05;        // acceptable range (~2.8°)
const int REQUIRED_CONSECUTIVE_OK = 2;     // loops to meet tolerance
const float OBSTACLE_THRESHOLD = 0.3;      // threshold for obstacle in meters

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

void pivotInPlace(float thetaError) {
    if (thetaError > 0) {
        // rotate left
        setMotorPWM(TURN_SPEED, TURN_SPEED, false, true);
    } else {
        // rotate right
        setMotorPWM(TURN_SPEED, TURN_SPEED, true, false);
    }
}

void moveForward() {
    setMotorPWM(BASE_SPEED, BASE_SPEED, true, true);
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

// Dummy / placeholder for obstacle detection — replace with real sensor code
float readObstacleDistance() {
    // ** TODO **: replace with actual distance sensor reading
    // For now just return a large value (no obstacle)
    return 999.0;
}

// ---------------- SETUP & LOOP ----------------
void setup() {
    int motorPins[] = {IN1, IN2, IN3, IN4, IN5, IN6, IN7, IN8, ENA1, ENB1, ENA2, ENB2};
    for (int i = 0; i < 12; i++) pinMode(motorPins[i], OUTPUT);

    Serial.begin(9600);
    Enes100.begin("Material_Mermaids", MATERIAL, 500, 1116, 50, 51);

    // Wait until visible
    while (!Enes100.isVisible()) {
        Serial.println("Waiting for visibility...");
        delay(50);
    }

    Serial.println("READY.");
}

void loop() {
    if (!Enes100.isVisible()) {
        stopMotors();
        Serial.println("Not visible → stopping.");
        delay(50);
        return;
    }

    // Read and print obstacle distance every loop
    float dist = readObstacleDistance();
    Serial.print("Distance reading = ");
    Serial.println(dist);

    float x = Enes100.getX();

    // Obstacle detection / avoidance
    if (dist <= OBSTACLE_THRESHOLD) {
        stopMotors();
        Serial.print("Obstacle detected (dist <= ");
        Serial.print(OBSTACLE_THRESHOLD);
        Serial.println("). Avoiding...");

        if (x <= 1.0f) {
            Serial.println("X ≤ 1.0 → turning LEFT to avoid obstacle");
            setMotorPWM(TURN_SPEED, TURN_SPEED, false, true);
        } else {
            Serial.println("X > 1.0 → turning RIGHT to avoid obstacle");
            setMotorPWM(TURN_SPEED, TURN_SPEED, true, false);
        }

        delay(50);
        return;  // skip orientation + forward logic this loop
    }

    // No obstacle → orientation correction + forward motion
    float thetaError = getThetaError();

    if (isThetaAcceptable(thetaError)) {
        thetaOKCounter++;
        moveForward();
    } else {
        thetaOKCounter = 0;
        pivotInPlace(thetaError);
    }

    delay(1);
}
