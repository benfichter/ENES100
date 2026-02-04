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
// Adjust these pins to match your wiring
#define TRIG_FRONT_LEFT   52
#define ECHO_FRONT_LEFT   53
#define TRIG_FRONT_RIGHT  30
#define ECHO_FRONT_RIGHT  31

// ---------------- PARAMETERS ----------------
const int BASE_SPEED = 150;               // forward motion speed
const int TURN_SPEED = 140;               // pivot speed
const float TARGET_THETA = 0.0;           // target orientation in radians
const float THETA_TOLERANCE = 0.05;       // acceptable angle tolerance (~2.8°)
const int REQUIRED_CONSECUTIVE_OK = 2;    // loops to confirm stable orientation
const float OBSTACLE_THRESHOLD = 0.3;     // obstacle threshold (meters)

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

// ---------------- ULTRASONIC DISTANCE FUNCTIONS ----------------
float readUltrasonicDistance(int trigPin, int echoPin) {
    // send pulse
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH, 30000); // 30 ms timeout
    if (duration == 0) {
        return -1.0; // invalid / no echo
    }
    // speed of sound ~ 343 m/s => 0.0343 cm/µs
    // distance in meters: (duration * 0.0343 / 2) / 100
    float distance_m = (duration * 0.0343 / 2.0) / 100.0;
    return distance_m;
}

float getFrontDistance() {
    float dL = readUltrasonicDistance(TRIG_FRONT_LEFT, ECHO_FRONT_LEFT);
    float dR = readUltrasonicDistance(TRIG_FRONT_RIGHT, ECHO_FRONT_RIGHT);
    if (dL < 0 && dR < 0) return -1.0;
    if (dL < 0) return dR;
    if (dR < 0) return dL;
    return min(dL, dR);
}

// ---------------- SETUP & LOOP ----------------
void setup() {
    int motorPins[] = {IN1, IN2, IN3, IN4, IN5, IN6, IN7, IN8, ENA1, ENB1, ENA2, ENB2};
    for (int i = 0; i < 12; i++) pinMode(motorPins[i], OUTPUT);

    pinMode(TRIG_FRONT_LEFT, OUTPUT);
    pinMode(ECHO_FRONT_LEFT, INPUT);
    pinMode(TRIG_FRONT_RIGHT, OUTPUT);
    pinMode(ECHO_FRONT_RIGHT, INPUT);

    Serial.begin(9600);
    Enes100.begin("Material_Mermaids", MATERIAL, 500, 1116, 50, 51);

    // Wait until vision system reports visible
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

    float distFront = getFrontDistance();
    float x = Enes100.getX();
    float thetaError = getThetaError();

    // debug print
    Serial.print("FrontDist(m): "); Serial.print(distFront);
    Serial.print("  X: "); Serial.print(x);
    Serial.print("  ThetaErr(rad): "); Serial.println(thetaError);

    // ---------------- OBSTACLE CHECK & AVOIDANCE ----------------
    if (distFront > 0 && distFront <= OBSTACLE_THRESHOLD) {
        stopMotors();
        Serial.println("Obstacle detected — performing avoidance turn.");

        if (x <= 1.0f) {
            Serial.println("X ≤ 1.0 → turning LEFT to avoid obstacle");
            setMotorPWM(TURN_SPEED, TURN_SPEED, false, true);
        } else {
            Serial.println("X > 1.0 → turning RIGHT to avoid obstacle");
            setMotorPWM(TURN_SPEED, TURN_SPEED, true, false);
        }

        delay(200);  // give some time for the turn
        return;     // skip orientation/forward this loop
    }

    // ---------------- ORIENTATION CORRECTION + FORWARD ----------------
    if (!isThetaAcceptable(thetaError)) {
        thetaOKCounter = 0;
        pivotInPlace(thetaError);
    } else {
        thetaOKCounter++;
        if (thetaOKCounter >= REQUIRED_CONSECUTIVE_OK) {
            moveForward();
        } else {
            stopMotors();
        }
    }

    delay(10);
}
