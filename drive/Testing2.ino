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

const float OBSTACLE_THRESHOLD = 0.25;     // obstacle detection front distance (m)
const float MIN_VALID_READING = 0.05;      // ignore readings < 5 cm

// Side-follow thresholds (tune these)
const float PARALLEL_THRESHOLD = 0.30;     // while alongside obstacle, side distance typically < this (m)
const float PARALLEL_CLEAR     = 0.45;     // when cleared, side distance typically > this (m)
const int SIDE_CLEAR_CONFIRM_COUNT = 3;    // require this many consecutive "clear" readings to confirm

// If side sensor is invalid repeatedly, treat as clear after this many invalids
const int SIDE_INVALID_MAX = 3;

// ---------------- STATE VARIABLES ----------------
int obstacleFrontCounter = 0;
float desiredTheta = 0.0;
unsigned long forwardStartTime = 0;

// Side-follow state
int sideClearCounter = 0;
int sideInvalidCounter = 0;
bool useLeftSideSensor = true; // which side sensor to use while following the obstacle

// ---------------- FSM ----------------
enum ObstacleState {
    NORMAL_DRIVING,
    TURN_TO_PARALLEL,
    FOLLOW_PARALLEL,         // NEW: use side sensor to detect end of obstacle
    TURN_BACK_TO_ZERO
};
ObstacleState obsState = NORMAL_DRIVING;

// ---------------- MOTOR CONTROL ----------------
void setMotorPWM(int leftPWM, int rightPWM, bool leftForward = true, bool rightForward = true) {
    leftPWM  = constrain(leftPWM, 0, 255);
    rightPWM = constrain(rightPWM, 0, 255);

    // Left motors direction
    digitalWrite(IN1, leftForward ? HIGH : LOW);
    digitalWrite(IN2, leftForward ? LOW : HIGH);
    digitalWrite(IN5, leftForward ? HIGH : LOW);
    digitalWrite(IN6, leftForward ? LOW : HIGH);
    analogWrite(ENA1, leftPWM);
    analogWrite(ENA2, leftPWM);

    // Right motors direction (note reversed logic to match wiring used previously)
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

void moveForward() {
    setMotorPWM(BASE_SPEED, BASE_SPEED, true, true);
}

void pivotInPlace(float error) {
    // error > 0 -> desired is left of current -> pivot left
    if (error > 0)
        setMotorPWM(TURN_SPEED, TURN_SPEED, false, true);  // left motor backwards, right forwards
    else
        setMotorPWM(TURN_SPEED, TURN_SPEED, true, false);  // left forwards, right backwards
}

// ---------------- ANGLE FUNCTIONS ----------------
float normalizeAngle(float a) {
    while (a >  M_PI) a -= 2 * M_PI;
    while (a < -M_PI) a += 2 * M_PI;
    return a;
}

float getThetaErrorTo(float target) {
    return normalizeAngle(target - Enes100.getTheta());
}

bool isThetaAcceptable(float err, float tolerance = THETA_TOLERANCE) {
    return fabs(err) <= tolerance;
}

// ---------------- ULTRASONIC FUNCTIONS ----------------
float readUltrasonicDistance(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW); delayMicroseconds(2);
    digitalWrite(trigPin, HIGH); delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH, 30000); // 30 ms timeout
    if (duration == 0) return -1.0;                // timeout / invalid

    // speed of sound ~ 343 m/s. duration in microseconds, divide by 2 for round-trip
    return (duration * 0.000343 / 2.0); // meters
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

float getLeftSideDistance() {
    float d = readUltrasonicDistance(TRIG_LEFT_SIDE, ECHO_LEFT_SIDE);
    if (d > 0 && d < MIN_VALID_READING) return -1;
    return d;
}

float getRightSideDistance() {
    float d = readUltrasonicDistance(TRIG_RIGHT_SIDE, ECHO_RIGHT_SIDE);
    if (d > 0 && d < MIN_VALID_READING) return -1;
    return d;
}

// ---------------- SETUP ----------------
void setup() {
    int pins[] = {IN1,IN2,IN3,IN4,IN5,IN6,IN7,IN8,ENA1,ENA2,ENB1,ENB2};
    for (int i = 0; i < 12; i++) pinMode(pins[i], OUTPUT);

    pinMode(TRIG_FRONT_LEFT, OUTPUT);  pinMode(ECHO_FRONT_LEFT, INPUT);
    pinMode(TRIG_FRONT_RIGHT, OUTPUT); pinMode(ECHO_FRONT_RIGHT, INPUT);
    pinMode(TRIG_LEFT_SIDE, OUTPUT);   pinMode(ECHO_LEFT_SIDE, INPUT);
    pinMode(TRIG_RIGHT_SIDE, OUTPUT);  pinMode(ECHO_RIGHT_SIDE, INPUT);

    Serial.begin(115200);

    Enes100.begin("Material_Mermaids", MATERIAL, 500, 1116, 50, 51);
    while (!Enes100.isVisible()) {
        delay(50);
    }

    stopMotors();
}

// ---------------- LOOP ----------------
void loop() {
    // If vision lost, stop for safety and keep loop running (don't return)
    if (!Enes100.isVisible()) {
        stopMotors();
        delay(10);
        return;
    }

    float distFront = getFrontDistance();
    float x = Enes100.getX();

    switch(obsState) {

        // ---------------- NORMAL DRIVING ----------------
        case NORMAL_DRIVING: {
            float err = getThetaErrorTo(TARGET_THETA);

            // FRONT OBSTACLE DETECTION: require several consistent readings
            if (distFront > 0 && distFront <= OBSTACLE_THRESHOLD)
                obstacleFrontCounter++;
            else
                obstacleFrontCounter = 0;

            // Confirmed obstacle in front
            if (obstacleFrontCounter >= 3) {
                stopMotors();

                // Decide which way to turn and which side sensor to use
                // If x <= 1.0 -> TURN RIGHT (-π/2) and use LEFT side sensor
                // If x >  1.0 -> TURN LEFT  (+π/2) and use RIGHT side sensor
                if (x <= 1.0f) {
                    desiredTheta = -M_PI/2;     // turn right
                    useLeftSideSensor = true;   // obstacle will be on left side while we follow
                } else {
                    desiredTheta = M_PI/2;      // turn left
                    useLeftSideSensor = false;  // obstacle will be on right side while we follow
                }

                desiredTheta = normalizeAngle(desiredTheta);

                // reset side-follow counters
                sideClearCounter = 0;
                sideInvalidCounter = 0;

                obsState = TURN_TO_PARALLEL;
                break;
            }

            // Normal heading correction (keep theta ~ 0)
            if (!isThetaAcceptable(err)) {
                stopMotors();
                pivotInPlace(err);
            } else {
                moveForward();
            }
        } break;

        // ---------------- TURN TO ±π/2 ----------------
        case TURN_TO_PARALLEL: {
            float err = getThetaErrorTo(desiredTheta);

            // Rotate in place until acceptable
            if (!isThetaAcceptable(err, 0.15)) {
                pivotInPlace(err);
            } else {
                stopMotors();
                // start following parallel with side sensors
                sideClearCounter = 0;
                sideInvalidCounter = 0;
                obsState = FOLLOW_PARALLEL;
            }
        } break;

        // ---------------- FOLLOW PARALLEL USING SIDE SENSOR ----------------
        case FOLLOW_PARALLEL: {
            // Keep heading parallel to the obstacle while moving forward
            float err = getThetaErrorTo(desiredTheta);
            if (!isThetaAcceptable(err, 0.15)) {
                stopMotors();
                pivotInPlace(err);
                // do not advance side counters while turning
            } else {
                moveForward();

                // read the appropriate side sensor
                float sideDist = useLeftSideSensor ? getLeftSideDistance() : getRightSideDistance();

                // sideDist < 0 indicates invalid/timeout reading
                if (sideDist < 0) {
                    // count invalids; if sensor consistently invalid, treat as cleared (fallback)
                    sideInvalidCounter++;
                    sideClearCounter = 0; // reset clear counter since we don't know
                    if (sideInvalidCounter >= SIDE_INVALID_MAX) {
                        // assume cleared if side sensor unavailable for a while
                        stopMotors();
                        obsState = TURN_BACK_TO_ZERO;
                    }
                } else {
                    // valid side reading -> reset invalid counter
                    sideInvalidCounter = 0;

                    // if we are still beside the obstacle (close), reset clear count
                    if (sideDist <= PARALLEL_THRESHOLD) {
                        sideClearCounter = 0; // still next to obstacle
                    }
                    // if we see a large side distance, increment clear counter
                    else if (sideDist >= PARALLEL_CLEAR) {
                        sideClearCounter++;
                        // require several consecutive clears to avoid noise
                        if (sideClearCounter >= SIDE_CLEAR_CONFIRM_COUNT) {
                            stopMotors();
                            obsState = TURN_BACK_TO_ZERO;
                        }
                    } else {
                        // intermediate distances (between PARALLEL_THRESHOLD and PARALLEL_CLEAR)
                        // treat as neither clearly alongside nor clearly cleared -> reset clear counter
                        sideClearCounter = 0;
                    }
                }
            }
        } break;

        // ---------------- TURN BACK TO θ = 0 ----------------
        case TURN_BACK_TO_ZERO: {
            float err = getThetaErrorTo(0.0);

            if (!isThetaAcceptable(err, 0.12)) {
                pivotInPlace(err);
            } else {
                stopMotors();
                obstacleFrontCounter = 0; // reset front detection
                sideClearCounter = 0;
                sideInvalidCounter = 0;
                obsState = NORMAL_DRIVING;
            }
        } break;
    }

    delay(10);
}
