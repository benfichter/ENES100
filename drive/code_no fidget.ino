#include <Arduino.h>
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

boolean turn_pi2 = false;
float target_theta = 0;
int turnPWM = 50;
int basePWM = 80;
void setup() {
    // Motor pins
    int motorPins[] = {IN1, IN2, IN3, IN4, IN5, IN6, IN7, IN8, ENA1, ENB1, ENA2, ENB2};
    for (int i = 0; i < 12; i++) pinMode(motorPins[i], OUTPUT);

    Serial.begin(9600);
    Enes100.begin("Material_Mermaids", MATERIAL, 500, 1116, 50, 51);

    // Wait until ENES100 sees robot
    while (!Enes100.isVisible()) {
        Serial.println("Waiting for visibility...");
        delay(50);
    }
    Serial.println("READY.");
    float startY = Enes100.getY();
    if (startY >= 1) {
      turn_pi2 = true;
      target_theta = -1.5708;
    } else {
      turn_pi2 = false;
      target_theta = 1.5708;
}
}

void loop() {
float theta = Enes100.getTheta();
float error = target_theta - theta;
if (fabs(error) > 0.15) {
  if (turn_pi2) {
    setMotorPWM(turnPWM, turnPWM, false, true);
    delay(10);
  } else {
    setMotorPWM(turnPWM, turnPWM, true, false); 
    delay(10);
}
stopMotors();
delay(50);
error = target_theta - Enes100.getTheta();  
} else {
  setMotorPWM(basePWM, basePWM, true, true);
}
}

void stopMotors() {
 setMotorPWM(0,0,true,true);
}


