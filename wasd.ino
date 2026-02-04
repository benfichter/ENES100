#include "Arduino.h"
#include "Tank.h"
#include "Enes100.h"

void setup() {
  Enes100.begin("Material Mermaids", MATERIAL, 13,1116,5,6);
  Serial.begin(9600);
  Tank.begin(); 
}

void loop() {
  move();  
}

void move() { 
while (Enes100.getX() <= 2 && Enes100.getY() <= 4) {
     Serial.println("Enter a letter:");
     if (Serial.available() > 0) { // has user input
      String userInput = Serial.readStringUntil('\n');
      userInput.trim();
      Serial.println(userInput);
     if (userInput == "x") {
      Tank.turnOffMotors();// turn it off
     } else if (userInput == "a") {
          // turn left
      } else if (userInput == "d") {
         // turn right
      } else if (userInput == "w") {
         Tank.setLeftMotorPWM(100);    
         Tank.setRightMotorPWM(100);
      } else if (userInput == "s") {
         Tank.setLeftMotorPWM(-100);    
         Tank.setRightMotorPWM(-100);
      } else {
        Tank.turnOffMotors();
      }
}
}
}
