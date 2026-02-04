#include "Arduino.h"
#include "Enes100.h"

// Declare functions from /drive/drive.ino
void driveSetup();
void driveLoop();

void setup() {
  // Initialize ENES100 (Wi-Fi, mission setup)
  Enes100.begin("Material Mermaids", MATERIAL, 3, 1116, 8, 9);

  // Initialize Tank subsystem
  driveSetup();
}

void loop() {
  driveLoop();
}
