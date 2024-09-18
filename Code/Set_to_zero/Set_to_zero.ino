#include <ESP32Servo.h>  // Include the ESP32Servo library

Servo ShoulderServo;  // Create a Servo object to control the servo
Servo ElbowServo;

#define Shoulderpin 27
#define Elbowpin 26

void setup() {
  ShoulderServo.attach(Shoulderpin);  // Attach the servo to a PWM-capable pin (e.g., GPIO 13 on ESP32)
  ElbowServo.attach(Elbowpin);
  ShoulderServo.write(90);
  ElbowServo.write(0);
}

void loop() {
}
