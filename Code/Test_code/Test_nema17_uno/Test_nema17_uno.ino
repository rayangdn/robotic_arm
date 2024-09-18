#include <ESP32Servo.h>  // Include the ESP32Servo library

Servo myServo;  // Create a Servo object to control the servo

// defines pins
#define stepPin 2
#define dirPin 5 
 
void setup() {
  Serial.begin(115200);  // Start serial communication at 115200 baud
  myServo.attach(13);  // Attach the servo to a PWM-capable pin (e.g., GPIO 13 on ESP32)
  myServo.write(0);
  // Sets the two pins as Outputs
  pinMode(stepPin,OUTPUT); 
  pinMode(dirPin,OUTPUT);
} 
void loop() {
  digitalWrite(dirPin,HIGH); // Enables the motor to move in a particular direction
  // Makes 200 pulses for making one full cycle rotation
  for(int x = 0; x < 200; x++) {
    digitalWrite(stepPin,HIGH); 
    delayMicroseconds(10000);    // by changing this time delay between the steps we can change the rotation speed
    digitalWrite(stepPin,LOW); 
    delayMicroseconds(10000); 
  }
  delay(1000); // One second delay
  
  digitalWrite(dirPin,LOW); //Changes the rotations direction
  // Makes 400 pulses for making two full cycle rotation
  for(int x = 0; x < 400; x++) {
    digitalWrite(stepPin,HIGH);
    delayMicroseconds(3000);
    digitalWrite(stepPin,LOW);
    delayMicroseconds(3000);
  }
  delay(1000);
}




