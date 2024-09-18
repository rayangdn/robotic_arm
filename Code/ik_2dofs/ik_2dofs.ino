#include <ESP32Servo.h>
#include <math.h>

Servo shoulderServo;
Servo elbowServo;
Servo wristRollServo;
Servo wristPitchServo;
Servo gripperServo;

// Define pins for servos
#define shoulderPin 25
#define elbowPin 26
#define wristRollPin 27
#define wristPitchPin 14
#define gripperPin 12

// Define initial positions
#define shoulderOffset 0
#define elbowOffset 90 
#define wristRollOffset 0
#define wristPitchOffset 90
#define gripperOffset 0

void setup() {
  Serial.begin(9600);

  // Setup servos
  shoulderServo.attach(shoulderPin);
  elbowServo.attach(elbowPin);
  wristRollServo.attach(wristRollPin);
  wristPitchServo.attach(wristPitchPin);
  gripperServo.attach(gripperPin);

  // Initialize servos at their offsets
  shoulderServo.write(shoulderOffset);
  elbowServo.write(elbowOffset);
  wristRollServo.write(wristRollOffset);
  wristPitchServo.write(wristPitchOffset);
  gripperServo.write(gripperOffset);

  delay(1000);
}

void loop() {
  if (Serial.available() > 0) {
    // Read the incoming data as a string
    String input = Serial.readStringUntil('\n');

    // Find the positions of the commas in the input string
    int firstCommaIndex = input.indexOf(',');
    int secondCommaIndex = input.indexOf(',', firstCommaIndex + 1);
    int thirdCommaIndex = input.indexOf(',', secondCommaIndex + 1);
    int forthCommaIndex = input.indexOf(',', thirdCommaIndex + 1);

    // Ensure that all commas are present
    if (firstCommaIndex > 0 && secondCommaIndex > 0 && thirdCommaIndex > 0) {
      // Extract and convert the angles
      float q1 = input.substring(0, firstCommaIndex).toFloat();
      float q2 = input.substring(firstCommaIndex + 1, secondCommaIndex).toFloat();
      float q3 = input.substring(secondCommaIndex + 1, thirdCommaIndex).toFloat();
      float q4 = input.substring(thirdCommaIndex + 1, forthCommaIndex).toFloat();
      float q5 = input.substring(forthCommaIndex + 1).toFloat();

      // Move the servos to the calculated angles
      shoulderServo.write(q1);
      elbowServo.write(elbowOffset - q2);  // Adjust elbow for reverse motion
      wristRollServo.write(q3);
      wristPitchServo.write(wristPitchOffset + q4);
      gripperServo.write(q5);
    }
  }
}
