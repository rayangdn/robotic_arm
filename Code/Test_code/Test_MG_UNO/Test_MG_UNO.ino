#include <Servo.h>  // Include the ESP32Servo library

Servo myServo;  // Create a Servo object to control the servo
char state = 0;

void setup() {
  Serial.begin(115200);  // Start serial communication at 115200 baud
  myServo.attach(7);  // Attach the servo to a PWM-capable pin (e.g., GPIO 13 on ESP32)
}

void loop() {
  if (Serial.available() > 0) {
    state = Serial.read();
    Serial.println(state);
  }

  switch (state) {
    case 'w':
      myServo.write(0);
      Serial.println("pos: 0");
      delay(100);
      break;

    case 's':
      myServo.write(45);
      Serial.println("pos: 45");
      delay(100);
      break;

    case 'q':
      myServo.write(90);
      Serial.println("pos: 90");
      delay(100);
      break;

    case 'e':
      myServo.write(135);
      Serial.println("pos: 135");
      delay(100);
      break;

    case 'x':
      myServo.write(180);
      Serial.println("pos: 180");
      delay(100);
      break;

    default:
      break;
  }
}
