#include <ESP32Servo.h>
#include <math.h>

Servo shoulderServo;
Servo elbowServo;
Servo wristServo;

// Define pins for MG/SG servos
#define shoulderPin 27
#define elbowPin 26
#define wristPin 25

// Define initial positions
#define shoulderPos 180
#define elbowPos 90 
#define wristPos 0

// Define pins for Nema motors
#define stepPin 2
#define dirPin 5

// Define robotic arm dimensions
#define lengthShoulder 120
#define lengthElbow 90
#define lengthWrist 20

// Forward Kinematics 2 DOFs
void forward_kinematics_2_dofs(float q1, float q2, float l1, float l2, float &x, float &y) 
{
  q1 = radians(q1);
  q2 = radians(q2);

  x = l1 * cos(q1) + l2 * cos(q1 + q2);
  y = l1 * sin(q1) + l2 * sin(q1 + q2);
}

// Inverse Kinematics 2 DOFs
void inverse_kinematics_2_dofs(float x, float y, float l1, float l2, float &q1, float &q2)
{
  float r = sqrt(x*x + y*y);
  float phi = atan2(y, x);

  q2 = degrees(acos((r*r - l1*l1 - l2*l2) / (2 * l1 * l2)));
  q1 = degrees(phi - atan2(l2 * sin(q2), l1 + l2 * cos(q2)));
}

// Forward Kinematics 3 DOFs
void forward_kinematics_3_dofs(float q1, float q2, float q3, float l1, float l2, float l3, float &x, float &y, float &theta)
{
  q1 = radians(q1);
  q2 = radians(q2);
  q3 = radians(q3);

  x = l1 * cos(q1) + l2 * cos(q1 + q2) + l3 * cos(q1 + q2 + q3);
  y = l1 * sin(q1) + l2 * sin(q1 + q2) + l3 * sin(q1 + q2 + q3);
  theta = degrees(q1 + q2 + q3);
}

// Inverse Kinematics 3 DOFs
void inverse_kinematics_3_dofs(float x, float y, float theta, float l1, float l2, float l3, float &q1, float &q2, float &q3) 
{
  theta = radians(theta);

  // Calculate the position for the 2-DOF subsystem
  float x_2dof = x - l3 * cos(theta);
  float y_2dof = y - l3 * sin(theta);
  
  // Get angles for the 2-DOF subsystem
  inverse_kinematics_2_dofs(x_2dof, y_2dof, l1, l2, q1, q2);
  q3 = degrees(theta - q1 - q2);
  q2 = degrees(q2);
  q1 = degrees(q1);
}

bool is_reachable_2_dofs(float x, float y, float l1, float l2) {
  float r = sqrt(x * x + y * y);
  if (r > l1 + l2 || r < abs(l1 - l2)) {
    return false;  // 2-DOF position is not reachable
  }
  return true;

}
bool is_reachable_3_dofs(float x, float y, float theta, float l1, float l2, float l3) 
{
  // Calculate distance to the target (x, y)
  float r = sqrt(x * x + y * y);
  float x_2dof = x - l3 * cos(theta);
  float y_2dof = y - l3 * sin(theta);

  // Check if the position is reachable by the 2-DOF subsystem
  float r_2dof = sqrt(x_2dof * x_2dof + y_2dof * y_2dof);
  
  // Check if the target is within reach
  if (r > (l1 + l2 + l3) || r < fabs(l1 - l2 - l3)) {
    return false;  // Out of reach
  } 
  if (r_2dof > l1 + l2 || r_2dof < abs(l1 - l2)) {
    return false;  // 2-DOF position is not reachable
  }
  return true;
}

void setup() 
{
  Serial.begin(9600);

  // Setup MG/SG servos
  shoulderServo.attach(shoulderPin);
  elbowServo.attach(elbowPin);
  wristServo.attach(wristPin);
  shoulderServo.write(shoulderPos);
  elbowServo.write(elbowPos);
  wristServo.write(wristPos);

  // Setup Nema motor
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
}

void loop() 
{
  if (Serial.available() > 0) {
    // Read the incoming data as a string
    String input = Serial.readStringUntil('\n');

    // Parse the input data (expected format: "x,y,theta")
    int firstCommaIndex = input.indexOf(',');
    int secondCommaIndex = input.indexOf(',', firstCommaIndex + 1);
    
    if (firstCommaIndex > 0 && secondCommaIndex > 0) {
      float x = input.substring(0, firstCommaIndex).toFloat();
      float y = input.substring(firstCommaIndex + 1, secondCommaIndex).toFloat();
      float theta = input.substring(secondCommaIndex + 1).toFloat();

      Serial.print("Inputs:\nx:");
      Serial.print(x);
      Serial.print(", y: ");
      Serial.print(y);
      Serial.print(", theta: ");
      Serial.println(theta);
    
      float q1, q2, q3;

      //2 DOFS arm
      if(is_reachable_2_dofs(x, y, lengthShoulder, lengthElbow))
      {
        Serial.println("Target position is reachable.");
        inverse_kinematics_2_dofs(x, y, lengthShoulder, lengthElbow, q1, q2);

        // Move the servos to the calculated angles
        shoulderServo.write(q1);
        elbowServo.write(elbowOffset-q2);

        Serial.print("Moving to position:\nq1: ");
        Serial.print(q1);
        Serial.print(", q2: ");
        Serial.println(q2);

        forward_kinematics_2_dofs(q1, q2, lengthShoulder, lengthElbow, x, y);
        
        Serial.print("Outputs:\nx:");
        Serial.print(x);
        Serial.print(", y: ");
        Serial.print(y);
      } else {
        Serial.println("Error: Target position is not reachable.");
      }
    } else {
      Serial.println("Error: Invalid input format. Expected 'x,y,theta'.");
    }
  }
}
