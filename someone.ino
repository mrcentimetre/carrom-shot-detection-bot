#include <Servo.h>

int stepPin = 2;       // Connect to the step pin of the driver
int directionPin = 3;

// Conversion ratios based on provided measurements
const float stepsPerCm = 18000.0 / 4.6; // Steps per centimeter

// Declare steps15cm globally so it can be accessed in both setup and loop
long steps15cm;

// Create servo objects
Servo baseArmServo;     // MG995 for the base arm
Servo secondArmServo;   // MG995 for the second arm
Servo thirdArmServo;    // MG90S for the third arm
Servo myServo;          // Additional servo for solenoid rotation

// Define the pin numbers
const int baseArmPin = 9;
const int secondArmPin = 8;
const int thirdArmPin = 6;
const int servoPin = 4;
const int relayPin = 7; // Pin connected to relay module

// Current angle of the additional servo
int currentAngle = 90;  // Initial angle (starting position)

// Variables to store received data
float distance;
int targetAngle;

void setup() {
  pinMode(stepPin, OUTPUT);
  pinMode(directionPin, OUTPUT);
  pinMode(relayPin, OUTPUT);

  // Attach the servos to their respective pins
  baseArmServo.attach(baseArmPin);
  secondArmServo.attach(secondArmPin);
  thirdArmServo.attach(thirdArmPin);
  myServo.attach(servoPin);

  // Set initial positions for the servos
  baseArmServo.write(90);      // Base arm starting position
  secondArmServo.write(53);    // Second arm starting position
  thirdArmServo.write(8);      // Third arm starting position
  myServo.write(90);           // Solenoid rotation servo starting position

  Serial.begin(9600); // Initialize serial communication
  Serial.println("Starting...");

  // Calculate steps for 15 cm
  steps15cm = distanceToSteps(15);

  // Move 15 cm in the HIGH direction
  digitalWrite(directionPin, HIGH);
  moveStepper(steps15cm);
}

void loop() {
  // Check if there's any serial input
  if (Serial.available() > 0) {
    // Read the first value for distance
    distance = Serial.parseFloat();
    // Read the second value for target angle
    targetAngle = Serial.parseInt();

    long stepsInput = distanceToSteps(distance);

    if (stepsInput != -1) {
      // Move the input distance in the HIGH direction
      digitalWrite(directionPin, HIGH);
      moveStepper(stepsInput);

      // Claw movements
      thirdArmServo.write(8);

      // Move the base arm continuously
      for (int pos = 90; pos >= 40; pos -= 1) { // Change the range and increment for desired movement
        baseArmServo.write(pos);
        delay(50); // Adjust delay for desired speed
      }
      // Move the second arm servo to 50 degrees
      secondArmServo.write(53);
      delay(1000); // Wait for 1 second for the servo to reach the position

      // Move the third arm servo
      delay(2000);
      thirdArmServo.write(65);
      delay(1000);
      thirdArmServo.write(8);
      delay(1000);

      // Move the base arm back to its original position
      for (int pos = 40; pos <= 90; pos += 1) { // Change the range and increment for desired movement
        baseArmServo.write(pos);
        delay(50); // Adjust delay for desired speed
      }
      delay(2000);

      // Move 6 cm in the LOW direction
      long steps6cm = distanceToSteps(6);
      digitalWrite(directionPin, LOW);
      moveStepper(steps6cm);

      // Handle solenoid rotation
      handleSolenoidRotation();

      // Move back to the starting position
      long stepsBack = steps15cm + stepsInput - steps6cm;
      moveStepper(stepsBack);

      Serial.println("Movement completed.");
    } else {
      Serial.println("Invalid distance. Please enter a valid cm value:");
    }

    // Prevent further execution of the loop
    while (1) {
      // Do nothing
    }
  }
}

// Function to convert distance to steps
long distanceToSteps(float distance) {
  if (distance > 0) {
    return distance * stepsPerCm;
  } else {
    return -1; // Invalid distance
  }
}

// Function to move the stepper motor
void moveStepper(long steps) {
  for (long stepCount = 0; stepCount < steps; stepCount++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(166); // Adjust this value if needed
    digitalWrite(stepPin, LOW);
    delayMicroseconds(166); // Adjust this value if needed
  }
}

// Function to rotate the servo gradually
void rotateServoGradually(int targetAngle) {
  Serial.print("Current angle: ");
  Serial.println(currentAngle);
  Serial.print("Target angle: ");
  Serial.println(targetAngle);

  if (targetAngle > currentAngle) {
    for (int angle = currentAngle; angle <= targetAngle; angle++) {
      myServo.write(angle);
      delay(20);  // Adjust the delay to control the speed
    }
  } else {
    for (int angle = currentAngle; angle >= targetAngle; angle--) {
      myServo.write(angle);
      delay(20);  // Adjust the delay to control the speed
    }
  }
  currentAngle = targetAngle;  // Update the current angle
}

// Function to handle solenoid rotation
void handleSolenoidRotation() {
  // Use the received target angle
  Serial.print("Using received target angle: ");
  Serial.println(targetAngle);

  // Rotate to target angle
  rotateServoGradually(targetAngle);
  Serial.print("Servo rotated to ");
  Serial.print(targetAngle);
  Serial.println(" degrees.");
  delay(2000);

  // Solenoid code

  // Turn the solenoid on (activate the relay)
  digitalWrite(relayPin, HIGH);
  delay(1000); // Keep the solenoid on for 1 second

  // Turn the solenoid off (deactivate the relay)
  digitalWrite(relayPin, LOW);

  // Rotate the servo back to 90 degrees gradually
  rotateServoGradually(90);
  Serial.println("Servo returned to 90 degrees.");
}
