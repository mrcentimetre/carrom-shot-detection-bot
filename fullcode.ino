#include <Servo.h>

int stepPin = 2;       // Connect to the step pin of the driver
int directionPin = 3;  // Connect to the direction pin of the driver

// Fixed positions
const long initialPosition = 0;
const long firstPosition = 59000;
const long secondPosition = 25000;
const long thirdPosition = 25000;
const long returnPosition = 59000;

// Variable to keep track of the current position
long currentPosition = initialPosition;
long inputSteps = 0;

// Create servo objects
Servo baseArmServo;     // MG995 for the base arm
Servo secondArmServo;   // MG995 for the second arm
Servo thirdArmServo;    // MG90S for the third arm
Servo myServo;          // New servo object for additional movement

// Define the pin numbers
const int baseArmPin = 9;
const int secondArmPin = 8;
const int thirdArmPin = 6;
const int servoPin = 4; // New servo pin

int pos = 90;
int currentAngle = 90;  // Initial angle (starting position for the new servo)

void setup() {
  pinMode(stepPin, OUTPUT);
  pinMode(directionPin, OUTPUT);
  Serial.begin(9600);  // Initialize serial communication

  thirdArmServo.write(8);
  delay(1000);

  Serial.println("Starting at initial position. Moving to First Position.");
  moveToPosition(firstPosition, HIGH); // Move to First Position
  delay(1000);
  Serial.println("Enter a number from 0 to 10 to rotate the stepper motor:");

  // Attach the servos to their respective pins
  baseArmServo.attach(baseArmPin);
  secondArmServo.attach(secondArmPin);
  thirdArmServo.attach(thirdArmPin);
  myServo.attach(servoPin); // Attach the new servo to its pin
  myServo.write(currentAngle);  // Set initial position to 90 degrees

  while (true) {
    if (Serial.available() > 0) {
      int input = Serial.parseInt();  // Read the input number from the serial monitor
      
      switch (input) {
        case 0: inputSteps = 0; break;
        case 1: inputSteps = 12000; break;
        case 2: inputSteps = 25000; break;
        case 3: inputSteps = 33000; break;
        case 4: inputSteps = 43000; break;
        case 5: inputSteps = 50000; break;
        case 6: inputSteps = 59000; break;
        case 7: inputSteps = 70000; break;
        case 8: inputSteps = 80000; break;
        case 9: inputSteps = 92000; break;
        case 10: inputSteps = 100000; break;
        default: 
          Serial.println("Invalid input. Enter a number from 0 to 10:");
          inputSteps = -1; // Invalid input indicator
          break;
      }

      if (inputSteps >= 0) {
        Serial.println("Rotating input steps in the same direction.");
        moveToPosition(inputSteps, HIGH); // Move input steps in the same direction
        delay(1000);

        // Call the function to control the arm movements
        controlArmMovements();

        Serial.println("Moving to Second Position.");
        moveToPosition(secondPosition, LOW); // Move to Second Position

        //Solenoid rotation
        Serial.println("Rotating the solenoid.");
        delay(1000);

        // New servo control part
        if (Serial.available() > 0) {
          // Read the input angle from the serial monitor
          int targetAngle = Serial.parseInt();

          // Check if the input angle is within the valid range
          if (targetAngle >= 0 && targetAngle <= 180) {
            // Rotate the servo to the specified angle gradually
            rotateServoGradually(targetAngle);
            Serial.print("Servo rotated to ");
            Serial.print(targetAngle);
            Serial.println(" degrees.");

            // Delay before returning to 90 degrees
            delay(1000);

            // Rotate the servo back to 90 degrees gradually
            rotateServoGradually(90);
            Serial.println("Servo returned to 90 degrees.");
          } else {
            Serial.println("Invalid input. Enter an angle between 0 and 180:");
          }

          // Clear the serial buffer
          while (Serial.available() > 0) {
            Serial.read();
          }
        }

        //Shot
        Serial.println("Shoot.");
        delay(1000);

        Serial.println("Moving to Third Position.");
        moveToPosition(thirdPosition, HIGH); // Move to Third Position

        Serial.println("Rotating input steps in the opposite direction.");
        moveToPosition(inputSteps, LOW); // Move input steps in the opposite direction

        Serial.println("Returning to Initial Position.");
        moveToPosition(returnPosition, LOW); // Return to Initial Position

        Serial.println("All movements complete.");
        break; // Exit the while loop after one complete operation
      } else {
        Serial.println("Invalid input. Enter a number from 0 to 10:");
      }
    }
  }
}

void loop() {
  // Leave empty to ensure the code runs only once
}

// Function to move the stepper motor to a specific position
void moveToPosition(long steps, bool direction) {
  long stepCount = abs(steps);
  digitalWrite(directionPin, direction);

  for (long i = 0; i < stepCount; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(166); // 1000000 microseconds / 6000 steps per second = 166 microseconds
    digitalWrite(stepPin, LOW);
    delayMicroseconds(166); // 1000000 microseconds / 6000 steps per second = 166 microseconds
  }

  // Update current position
  currentPosition += (direction == HIGH) ? steps : -steps;
}

// Function to control the arm movements
void controlArmMovements() {
  // Move the base arm continuously
  for (pos = 90; pos >= 40; pos -= 1) { // Change the range and increment for desired movement
    baseArmServo.write(pos);
    delay(50); // Adjust delay for desired speed
  }
  // Move the second arm servo to 50 degrees
  secondArmServo.write(50);
  delay(2000); // Wait for 1 second for the servo to reach the position

  // Move the third arm servo
  delay(5000);
  thirdArmServo.write(65);
  delay(1000);
  thirdArmServo.write(8);
  delay(1000);
 
  // Move the base arm back to its original position
  for (pos = 40; pos <= 90; pos += 1) { // Change the range and increment for desired movement
    baseArmServo.write(pos);
    delay(50); // Adjust delay for desired speed
  }
  delay(5000);
}

// Function to rotate the new servo gradually
void rotateServoGradually(int targetAngle) {
  if (targetAngle > currentAngle) {
    for (int angle = currentAngle; angle <= targetAngle; angle++) {
      myServo.write(angle);
      delay(50);  // Adjust the delay to control the speed
    }
  } else {
    for (int angle = currentAngle; angle >= targetAngle; angle--) {
      myServo.write(angle);
      delay(50);  // Adjust the delay to control the speed
    }
  }
  currentAngle = targetAngle;  // Update the current angle
}
