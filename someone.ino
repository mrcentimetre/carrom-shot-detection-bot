#include <Servo.h>

int stepPin = 2;      // Connect to the step pin of the driver
int directionPin = 3;

// Conversion ratios based on provided measurements
const float stepsPerCm = 18000.0 / 4.6; // Steps per centimeter

// Declare steps15cm globally so it can be accessed in both setup and loop
long steps15cm;

// Create servo objects
Servo baseArmServo;     // MG995 for the base arm
Servo secondArmServo;   // MG995 for the second arm
Servo thirdArmServo;    // MG995 for the third arm

int servoPin = 9;

void setup() {
  // Initialize serial communication at 9600 baud rate
  Serial.begin(9600);

  pinMode(stepPin, OUTPUT);
  pinMode(directionPin, OUTPUT);

  baseArmServo.attach(10);      // Attach the base arm servo to pin 10
  secondArmServo.attach(11);    // Attach the second arm servo to pin 11
  thirdArmServo.attach(12);     // Attach the third arm servo to pin 12

  // Move the servo to the initial position
  baseArmServo.write(90);       // Set the initial position for the base arm servo
  secondArmServo.write(90);     // Set the initial position for the second arm servo
  thirdArmServo.write(90);      // Set the initial position for the third arm servo
}

void loop() {
  // Check if data is available to read from serial
  if (Serial.available() > 0) {
    // Read the incoming data from the serial port
    String data = Serial.readStringUntil('\n');

    // Parse the length and angle from the received data
    int stepperMotorLength = data.toInt();
    float servoAngle = data.toFloat();

    // Calculate the number of steps required for the stepper motor
    long steps = stepperMotorLength * stepsPerCm;

    // Print the received values for debugging
    Serial.print("Received Length (cm): ");
    Serial.println(stepperMotorLength);
    Serial.print("Received Servo Angle: ");
    Serial.println(servoAngle);

    // Control the stepper motor
    if (stepperMotorLength > 0) {
      digitalWrite(directionPin, HIGH);
    } else {
      digitalWrite(directionPin, LOW);
    }

    for (long i = 0; i < abs(steps); i++) {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(500);
    }

    // Move the servo to the desired angle
    baseArmServo.write(servoAngle);

    // Add any additional delays or actions as needed

    // Respond back to Python to indicate the action is complete
    Serial.println("Action Complete");
  }
}
