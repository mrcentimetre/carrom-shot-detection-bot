int stepPin = 2;      // Connect to the step pin of the driver
int directionPin = 3; // Connect to the direction pin of the driver

void setup() {
  // Set the pins as outputs
  pinMode(stepPin, OUTPUT);
  pinMode(directionPin, OUTPUT);
  
  // Set the direction to clockwise
  digitalWrite(directionPin, HIGH);

  // Move the motor 4000 steps clockwise
  for(long stepCount = 1; stepCount <= 20000; stepCount++) {  // Changed int to long
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(100);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(100);
  }
}

void loop() {
  // Do nothing here, the code in setup() runs only once
}
