/*
  Motor Idle Sketch

  This sketch initializes the motor pins and immediately stops them.
  The motors will remain idle.
*/

// Motor Driver (L298N) Pins based on hardware_test_v4.ino
const int MOTOR_A_IN1 = 12; // Corresponds to L298N IN1
const int MOTOR_A_IN2 = 13; // Corresponds to L298N IN2
const int MOTOR_B_IN1 = 5;  // Corresponds to L298N IN3
const int MOTOR_B_IN2 = 23; // Corresponds to L298N IN4

void setup() {
  // Set all motor pins to OUTPUT
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_IN1, OUTPUT);
  pinMode(MOTOR_B_IN2, OUTPUT);

  // Stop both motors
  stopMotors();
}

void loop() {
  // Do nothing in the loop, so the motors remain stopped.
}

// Function to stop both motors
void stopMotors() {
  // Stop Motor A
  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, LOW);

  // Stop Motor B
  digitalWrite(MOTOR_B_IN1, LOW);
  digitalWrite(MOTOR_B_IN2, LOW);
}
