/*
  Hardware Test Sketch for Smart Seat Project - V2

  This sketch includes fixes for distance sensor readings and provides more 
  detailed debugging information.

  V2 Changes:
  - Reads distance sensors sequentially to prevent ultrasonic interference.
  - Adds a timeout to the pulseIn() function for more stable readings.
  - Adds more detailed Serial output for easier debugging.
*/

// ====== PIN DEFINITIONS (as per your latest request) ======

// Pressure Sensors (FSRs)
const int PRESSURE_SENSOR_PINS[6] = {39, 36, 34, 35, 27, 14};
const int NUM_PRESSURE_SENSORS = 6;

// Distance Sensors (HC-SR04)
const int DISTANCE_TRIG_PINS[2] = {25, 16};
const int DISTANCE_ECHO_PINS[2] = {26, 17};
const int NUM_DISTANCE_SENSORS = 2;

// Motor Driver (L298N)
const int MOTOR_A_IN1 = 23; // Corresponds to L298N IN1
const int MOTOR_A_IN2 = 5; // Corresponds to L298N IN2
const int MOTOR_B_IN1 = 13; // Corresponds to L298N IN3
const int MOTOR_B_IN2 = 12; // Corresponds to L298N IN4

// ====== CONSTANTS ======
const int TARGET_DISTANCE_MM = 50;
const int DISTANCE_THRESHOLD_MM = 5; // 5mm tolerance
const unsigned long MAX_DISTANCE_TIMEOUT_US = 30000; // Timeout for pulseIn (microseconds)

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Hardware Test V2 Initialized...");

  // Pin initializations...
  for (int i = 0; i < NUM_PRESSURE_SENSORS; ++i) {
    pinMode(PRESSURE_SENSOR_PINS[i], INPUT);
  }
  for (int i = 0; i < NUM_DISTANCE_SENSORS; ++i) {
    pinMode(DISTANCE_TRIG_PINS[i], OUTPUT);
    pinMode(DISTANCE_ECHO_PINS[i], INPUT);
  }
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_IN1, OUTPUT);
  pinMode(MOTOR_B_IN2, OUTPUT);
}

void loop() {
  Serial.println("\n----- New Reading Cycle -----");

  int pressureValues[NUM_PRESSURE_SENSORS];
  long distanceValues[NUM_DISTANCE_SENSORS];

  readPressureSensors(pressureValues);
  printPressureValues(pressureValues);

  // Read and print distance sensors one by one to avoid interference
  Serial.println("Reading Distance Sensors...");
  for (int i = 0; i < NUM_DISTANCE_SENSORS; ++i) {
    distanceValues[i] = readDistance(i);
    Serial.print("  Sensor "); Serial.print(i + 1);
    Serial.print(": "); Serial.print(distanceValues[i]); Serial.println(" mm");
    delay(100); // Short delay between sensor readings
  }

  controlMotors(pressureValues, distanceValues);

  delay(2000);
}

void readPressureSensors(int pressureValues[]) {
  for (int i = 0; i < NUM_PRESSURE_SENSORS; ++i) {
    pressureValues[i] = analogRead(PRESSURE_SENSOR_PINS[i]);
  }
}

void printPressureValues(const int pressureValues[]) {
  Serial.print("Pressure Values: [");
  for (int i = 0; i < NUM_PRESSURE_SENSORS; ++i) {
    Serial.print(pressureValues[i]);
    if (i < NUM_PRESSURE_SENSORS - 1) Serial.print(", ");
  }
  Serial.println("]");
}

long readDistance(int sensorIndex) {
  int trigPin = DISTANCE_TRIG_PINS[sensorIndex];
  int echoPin = DISTANCE_ECHO_PINS[sensorIndex];

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read the echo pulse with a timeout
  long duration_us = pulseIn(echoPin, HIGH, MAX_DISTANCE_TIMEOUT_US);

  // If timeout occurs, pulseIn returns 0. This is a failed reading.
  if (duration_us == 0) {
    return -1; // Return -1 to indicate a failed reading
  }

  // Convert duration to millimeters
  return duration_us * 343 / 2000; // Speed of sound = 343 m/s
}

void controlMotors(const int pressureValues[], const long distanceValues[]) {
  int sumLeft = pressureValues[0] + pressureValues[1] + pressureValues[2];
  int sumRight = pressureValues[3] + pressureValues[4] + pressureValues[5];

  Serial.print("Pressure Sums -> Left: ");
  Serial.print(sumLeft);
  Serial.print(", Right: ");
  Serial.println(sumRight);

  // Motor A (Left) Logic
  if (sumLeft > sumRight && distanceValues[0] != -1 && distanceValues[0] < TARGET_DISTANCE_MM) {
    Serial.println("Action: Activating Left Motor (Forward)");
    digitalWrite(MOTOR_A_IN1, HIGH);
    digitalWrite(MOTOR_A_IN2, LOW);
  } else if (distanceValues[0] != -1 && distanceValues[0] > TARGET_DISTANCE_MM + DISTANCE_THRESHOLD_MM) {
    Serial.println("Action: Activating Left Motor (Backward - Lowering)");
    digitalWrite(MOTOR_A_IN1, LOW);
    digitalWrite(MOTOR_A_IN2, HIGH);
  } else {
    Serial.println("Action: Stopping Left Motor");
    digitalWrite(MOTOR_A_IN1, LOW);
    digitalWrite(MOTOR_A_IN2, LOW);
  }

  // Motor B (Right) Logic
  if (sumRight > sumLeft && distanceValues[1] != -1 && distanceValues[1] < TARGET_DISTANCE_MM) {
    Serial.println("Action: Activating Right Motor (Forward)");
    digitalWrite(MOTOR_B_IN1, HIGH);
    digitalWrite(MOTOR_B_IN2, LOW);
  } else if (distanceValues[1] != -1 && distanceValues[1] > TARGET_DISTANCE_MM + DISTANCE_THRESHOLD_MM) {
    Serial.println("Action: Activating Right Motor (Backward - Lowering)");
    digitalWrite(MOTOR_B_IN1, LOW);
    digitalWrite(MOTOR_B_IN2, HIGH);
  } else {
    Serial.println("Action: Stopping Right Motor");
    digitalWrite(MOTOR_B_IN1, LOW);
    digitalWrite(MOTOR_B_IN2, LOW);
  }
}
