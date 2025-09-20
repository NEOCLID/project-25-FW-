/*
  Hardware Test Sketch for Smart Seat Project - V4

  This sketch uses the final, stable pin configuration and adds a 
  pressure tolerance to prevent motor jitter.

  V4 Changes:
  - Moved all pressure sensors to stable ADC1 pins to fix high default readings.
  - Added PRESSURE_TOLERANCE to the motor logic.
  - Clarified code comments for motor control logic.
*/

// ====== PIN DEFINITIONS (Final Stable Configuration) ======

// Pressure Sensors (FSRs) - All pins are on the stable ADC1 unit
const int PRESSURE_SENSOR_PINS[6] = {39, 36, 34, 35, 32, 33};
const int NUM_PRESSURE_SENSORS = 6;

// Distance Sensors (HC-SR04)
const int DISTANCE_TRIG_PINS[2] = {26, 17};
const int DISTANCE_ECHO_PINS[2] = {25, 16};
const int NUM_DISTANCE_SENSORS = 2;

// Motor Driver (L298N)
const int MOTOR_A_IN1 = 12; // Corresponds to L298N IN1
const int MOTOR_A_IN2 = 13; // Corresponds to L298N IN2
const int MOTOR_B_IN1 = 5;  // Corresponds to L298N IN3
const int MOTOR_B_IN2 = 23; // Corresponds to L298N IN4

// ====== CONSTANTS ======
const int TARGET_DISTANCE_MM = 100;
const int PRESSURE_TOLERANCE = 100; // Dead zone for pressure sum comparison
const unsigned long MAX_DISTANCE_TIMEOUT_US = 30000; // Timeout for pulseIn

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Hardware Test V4 Initialized...");

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

  readSensors(pressureValues, distanceValues);
  printSensorValues(pressureValues, distanceValues);
  controlMotors(pressureValues, distanceValues);

  delay(2000);
}

void readSensors(int pressureValues[], long distanceValues[]) {
  for (int i = 0; i < NUM_PRESSURE_SENSORS; ++i) {
    pressureValues[i] = analogRead(PRESSURE_SENSOR_PINS[i]);
  }
  for (int i = 0; i < NUM_DISTANCE_SENSORS; ++i) {
    distanceValues[i] = readDistance(i);
    delay(50);
  }
}

void printSensorValues(const int pVals[], const long dVals[]) {
  Serial.print("Pressure Values: [");
  for (int i = 0; i < NUM_PRESSURE_SENSORS; ++i) {
    Serial.print(pVals[i]);
    if (i < NUM_PRESSURE_SENSORS - 1) Serial.print(", ");
  }
  Serial.println("]");

  Serial.print("Distance Values (mm): [");
  for (int i = 0; i < NUM_DISTANCE_SENSORS; ++i) {
    Serial.print(dVals[i]);
    if (i < NUM_DISTANCE_SENSORS - 1) Serial.print(", ");
  }
  Serial.println("]");
}

long readDistance(int sensorIndex) {
  digitalWrite(DISTANCE_TRIG_PINS[sensorIndex], LOW);
  delayMicroseconds(2);
  digitalWrite(DISTANCE_TRIG_PINS[sensorIndex], HIGH);
  delayMicroseconds(10);
  digitalWrite(DISTANCE_TRIG_PINS[sensorIndex], LOW);
  long duration_us = pulseIn(DISTANCE_ECHO_PINS[sensorIndex], HIGH, MAX_DISTANCE_TIMEOUT_US);
  if (duration_us == 0) return -1;
  return duration_us * 343 / 2000;
}

void controlMotors(const int pressureValues[], const long distanceValues[]) {
  int sumLeft = pressureValues[0] + pressureValues[1] + pressureValues[2];
  int sumRight = pressureValues[3] + pressureValues[4] + pressureValues[5];

  Serial.print("Pressure Sums -> Left: "); Serial.print(sumLeft);
  Serial.print(", Right: "); Serial.println(sumRight);

  // ---
  // Trigger: Left pressure is significantly higher than right.
  // Limit: Left distance has not reached the target height.
  if ((sumLeft > sumRight + PRESSURE_TOLERANCE) && (distanceValues[0] < TARGET_DISTANCE_MM && distanceValues[0] != -1)) {
    Serial.println("Action: Activating Left Motor (Forward)");
    digitalWrite(MOTOR_A_IN1, HIGH);
    digitalWrite(MOTOR_A_IN2, LOW);
  } else {
    Serial.println("Action: Stopping Left Motor");
    digitalWrite(MOTOR_A_IN1, LOW);
    digitalWrite(MOTOR_A_IN2, LOW);
  }

  // ---
  // Trigger: Right pressure is significantly higher than left.
  // Limit: Right distance has not reached the target height.
  if ((sumRight > sumLeft + PRESSURE_TOLERANCE) && (distanceValues[1] < TARGET_DISTANCE_MM && distanceValues[1] != -1)) {
    Serial.println("Action: Activating Right Motor (Forward)");
    digitalWrite(MOTOR_B_IN1, HIGH);
    digitalWrite(MOTOR_B_IN2, LOW);
  } else {
    Serial.println("Action: Stopping Right Motor");
    digitalWrite(MOTOR_B_IN1, LOW);
    digitalWrite(MOTOR_B_IN2, LOW);
  }
}
