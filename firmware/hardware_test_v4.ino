/**
 * @file hardware_test_v4.ino
 * @brief Hardware test for the Smart Seat.
 * 
 * V4 Changes:
 * - Pressure sensors moved to stable ADC1 pins.
 * - Added PRESSURE_TOLERANCE to prevent motor jitter.
 */

// ====== PIN DEFINITIONS ======
const int PRESSURE_SENSOR_PINS[6] = {39, 36, 34, 35, 32, 33}; // FSRs on ADC1
const int NUM_PRESSURE_SENSORS = 6;

const int DISTANCE_TRIG_PINS[2] = {26, 17}; // HC-SR04 Trig
const int DISTANCE_ECHO_PINS[2] = {25, 16}; // HC-SR04 Echo
const int NUM_DISTANCE_SENSORS = 2;

const int MOTOR_A_IN1 = 12; // L298N IN1
const int MOTOR_A_IN2 = 13; // L298N IN2
const int MOTOR_B_IN1 = 5;  // L298N IN3
const int MOTOR_B_IN2 = 23; // L298N IN4

// ====== CONSTANTS ======
const int TARGET_DISTANCE_MM = 100;
const int PRESSURE_TOLERANCE = 100; // Dead zone for pressure sum comparison
const unsigned long MAX_DISTANCE_TIMEOUT_US = 30000; // pulseIn timeout

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Hardware Test V4 Initialized");

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
  Serial.print("Pressure: [");
  for (int i = 0; i < NUM_PRESSURE_SENSORS; ++i) {
    Serial.print(pVals[i]);
    if (i < NUM_PRESSURE_SENSORS - 1) Serial.print(", ");
  }
  Serial.println("]");

  Serial.print("Distance (mm): [");
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
  if (duration_us == 0) return -1; // Timeout
  return duration_us * 343 / 2000; // Convert to mm
}

void controlMotors(const int pressureValues[], const long distanceValues[]) {
  int sumLeft = pressureValues[0] + pressureValues[1] + pressureValues[2];
  int sumRight = pressureValues[3] + pressureValues[4] + pressureValues[5];

  int pressureDifference = sumLeft - sumRight;

  // Left Motor: Activate if left pressure is high and distance is below target
  if (pressureDifference > PRESSURE_TOLERANCE && (distanceValues[0] < TARGET_DISTANCE_MM && distanceValues[0] != -1)) {
    digitalWrite(MOTOR_A_IN1, HIGH);
    digitalWrite(MOTOR_A_IN2, LOW);
  } else {
    digitalWrite(MOTOR_A_IN1, LOW);
    digitalWrite(MOTOR_A_IN2, LOW);
  }

  // Right Motor: Activate if right pressure is high and distance is below target
  if (pressureDifference < -PRESSURE_TOLERANCE && (distanceValues[1] < TARGET_DISTANCE_MM && distanceValues[1] != -1)) {
    digitalWrite(MOTOR_B_IN1, HIGH);
    digitalWrite(MOTOR_B_IN2, LOW);
  } else {
    digitalWrite(MOTOR_B_IN1, LOW);
    digitalWrite(MOTOR_B_IN2, LOW);
  }
}
