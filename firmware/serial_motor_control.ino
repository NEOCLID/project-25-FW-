/**
 * @brief Manually control motors via serial commands while monitoring sensors.
 *
 * Motor Commands:
 * '1': Motor A, Forward
 * '2': Motor A, Backward
 * '3': Motor B, Forward
 * '4': Motor B, Backward
 * '0': Stop All Motors
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
const unsigned long SENSOR_READ_INTERVAL_MS = 2000; // Read sensors every 2 seconds
const unsigned long MAX_DISTANCE_TIMEOUT_US = 30000; // pulseIn timeout

unsigned long lastSensorReadTime = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);

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

  Serial.println("Serial Motor Control Initialized");
  Serial.println("Commands: '1'/'2' (Motor A), '3'/'4' (Motor B), '0' (Stop)");

  stopMotors();
}

void loop() {
  if (Serial.available() > 0) {
    handleMotorCommands();
  }

  if (millis() - lastSensorReadTime >= SENSOR_READ_INTERVAL_MS) {
    lastSensorReadTime = millis();
    readAndPrintSensors();
  }
}

void handleMotorCommands() {
  char command = Serial.read();

  if (command == '\n' || command == '\r') return;

  switch (command) {
    case '1':
      digitalWrite(MOTOR_A_IN1, HIGH);
      digitalWrite(MOTOR_A_IN2, LOW);
      break;
    case '2':
      digitalWrite(MOTOR_A_IN1, LOW);
      digitalWrite(MOTOR_A_IN2, HIGH);
      break;
    case '3':
      digitalWrite(MOTOR_B_IN1, HIGH);
      digitalWrite(MOTOR_B_IN2, LOW);
      break;
    case '4':
      digitalWrite(MOTOR_B_IN1, LOW);
      digitalWrite(MOTOR_B_IN2, HIGH);
      break;
    case '0':
      stopMotors();
      break;
    default:
      stopMotors();
      break;
  }
}

void readAndPrintSensors() {
  int pressureValues[NUM_PRESSURE_SENSORS];
  long distanceValues[NUM_DISTANCE_SENSORS];

  for (int i = 0; i < NUM_PRESSURE_SENSORS; ++i) {
    pressureValues[i] = analogRead(PRESSURE_SENSOR_PINS[i]);
  }
  for (int i = 0; i < NUM_DISTANCE_SENSORS; ++i) {
    distanceValues[i] = readDistance(i);
    delay(10);
  }

  Serial.print("Pressure: [");
  for (int i = 0; i < NUM_PRESSURE_SENSORS; ++i) {
    Serial.print(pressureValues[i]);
    if (i < NUM_PRESSURE_SENSORS - 1) Serial.print(", ");
  }
  Serial.print("] | Distance (mm): [");
  for (int i = 0; i < NUM_DISTANCE_SENSORS; ++i) {
    Serial.print(distanceValues[i]);
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

void stopMotors() {
  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_B_IN1, LOW);
  digitalWrite(MOTOR_B_IN2, LOW);
}
