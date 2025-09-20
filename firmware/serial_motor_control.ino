/*
  Serial Motor Control with Sensor Monitoring

  This sketch allows you to control two motors (A and B) by sending
  single-character commands over the serial monitor, while also continuously
  printing sensor data for monitoring purposes.

  Motor Commands:
  '1': Motor A, Forward
  '2': Motor A, Backward
  '3': Motor B, Forward
  '4': Motor B, Backward
  '0': Stop All Motors
*/

// ====== PIN DEFINITIONS (from hardware_test_v4.ino) ======

// Pressure Sensors (FSRs)
const int PRESSURE_SENSOR_PINS[6] = {39, 36, 34, 35, 32, 33};
const int NUM_PRESSURE_SENSORS = 6;

// Distance Sensors (HC-SR04)
const int DISTANCE_TRIG_PINS[2] = {17,26};
const int DISTANCE_ECHO_PINS[2] = {16,25};
const int NUM_DISTANCE_SENSORS = 2;

// Motor Driver (L298N)
const int MOTOR_A_IN1 = 13; // Corresponds to L298N IN1
const int MOTOR_A_IN2 = 12; // Corresponds to L298N IN2
const int MOTOR_B_IN1 = 23;  // Corresponds to L298N IN3
const int MOTOR_B_IN2 = 5; // Corresponds to L298N IN4

// ====== CONSTANTS ======
const unsigned long SENSOR_READ_INTERVAL_MS = 2000; // Read sensors every 2 seconds
const unsigned long MAX_DISTANCE_TIMEOUT_US = 30000; // Timeout for pulseIn

// ====== GLOBAL VARIABLES FOR TIMING ======
unsigned long lastSensorReadTime = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Pin initializations
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

  // Print instructions to the serial monitor
  Serial.println("Serial Motor Control w/ Sensor Monitoring Initialized.");
  Serial.println("-----------------------------------------------------");
  Serial.println("Send a command to control the motors:");
  Serial.println(" '1': Motor A Forward | '2': Motor A Backward");
  Serial.println(" '3': Motor B Forward | '4': Motor B Backward");
  Serial.println(" '0': Stop All Motors");
  Serial.println("-----------------------------------------------------");

  // Ensure motors are stopped initially
  stopMotors();
}

void loop() {
  unsigned long currentTime = millis();

  // --- Part 1: Handle Serial Motor Commands ---
  if (Serial.available() > 0) {
    handleMotorCommands();
  }

  // --- Part 2: Periodically Read and Print Sensor Data ---
  if (currentTime - lastSensorReadTime >= SENSOR_READ_INTERVAL_MS) {
    lastSensorReadTime = currentTime;
    readAndPrintSensors();
  }
}

void handleMotorCommands() {
  char command = Serial.read();

  // Ignore newline or carriage return characters to prevent "Unknown command" errors.
  if (command == '\n' || command == '\r') {
    return; // Exit the function immediately.
  }

  stopMotors();
  delay(50); // Small delay

  switch (command) {
    case '1':
      Serial.println("CMD: Motor A Forward");
      digitalWrite(MOTOR_A_IN1, HIGH);
      digitalWrite(MOTOR_A_IN2, LOW);
      break;
    case '2':
      Serial.println("CMD: Motor A Backward");
      digitalWrite(MOTOR_A_IN1, LOW);
      digitalWrite(MOTOR_A_IN2, HIGH);
      break;
    case '3':
      Serial.println("CMD: Motor B Forward");
      digitalWrite(MOTOR_B_IN1, HIGH);
      digitalWrite(MOTOR_B_IN2, LOW);
      break;
    case '4':
      Serial.println("CMD: Motor B Backward");
      digitalWrite(MOTOR_B_IN1, LOW);
      digitalWrite(MOTOR_B_IN2, HIGH);
      break;
    case '0':
      Serial.println("CMD: Stop All Motors");
      break; // stopMotors() was already called
    default:
      Serial.print("Unknown command: '");
      Serial.print(command);
      Serial.println("'. Stopping motors.");
      break;
  }
}

void readAndPrintSensors() {
  int pressureValues[NUM_PRESSURE_SENSORS];
  long distanceValues[NUM_DISTANCE_SENSORS];

  // Read all sensors
  for (int i = 0; i < NUM_PRESSURE_SENSORS; ++i) {
    pressureValues[i] = analogRead(PRESSURE_SENSOR_PINS[i]);
  }
  for (int i = 0; i < NUM_DISTANCE_SENSORS; ++i) {
    distanceValues[i] = readDistance(i);
    delay(10);
  }

  // Print all sensor values
  Serial.print("DATA -> Pressure: [");
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
  if (duration_us == 0) return -1;
  return duration_us * 343 / 2000;
}

void stopMotors() {
  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_B_IN1, LOW);
  digitalWrite(MOTOR_B_IN2, LOW);
}
