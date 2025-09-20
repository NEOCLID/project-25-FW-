/*
  Motor Test Sketch

  This sketch tests each motor by running it in one direction, then the other,
  for a set duration. It continuously reads and prints sensor data while the
  motors are being tested.
*/

// ====== PIN DEFINITIONS (from hardware_test_v4.ino) ======

// Pressure Sensors (FSRs)
const int PRESSURE_SENSOR_PINS[6] = {39, 36, 34, 35, 32, 33};
const int NUM_PRESSURE_SENSORS = 6;

// Distance Sensors (HC-SR04)
const int DISTANCE_TRIG_PINS[2] = {25, 16};
const int DISTANCE_ECHO_PINS[2] = {26, 17};
const int NUM_DISTANCE_SENSORS = 2;

// Motor Driver (L298N)
const int MOTOR_A_IN1 = 12; // Corresponds to L298N IN1
const int MOTOR_A_IN2 = 13; // Corresponds to L298N IN2
const int MOTOR_B_IN1 = 5;  // Corresponds to L298N IN3
const int MOTOR_B_IN2 = 23; // Corresponds to L298N IN4

// ====== CONSTANTS ======
const unsigned long MOTOR_TEST_DURATION_MS = 2000; // Run each motor test for 2 seconds
const unsigned long SENSOR_READ_INTERVAL_MS = 500; // Read sensors every 500ms
const unsigned long MAX_DISTANCE_TIMEOUT_US = 30000; // Timeout for pulseIn

// ====== GLOBAL VARIABLES FOR TIMING ======
unsigned long lastSensorReadTime = 0;
unsigned long testPhaseStartTime = 0;
int currentTestPhase = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Motor Test Sketch Initialized...");

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

  // Ensure motors are stopped initially
  stopMotors();
}

void loop() {
  unsigned long currentTime = millis();

  // 1. Continuously read and print sensor data at a fixed interval
  if (currentTime - lastSensorReadTime >= SENSOR_READ_INTERVAL_MS) {
    lastSensorReadTime = currentTime;
    readAndPrintSensors();
  }

  // 2. Progress through the motor test sequence
  if (currentTime - testPhaseStartTime >= MOTOR_TEST_DURATION_MS) {
    testPhaseStartTime = currentTime; // Reset timer for the new phase
    
    stopMotors(); // Stop motors before starting the next phase
    delay(100);   // Brief pause

    currentTestPhase++;
    if (currentTestPhase > 4) { // Loop back to the start after the last test
      currentTestPhase = 1;
    }

    switch (currentTestPhase) {
      case 1:
        Serial.println("\n----- PHASE 1: Testing Motor A (Forward) -----");
        digitalWrite(MOTOR_A_IN1, HIGH);
        digitalWrite(MOTOR_A_IN2, LOW);
        break;
      case 2:
        Serial.println("\n----- PHASE 2: Testing Motor A (Backward) -----");
        digitalWrite(MOTOR_A_IN1, LOW);
        digitalWrite(MOTOR_A_IN2, HIGH);
        break;
      case 3:
        Serial.println("\n----- PHASE 3: Testing Motor B (Forward) -----");
        digitalWrite(MOTOR_B_IN1, HIGH);
        digitalWrite(MOTOR_B_IN2, LOW);
        break;
      case 4:
        Serial.println("\n----- PHASE 4: Testing Motor B (Backward) -----");
        digitalWrite(MOTOR_B_IN1, LOW);
        digitalWrite(MOTOR_B_IN2, HIGH);
        break;
    }
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
    delay(10); // Small delay between distance sensor pings
  }

  // Print all sensor values
  Serial.print("Sensor Data -> Pressure: [");
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
  if (duration_us == 0) return -1; // Timeout or error
  return duration_us * 343 / 2000;
}

void stopMotors() {
  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_B_IN1, LOW);
  digitalWrite(MOTOR_B_IN2, LOW);
}
