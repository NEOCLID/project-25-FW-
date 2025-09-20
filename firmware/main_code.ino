#include <WiFi.h>
#include <HTTPClient.h>
#include <math.h>

// Network Config
const char* WIFI_SSID     = "m";
const char* WIFI_PASSWORD = "aaaa4444";
const char* SERVER_URL    = "http://172.20.10.2:8000/ingest"; //FastAPI endpoint
const char* BASE_DEVICE_ID = "smart-seat-01";
const uint32_t SAMPLE_INTERVAL_MS = 5000; // post data every 5s
uint32_t lastPostMs = 0;

// Pins
const int PRESSURE_SENSOR_PINS[6] = {39, 36, 34, 35, 32, 33};
const int NUM_PRESSURE_SENSORS = 6;
const int DISTANCE_TRIG_PINS[2] = {17, 26};
const int DISTANCE_ECHO_PINS[2] = {16, 25};
const int NUM_DISTANCE_SENSORS = 2;
const int MOTOR_A_IN1 = 23;
const int MOTOR_A_IN2 = 5;
const int MOTOR_B_IN1 = 12;
const int MOTOR_B_IN2 = 13;

// Control & Tuning
#define PRESSURE_THRESHOLD 1500      // sitting pressure 
#define IDLE_DISTANCE_MM 40          // target dist for idle mode
#define DISTANCE_TOLERANCE_MM 5     
#define ITERATION_PAUSE_MS 5000      

const float DISTANCE_AT_IDLE = 40.0; //seat arm length
const float TARGET_ANGLE_RAD = 15.0 * PI / 180.0; // 15 degrees

// calculated target distances
const int ITERATION_TARGET_DISTANCE_MM = (int)(15.0 * tan(TARGET_ANGLE_RAD) + DISTANCE_AT_IDLE);
const int EDGE_CASE_TARGET_DISTANCE_MM = (int)(ITERATION_TARGET_DISTANCE_MM * 0.7); // 70% of normal

const unsigned long MAX_DISTANCE_TIMEOUT_US = 30000; // for pulseIn
const int ADC_MAX = 4095;


// Global State
bool devMode = false; // dev mode flag

// state machine for control logic
enum ControlMode { MODE_IDLE, MODE_ITERATION };
ControlMode currentMode = MODE_IDLE;

enum IterationState { STATE_UP_A, STATE_PAUSE_A, STATE_DOWN_A, STATE_UP_B, STATE_PAUSE_B, STATE_DOWN_B };
IterationState iterState = STATE_UP_A;
unsigned long lastStateChangeTime = 0;


// Func Prototypes
void handleSerialInput();
void stopMotors();
void readSensors(int pressureValues[], long distanceValues[]);
void printSensorValues(const int pVals[], const long dVals[]);
long readDistance(int sensorIndex);
void controlMotors(const int pressureValues[], const long distanceValues[]);
void runIdleMode(const long distanceValues[]);
void runIterationMode(const long distanceValues[]);
void driveMotor(int motorIndex, int dir);


void waitForWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - t0) < 20000) {
    delay(250);
    Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("Connected! IP: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("WiFi connection failed.");
  }
}

bool postBatch(const int values[], int n) {
  if (WiFi.status() != WL_CONNECTED) {
    waitForWiFi();
    if (WiFi.status() != WL_CONNECTED) return false;
  }

  String json = "{"device_id":"" + String(BASE_DEVICE_ID) + "","values":[";
  for (int i = 0; i < n; ++i) {
    json += String(values[i]);
    if (i < n - 1) json += ",";
  }
  json += "]}";

  WiFiClient client;
  HTTPClient http;
  http.begin(client, SERVER_URL);
  http.addHeader("Content-Type", "application/json");

  int code = http.POST(json);
  if (code > 0) {
    // Serial.printf("POST code: %d\n", code);
  } else {
    Serial.printf("POST failed, error: %s\n", http.errorToString(code).c_str());
  }
  http.end();
  return (code >= 200 && code < 300);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("===== Smart Seat System Booting Up =====");
  Serial.println("Send '9' for dev mode, '0' to exit.");

  // init all pins
  for (int i = 0; i < NUM_PRESSURE_SENSORS; ++i) pinMode(PRESSURE_SENSOR_PINS[i], INPUT);
  for (int i = 0; i < NUM_DISTANCE_SENSORS; ++i) {
    pinMode(DISTANCE_TRIG_PINS[i], OUTPUT);
    pinMode(DISTANCE_ECHO_PINS[i], INPUT);
  }
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_IN1, OUTPUT);
  pinMode(MOTOR_B_IN2, OUTPUT);

  stopMotors();
  lastStateChangeTime = millis();
}

void loop() {
  handleSerialInput(); // always check for user input

  // if dev mode is on, skip all the auto logic
  if (devMode) {
    delay(100);
    return;
  }

  int pressureValues[NUM_PRESSURE_SENSORS];
  long distanceValues[NUM_DISTANCE_SENSORS];

  readSensors(pressureValues, distanceValues);
  printSensorValues(pressureValues, distanceValues);
  controlMotors(pressureValues, distanceValues);

  if (millis() - lastPostMs >= SAMPLE_INTERVAL_MS) {
    lastPostMs = millis();
    postBatch(pressureValues, NUM_PRESSURE_SENSORS);
  }

  delay(100); // or 50
}

void handleSerialInput() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == '\n' || cmd == '\r') return;

    if (cmd == '9') {
      if (!devMode) {
        devMode = true;
        stopMotors();
        Serial.println("DEV MODE ON. Auto control paused.");
      }
    } else if (cmd == '0') {
      if (devMode) {
        devMode = false;
        stopMotors();
        iterState = STATE_UP_A; // reset state machine
        lastStateChangeTime = millis();
        Serial.println("DEV MODE OFF. Auto control resumed.");
      }
    } else if (devMode) {
      // manual motor control
      stopMotors();
      delay(25);
      switch (cmd) {
        case '1': Serial.println("MANUAL: MTR A FWD"); driveMotor(0, 1); break;
        case '2': Serial.println("MANUAL: MTR A BWD"); driveMotor(0, -1); break;
        case '3': Serial.println("MANUAL: MTR B FWD"); driveMotor(1, 1); break;
        case '4': Serial.println("MANUAL: MTR B BWD"); driveMotor(1, -1); break;
        case 's': Serial.println("MANUAL: STOP"); stopMotors(); break;
      }
    }
  }
}

void stopMotors() {
  driveMotor(0, 0); // motor A
  driveMotor(1, 0); // motor B
}

// motorIndex: 0 for A, 1 for B
// dir: 1 for fwd, -1 for bwd, 0 for stop
void driveMotor(int motorIndex, int dir) {
  int in1 = (motorIndex == 0) ? MOTOR_A_IN1 : MOTOR_B_IN1;
  int in2 = (motorIndex == 0) ? MOTOR_A_IN2 : MOTOR_B_IN2;

  if (dir == 1) { // Fwd
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (dir == -1) { // Bwd
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else { // Stop
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

void readSensors(int pressureValues[], long distanceValues[]) {
  for (int i = 0; i < NUM_PRESSURE_SENSORS; ++i) {
    pressureValues[i] = analogRead(PRESSURE_SENSOR_PINS[i]);
    if (pressureValues[i] >= ADC_MAX) {
      Serial.printf("WARN: Pressure sensor %d is maxed out!\n", i);
    }
  }
  for (int i = 0; i < NUM_DISTANCE_SENSORS; ++i) {
    distanceValues[i] = readDistance(i);
    delay(50); // for ultrasonic
  }
}

void printSensorValues(const int pVals[], const long dVals[]) {
  Serial.print("Pressure: [");
  for (int i = 0; i < NUM_PRESSURE_SENSORS; ++i) {
    Serial.print(pVals[i]);
    if (i < NUM_PRESSURE_SENSORS - 1) Serial.print(", ");
  }
  Serial.Serial.print("] | Dist (mm): [");
  for (int i = 0; i < NUM_DISTANCE_SENSORS; ++i) {
    Serial.print(dVals[i] == -1 ? "ERR" : String(dVals[i]));
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
  long duration = pulseIn(DISTANCE_ECHO_PINS[sensorIndex], HIGH, MAX_DISTANCE_TIMEOUT_US);
  return (duration == 0) ? -1 : duration * 343 / 2000; // convert to mm
}

// main logic
void controlMotors(const int pressureValues[], const long distanceValues[]) {
  long totalPressure = 0;
  for (int i = 0; i < NUM_PRESSURE_SENSORS; i++) {
    totalPressure += pressureValues[i];
  }
  // Serial.printf("Total Pressure: %ld\n", totalPressure);

  // Is someone sitting?
  if (totalPressure < PRESSURE_THRESHOLD) {
    // Not sitting -> idle
    if (currentMode != MODE_IDLE) {
      Serial.println("User left. Switching to IDLE mode.");
      currentMode = MODE_IDLE;
      stopMotors();
    }
  } else {
    // Yup, someone is here
    if (currentMode != MODE_ITERATION) {
      Serial.println("User detected. Switching to ITERATION mode.");
      currentMode = MODE_ITERATION;
      iterState = STATE_UP_A; // start sequence from beginning
      lastStateChangeTime = millis();
      stopMotors();
    }
  }

  if (currentMode == MODE_IDLE) {
    runIdleMode(distanceValues);
  } else {
    runIterationMode(distanceValues);
  }
}

void runIdleMode(const long distanceValues[]) {
  Serial.println("State: IDLE");
  bool allIdle = true;

  if (distanceValues[0] > IDLE_DISTANCE_MM + DISTANCE_TOLERANCE_MM) {
    driveMotor(0, 1); // move down
    allIdle = false;
  } else if (distanceValues[0] < IDLE_DISTANCE_MM - DISTANCE_TOLERANCE_MM) {
    driveMotor(0, -1); // move up
    allIdle = false;
  } else {
    driveMotor(0, 0); // stop
  }

  if (distanceValues[1] > IDLE_DISTANCE_MM + DISTANCE_TOLERANCE_MM) {
    driveMotor(1, 1); // move down
    allIdle = false;
  } else if (distanceValues[1] < IDLE_DISTANCE_MM - DISTANCE_TOLERANCE_MM) {
    driveMotor(1, -1); // move up
    allIdle = false;
  } else {
    driveMotor(1, 0); // stop
  }
}

void runIterationMode(const long distanceValues[]) {
  unsigned long currentTime = millis();
  long distA = distanceValues[0];
  long distB = distanceValues[1];

  // Bail if a sensor fails
  if (distA == -1 || distB == -1) {
    Serial.println("ERROR: Distance sensor fail! Halting motors.");
    stopMotors();
    return;
  }

  // state machine for the seat movement
  switch (iterState) {
    case STATE_UP_A: {
      Serial.println("State: ITERATION - A up");
      // Edge case: B is way lower than A, don't go up as high
      float ratio = (distA > 0) ? (float)distB / distA : 0;
      int targetDist = (ratio > 4.0) ? EDGE_CASE_TARGET_DISTANCE_MM : ITERATION_TARGET_DISTANCE_MM;

      driveMotor(1, 1); // B goes down
      if (distA < targetDist) {
        driveMotor(0, -1); // A goes up
      } else {
        driveMotor(0, 0); // A at target
        Serial.println("  Target reached. Pausing.");
        iterState = STATE_PAUSE_A;
        lastStateChangeTime = currentTime;
        stopMotors();
      }
      break;
    }
    case STATE_PAUSE_A:
      // Just wait here for a bit
      if (currentTime - lastStateChangeTime >= ITERATION_PAUSE_MS) {
        Serial.println("  Pause over. Moving A down.");
        iterState = STATE_DOWN_A;
        lastStateChangeTime = currentTime;
      }
      break;

    case STATE_DOWN_A:
      Serial.println("State: ITERATION - A down");
      if (distA > IDLE_DISTANCE_MM + DISTANCE_TOLERANCE_MM) {
        driveMotor(0, 1); // A moves down to idle
      } else {
        driveMotor(0, 0);
        Serial.println("  A is idle. Moving B up.");
        iterState = STATE_UP_B;
        lastStateChangeTime = currentTime;
        stopMotors();
      }
      break;

    case STATE_UP_B: {
      Serial.println("State: ITERATION - B up");
      // Edge case again, for B
      float ratio = (distB > 0) ? (float)distA / distB : 0;
      int targetDist = (ratio > 4.0) ? EDGE_CASE_TARGET_DISTANCE_MM : ITERATION_TARGET_DISTANCE_MM;

      driveMotor(0, 1); // A goes down
      if (distB < targetDist) {
        driveMotor(1, -1); // B goes up
      } else {
        driveMotor(1, 0); // B is at target
        Serial.println("  Target reached. Pausing.");
        iterState = STATE_PAUSE_B;
        lastStateChangeTime = currentTime;
        stopMotors();
      }
      break;
    }
    case STATE_PAUSE_B:
      // wait again
      if (currentTime - lastStateChangeTime >= ITERATION_PAUSE_MS) {
        Serial.println("  Pause over. Moving B down.");
        iterState = STATE_DOWN_B;
        lastStateChangeTime = currentTime;
      }
      break;

    case STATE_DOWN_B:
      Serial.println("State: ITERATION - B down");
      if (distB > IDLE_DISTANCE_MM + DISTANCE_TOLERANCE_MM) {
        driveMotor(1, 1); // B moves down to idle
      } else {
        driveMotor(1, 0);
        Serial.println("  B is idle. Looping sequence.");
        iterState = STATE_UP_A; // loop back to the beginning
        lastStateChangeTime = currentTime;
        stopMotors();
      }
      break;
  }
}
