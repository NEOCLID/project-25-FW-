/*
  Pressure-Triggered Iteration Motor Control

  This sketch uses a state machine to control an alternating motor sequence
  based on input from pressure sensors.

  Primary logic:
  - Default State: Motors are stopped in the 'down' position.
  - Homing: When pressure is lost, active motors return to the 'down' position.
  - Iteration: When pressure is detected, a timed sequence of motor
    movements begins and repeats until pressure is lost.
*/

// Pin Definitions
const int PRESSURE_SENSOR_PINS[6] = {39, 36, 34, 35, 32, 33};
const int MOTOR_A_IN1 = 12; // Left Motor
const int MOTOR_A_IN2 = 13;
const int MOTOR_B_IN1 = 23;  // Right Motor
const int MOTOR_B_IN2 = 5;

// System Constants
const int PRESSURE_THRESHOLD = 300;       // Minimum total pressure to begin iteration.
const unsigned long MOTOR_UP_MS = 1000;     // Duration for a motor to move up.
const unsigned long MOTOR_DOWN_MS = 1100;   // Duration for a motor to move down.
const unsigned long ITERATION_WAIT_MS = 10000; // 10-second wait duration.

// State Machine
enum State { IDLE, HOMING, A_MOVING_UP, WAIT_WITH_A_UP, A_DOWN_B_UP, WAIT_WITH_B_UP, B_DOWN_A_UP };
State currentState = IDLE;
unsigned long lastStateChangeTime = 0;
bool homing_requested = false;

void setup() {
  Serial.begin(115200);
  // Initialize all I/O pins.
  for (int pin : PRESSURE_SENSOR_PINS) { pinMode(pin, INPUT); }
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_IN1, OUTPUT);
  pinMode(MOTOR_B_IN2, OUTPUT);

  stopAllMotors();
  Serial.println("System initialized. State: Default");
}

void loop() {
  // =================================================================
  //  1. SENSOR READING & PRESENCE DETECTION
  // =================================================================
  int sumPressure = 0;
  for (int pin : PRESSURE_SENSOR_PINS) {
    sumPressure += analogRead(pin);
  }
  bool userIsPresent = sumPressure > PRESSURE_THRESHOLD;

  // If user is not present, set a flag to request homing.
  // The actual homing will be handled by the state machine.
  if (!userIsPresent) {
    homing_requested = true;
  }

  // =================================================================
  //  2. STATE MACHINE LOGIC
  // =================================================================
  unsigned long currentTime = millis();

  switch (currentState) {
    // -----------------------------------------------------------------
    //  IDLE: Waiting for a user.
    // -----------------------------------------------------------------
    case IDLE:
      if (userIsPresent) {
        Serial.println("Pressure detected. Starting sequence.");
        homing_requested = false; // Reset homing flag
        currentState = A_MOVING_UP;
        motorA_up();
        lastStateChangeTime = currentTime;
      }
      break;

    // -----------------------------------------------------------------
    //  HOMING: A motor is returning to the down position.
    // -----------------------------------------------------------------
    case HOMING:
      if (currentTime - lastStateChangeTime >= MOTOR_DOWN_MS) {
        stopAllMotors();
        Serial.println("Homing complete. State: IDLE");
        currentState = IDLE;
      }
      break;

    // -----------------------------------------------------------------
    //  A_MOVING_UP: Motor A is moving up.
    // -----------------------------------------------------------------
    case A_MOVING_UP:
      if (currentTime - lastStateChangeTime >= MOTOR_UP_MS) {
        stopAllMotors();
        Serial.println("State: Wait (A up)");
        currentState = WAIT_WITH_A_UP;
        lastStateChangeTime = currentTime;
      }
      break;

    // -----------------------------------------------------------------
    //  WAIT_WITH_A_UP: Motor A is up, waiting.
    // -----------------------------------------------------------------
    case WAIT_WITH_A_UP:
      // If homing is requested, start returning motor A.
      if (homing_requested) {
        Serial.println("Homing: Motor A returning.");
        motorA_down();
        currentState = HOMING;
        lastStateChangeTime = currentTime;
      }
      // Otherwise, continue the normal iteration after the wait time.
      else if (currentTime - lastStateChangeTime >= ITERATION_WAIT_MS) {
        Serial.println("State: A down, B up");
        motorA_down();
        motorB_up();
        currentState = A_DOWN_B_UP;
        lastStateChangeTime = currentTime;
      }
      break;

    // -----------------------------------------------------------------
    //  A_DOWN_B_UP: Motor A moves down, Motor B moves up.
    // -----------------------------------------------------------------
    case A_DOWN_B_UP:
      if (currentTime - lastStateChangeTime >= MOTOR_DOWN_MS) { // Wait for the longest motor duration.
        stopAllMotors();
        Serial.println("State: Wait (B up)");
        currentState = WAIT_WITH_B_UP;
        lastStateChangeTime = currentTime;
      }
      break;

    // -----------------------------------------------------------------
    //  WAIT_WITH_B_UP: Motor B is up, waiting.
    // -----------------------------------------------------------------
    case WAIT_WITH_B_UP:
      // If homing is requested, start returning motor B.
      if (homing_requested) {
        Serial.println("Homing: Motor B returning.");
        motorB_down();
        currentState = HOMING;
        lastStateChangeTime = currentTime;
      }
      // Otherwise, continue the normal iteration after the wait time.
      else if (currentTime - lastStateChangeTime >= ITERATION_WAIT_MS) {
        Serial.println("State: B down, A up (Loop)");
        motorB_down();
        motorA_up();
        currentState = B_DOWN_A_UP;
        lastStateChangeTime = currentTime;
      }
      break;

    // -----------------------------------------------------------------
    //  B_DOWN_A_UP: Motor B moves down, Motor A moves up.
    // -----------------------------------------------------------------
    case B_DOWN_A_UP:
      if (currentTime - lastStateChangeTime >= MOTOR_DOWN_MS) { // Wait for the longest motor duration.
        stopAllMotors();
        Serial.println("State: Wait (A up)");
        currentState = WAIT_WITH_A_UP; // Loop back to the wait state.
        lastStateChangeTime = currentTime;
      }
      break;
  }
}

// Motor Control Functions
void motorA_up() { digitalWrite(MOTOR_A_IN1, HIGH); digitalWrite(MOTOR_A_IN2, LOW); }
void motorA_down() { digitalWrite(MOTOR_A_IN1, LOW); digitalWrite(MOTOR_A_IN2, HIGH); }
void motorB_up() { digitalWrite(MOTOR_B_IN1, HIGH); digitalWrite(MOTOR_B_IN2, LOW); }
void motorB_down() { digitalWrite(MOTOR_B_IN1, LOW); digitalWrite(MOTOR_B_IN2, HIGH); }

void stopAllMotors() {
  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_B_IN1, LOW);
  digitalWrite(MOTOR_B_IN2, LOW);
}