/*
  Pressure-Triggered Iteration Motor Control with Degree Management

  This sketch uses a state machine to control an alternating motor sequence
  based on input from pressure sensors, incorporating motor degree management
  and smooth transitions.

  Primary logic:
  - Default State: Motors are stopped in the 'down' position (0 degrees).
  - Homing: When pressure is lost, active motors return to the 'down' position (0 degrees).
  - Iteration: When pressure is detected, a timed sequence of motor
    movements begins and repeats until pressure is lost.
  - Degree Management: Motor positions are tracked as degrees, preventing
    movement below 0 degrees and ensuring smooth transitions.
*/

#include <WiFi.h>

const char* ssid = "m"; // REPLACE WITH YOUR WIFI SSID
const char* password = "aaaa4444"; // REPLACE WITH YOUR WIFI PASSWORD

WiFiServer server(80); // Create a server on port 80

// Pin Definitions
bool DEV_MODE = false; // Set to true to bypass pressure sensor and run continuously
const int PRESSURE_SENSOR_PINS[6] = {39, 36, 34, 35, 32, 33};
const int MOTOR_A_IN1 = 12; // Left Motor
const int MOTOR_A_IN2 = 13;
const int MOTOR_B_IN1 = 23;  // Right Motor
const int MOTOR_B_IN2 = 5;

// System Constants
const int PRESSURE_THRESHOLD = 1000;       // Minimum total pressure to begin iteration.
const unsigned long ITERATION_WAIT_MS = 3000; // 10-second wait duration.

// Motor Control Constants
const float MAX_MOTOR_DEGREE =45.0;      // Maximum degree the motor can travel from its home position (0 degrees).
const float MOTOR_SPEED_DEG_PER_MS = 0.05; // Degrees per millisecond (needs tuning based on motor and gear ratio)
const unsigned long HOMING_TIMEOUT_MS = 5000; // Timeout for homing operation

// State Machine
enum State { IDLE, HOMING, A_MOVING_UP, WAIT_WITH_A_UP, A_MOVING_DOWN_B_MOVING_UP, WAIT_WITH_B_UP, B_MOVING_DOWN_A_MOVING_UP };
State currentState = IDLE;
unsigned long lastStateChangeTime = 0;
bool homing_requested = false;

// Motor Degree Management Variables
float motorA_current_degree = 0.0;
float motorB_current_degree = 0.0;

enum MotorDirection { STOPPED, UP, DOWN };
MotorDirection motorA_direction = STOPPED;
MotorDirection motorB_direction = STOPPED;

unsigned long motorA_last_update_time = 0;
unsigned long motorB_last_update_time = 0;

// Target degrees for each motor
float motorA_target_degree = 0.0;
float motorB_target_degree = 0.0;

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

  // Connect to Wi-Fi
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // Start the server
  server.begin();
  Serial.println("Web server started.");
}

void loop() {
  unsigned long currentTime = millis();

  // =================================================================
  //  0. SERIAL COMMAND HANDLING
  // =================================================================
  if (Serial.available()) {
    char command = Serial.read();
    if (command == 'c') {
      Serial.print("WiFi Status: ");
      Serial.println(WiFi.status());
      Serial.print("IP Address: ");
      Serial.println(WiFi.localIP());
    } else if (command == '9') {
      DEV_MODE = !DEV_MODE; // Toggle DEV_MODE
      stopAllMotors(); // Stop all motors on mode change
      currentState = IDLE; // Reset state on mode change
      if (DEV_MODE) {
        Serial.println("DEV_MODE ENABLED: Manual control only. Use 1,2,3,4 for direct motor control, 0 to stop, v to reset degrees. WiFi commands also active.");
      } else {
        Serial.println("DEV_MODE DISABLED: Normal operation (state machine active).");
      }
    } else if (DEV_MODE) { // Only process motor commands if DEV_MODE is enabled
      switch (command) {
        case '1': // Motor A Up
          motorA_target_degree = MAX_MOTOR_DEGREE;
          setMotorDirection(motorA_direction, UP);
          Serial.println("Motor A moving UP");
          break;
        case '2': // Motor A Down
          motorA_target_degree = 0.0;
          setMotorDirection(motorA_direction, DOWN);
          Serial.println("Motor A moving DOWN");
          break;
        case '3': // Motor B Up
          motorB_target_degree = MAX_MOTOR_DEGREE;
          setMotorDirection(motorB_direction, UP);
          Serial.println("Motor B moving UP");
          break;
        case '4': // Motor B Down
          motorB_target_degree = 0.0;
          setMotorDirection(motorB_direction, DOWN);
          Serial.println("Motor B moving DOWN");
          break;
        case '0': // Stop All Motors
          stopAllMotors();
          Serial.println("All Motors STOPPED");
          break;
        case 'v': // Value Reset (reset accumulated degrees to 0)
          motorA_current_degree = 0.0;
          motorB_current_degree = 0.0;
          stopAllMotors(); // Ensure motors are stopped when resetting degrees
          Serial.println("Motor degrees reset to 0.");
          break;
        default:
          Serial.print("Unknown command in DEV_MODE: ");
          Serial.println(command);
          break;
      }
    }
  }

  // Handle WiFi Commands
  WiFiClient client = server.available();
  if (client) {
    Serial.println("New Client.");
    String currentLine = "";
    while (client.connected() && client.available()) {
      char c = client.read();
      Serial.write(c);
      if (c == '\n') {
        if (currentLine.length() == 0) {
          // HTTP headers received, now process command
          if (DEV_MODE) {
            // Extract command from URL (e.g., /command?cmd=1)
            int cmdIndex = currentLine.indexOf("cmd=");
            if (cmdIndex != -1) {
              char command = currentLine.charAt(cmdIndex + 4);
              switch (command) {
                case '1': // Motor A Up
                  motorA_target_degree = MAX_MOTOR_DEGREE;
                  setMotorDirection(motorA_direction, UP);
                  client.println("Motor A moving UP");
                  break;
                case '2': // Motor A Down
                  motorA_target_degree = 0.0;
                  setMotorDirection(motorA_direction, DOWN);
                  client.println("Motor A moving DOWN");
                  break;
                case '3': // Motor B Up
                  motorB_target_degree = MAX_MOTOR_DEGREE;
                  setMotorDirection(motorB_direction, UP);
                  client.println("Motor B moving UP");
                  break;
                case '4': // Motor B Down
                  motorB_target_degree = 0.0;
                  setMotorDirection(motorB_direction, DOWN);
                  client.println("Motor B moving DOWN");
                  break;
                case '0': // Stop All Motors
                  stopAllMotors();
                  client.println("All Motors STOPPED");
                  break;
                case 'v': // Value Reset (reset accumulated degrees to 0)
                  motorA_current_degree = 0.0;
                  motorB_current_degree = 0.0;
                  stopAllMotors(); // Ensure motors are stopped when resetting degrees
                  client.println("Motor degrees reset to 0.");
                  break;
                default:
                  client.print("Unknown command in DEV_MODE: ");
                  client.println(command);
                  break;
              }
            } else {
              client.println("HTTP/1.1 200 OK");
              client.println("Content-type:text/html");
              client.println();
              client.print("Invalid command format. Use /command?cmd=X");
            }
          } else {
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();
            client.print("DEV_MODE is OFF. Enable with serial '9' for manual control.");
          }
          break;
        } else { // if you got a newline, then clear currentLine
          currentLine = "";
        }
      } else if (c != '\r') { // if you got anything else but a carriage return character, add it to the end of the currentLine
        currentLine += c;
      }
    }
    // Close the connection
    client.stop();
    Serial.println("Client Disconnected.");
  }

  // =================================================================
  //  1. SENSOR READING & PRESENCE DETECTION
  // =================================================================
  bool userIsPresent;
  if (DEV_MODE) {
    userIsPresent = true; // Always present in dev mode for state machine bypass

    // In DEV_MODE, read and display individual pressure sensor values if pressed
    int dev_sumPressure = 0;
    String pressure_readings = "";
    for (int i = 0; i < 6; i++) {
      int sensor_val = analogRead(PRESSURE_SENSOR_PINS[i]);
      dev_sumPressure += sensor_val;
      pressure_readings += "P" + String(i) + ":" + String(sensor_val) + " ";
    }
    if (dev_sumPressure > PRESSURE_THRESHOLD) {
      Serial.print("DEV_MODE Pressure: ");
      Serial.println(pressure_readings);
    }

  } else {
    int sumPressure = 0;
    for (int pin : PRESSURE_SENSOR_PINS) {
      sumPressure += analogRead(pin);
    }
    userIsPresent = sumPressure > PRESSURE_THRESHOLD;
  }

  // If user is not present, set a flag to request homing.
  if (!userIsPresent) {
    homing_requested = true;
  }

  // =================================================================
  //  2. MOTOR POSITION UPDATE (Continuous)
  // =================================================================
  updateMotorPosition(currentTime, motorA_current_degree, motorA_direction, motorA_last_update_time, MOTOR_A_IN1, MOTOR_A_IN2);
  updateMotorPosition(currentTime, motorB_current_degree, motorB_direction, motorB_last_update_time, MOTOR_B_IN1, MOTOR_B_IN2);

  // =================================================================
  //  3. STATE MACHINE LOGIC (Only runs if DEV_MODE is OFF)
  // =================================================================
  if (!DEV_MODE) {
    switch (currentState) {
    // -----------------------------------------------------------------
    //  IDLE: Waiting for a user.
    // -----------------------------------------------------------------
    case IDLE:
      if (userIsPresent) {
        Serial.println("Pressure detected. Starting sequence.");
        homing_requested = false; // Reset homing flag
        motorA_target_degree = MAX_MOTOR_DEGREE;
        setMotorDirection(motorA_direction, UP);
        lastStateChangeTime = currentTime;
        currentState = A_MOVING_UP;
      }
      break;

    // -----------------------------------------------------------------
    //  HOMING: Motors returning to 0 degrees.
    // -----------------------------------------------------------------
    case HOMING:
      motorA_target_degree = 0.0;
      motorB_target_degree = 0.0;
      setMotorDirection(motorA_direction, DOWN);
      setMotorDirection(motorB_direction, DOWN);

      // Check if both motors are at 0 degrees or if homing timed out
      if ((motorA_current_degree <= 0.0 && motorB_current_degree <= 0.0) || (currentTime - lastStateChangeTime >= HOMING_TIMEOUT_MS)) {
        stopAllMotors();
        Serial.println("Homing complete. State: IDLE");
        currentState = IDLE;
      }
      break;

    // -----------------------------------------------------------------
    //  A_MOVING_UP: Motor A is moving up to MAX_MOTOR_DEGREE.
    // -----------------------------------------------------------------
    case A_MOVING_UP:
      if (homing_requested) {
        Serial.println("Homing requested during A_MOVING_UP.");
        lastStateChangeTime = currentTime; // Reset timer for homing timeout
        currentState = HOMING;
      } else if (motorA_current_degree >= MAX_MOTOR_DEGREE) {
        stopMotor(motorA_direction, MOTOR_A_IN1, MOTOR_A_IN2);
        Serial.println("State: Wait (A up)");
        currentState = WAIT_WITH_A_UP;
        lastStateChangeTime = currentTime;
      }
      break;

    // -----------------------------------------------------------------
    //  WAIT_WITH_A_UP: Motor A is up, waiting.
    // -----------------------------------------------------------------
    case WAIT_WITH_A_UP:
      if (homing_requested) {
        Serial.println("Homing requested during WAIT_WITH_A_UP.");
        lastStateChangeTime = currentTime; // Reset timer for homing timeout
        currentState = HOMING;
      } else if (currentTime - lastStateChangeTime >= ITERATION_WAIT_MS) {
        Serial.println("State: A down, B up");
        motorA_target_degree = 0.0;
        motorB_target_degree = MAX_MOTOR_DEGREE;
        setMotorDirection(motorA_direction, DOWN);
        setMotorDirection(motorB_direction, UP);
        lastStateChangeTime = currentTime;
        currentState = A_MOVING_DOWN_B_MOVING_UP;
      }
      break;

    // -----------------------------------------------------------------
    //  A_MOVING_DOWN_B_MOVING_UP: Motor A moves down, Motor B moves up.
    // -----------------------------------------------------------------
    case A_MOVING_DOWN_B_MOVING_UP:
      if (homing_requested) {
        Serial.println("Homing requested during A_MOVING_DOWN_B_MOVING_UP.");
        lastStateChangeTime = currentTime; // Reset timer for homing timeout
        currentState = HOMING;
      } else if (motorA_current_degree <= 0.0 && motorB_current_degree >= MAX_MOTOR_DEGREE) {
        stopMotor(motorA_direction, MOTOR_A_IN1, MOTOR_A_IN2);
        stopMotor(motorB_direction, MOTOR_B_IN1, MOTOR_B_IN2);
        Serial.println("State: Wait (B up)");
        currentState = WAIT_WITH_B_UP;
        lastStateChangeTime = currentTime;
      }
      break;

    // -----------------------------------------------------------------
    //  WAIT_WITH_B_UP: Motor B is up, waiting.
    // -----------------------------------------------------------------
    case WAIT_WITH_B_UP:
      if (homing_requested) {
        Serial.println("Homing requested during WAIT_WITH_B_UP.");
        lastStateChangeTime = currentTime; // Reset timer for homing timeout
        currentState = HOMING;
      } else if (currentTime - lastStateChangeTime >= ITERATION_WAIT_MS) {
        Serial.println("State: B down, A up (Loop)");
        motorB_target_degree = 0.0;
        motorA_target_degree = MAX_MOTOR_DEGREE;
        setMotorDirection(motorB_direction, DOWN);
        setMotorDirection(motorA_direction, UP);
        lastStateChangeTime = currentTime;
        currentState = B_MOVING_DOWN_A_MOVING_UP;
      }
      break;

    // -----------------------------------------------------------------
    //  B_MOVING_DOWN_A_MOVING_UP: Motor B moves down, Motor A moves up.
    // -----------------------------------------------------------------
    case B_MOVING_DOWN_A_MOVING_UP:
      if (homing_requested) {
        Serial.println("Homing requested during B_MOVING_DOWN_A_MOVING_UP.");
        lastStateChangeTime = currentTime; // Reset timer for homing timeout
        currentState = HOMING;
      } else if (motorB_current_degree <= 0.0 && motorA_current_degree >= MAX_MOTOR_DEGREE) {
        stopMotor(motorB_direction, MOTOR_B_IN1, MOTOR_B_IN2);
        stopMotor(motorA_direction, MOTOR_A_IN1, MOTOR_A_IN2);
        Serial.println("State: Wait (A up)");
        currentState = WAIT_WITH_A_UP; // Loop back to the wait state.
        lastStateChangeTime = currentTime;
      }
      break;
  }
  } // End of if (!DEV_MODE)
}

// Motor Control Functions
void setMotorDirection(MotorDirection& motor_dir_var, MotorDirection new_direction) {
  if (motor_dir_var != new_direction) {
    motor_dir_var = new_direction;
  }
}

void stopMotor(MotorDirection& motor_dir_var, int in1_pin, int in2_pin) {
  motor_dir_var = STOPPED;
  digitalWrite(in1_pin, LOW);
  digitalWrite(in2_pin, LOW);
}

void stopAllMotors() {
  stopMotor(motorA_direction, MOTOR_A_IN1, MOTOR_A_IN2);
  stopMotor(motorB_direction, MOTOR_B_IN1, MOTOR_B_IN2);
}

void updateMotorPosition(unsigned long currentTime, float& current_degree, MotorDirection& direction, unsigned long& last_update_time, int in1_pin, int in2_pin) {
  if (direction == STOPPED) {
    // Ensure motor pins are low if stopped
    digitalWrite(in1_pin, LOW);
    digitalWrite(in2_pin, LOW);
    last_update_time = currentTime; // Reset last update time when stopped
    return;
  }

  unsigned long time_delta = currentTime - last_update_time;
  if (time_delta == 0) return; // No time has passed, no update needed

  float degree_change = MOTOR_SPEED_DEG_PER_MS * time_delta;

  if (direction == UP) {
    current_degree += degree_change;
    if (current_degree > MAX_MOTOR_DEGREE) {
      current_degree = MAX_MOTOR_DEGREE;
      direction = STOPPED; // Stop when max degree is reached
    }
    digitalWrite(in1_pin, HIGH);
    digitalWrite(in2_pin, LOW);
  } else if (direction == DOWN) {
    current_degree -= degree_change;
    if (current_degree < 0.0) {
      current_degree = 0.0;
      direction = STOPPED; // Stop when 0 degree is reached
    }
    digitalWrite(in1_pin, LOW);
    digitalWrite(in2_pin, HIGH);
  }
  if (DEV_MODE) {
    Serial.print("Motor (IN1:"); Serial.print(in1_pin); Serial.print(") Degree: "); Serial.println(current_degree);
  }
  last_update_time = currentTime;
}
