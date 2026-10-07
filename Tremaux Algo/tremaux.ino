/*
 * ============================================================
 * TECHFEST MEShMERIZE
 * Arduino Nano + L298N + 2 DC Motors + 8 IR Sensors
 * ============================================================
 *
 * DRY RUN:
 *   Sensor -> PID line following
 *          -> Junction detection
 *          -> Trémaux decision
 *          -> Record route
 *          -> Reach goal
 *          -> Optimize route
 *
 * ACTUAL RUN:
 *   Sensor -> PID line following
 *          -> Read stored optimized route
 *          -> Execute turns
 *          -> Reach goal
 *
 * IMPORTANT:
 * This is the integrated software architecture.
 * Sensor thresholds, PID constants, turning times and
 * junction patterns MUST be calibrated on the real robot.
 *
 * ============================================================
 */

// ============================================================
//                         RUN MODE
// ============================================================

// Change this during development/testing.
// In competition, do NOT alter deposited code.
#include "MazeGraph.h"
#include <cstdint>
#define DRY_RUN       0
#define ACTUAL_RUN    1
#define RUN_MODE DRY_RUN


// ============================================================
//                      MOTOR PINS
// ============================================================
//
// Your actual L298N configuration
//

#define ENA 3
#define IN1 5
#define IN2 6
#define ENB 11
#define IN3 9
#define IN4 10


// ============================================================
//                       IR SENSOR PINS
// ============================================================
//
// D3 is already occupied by ENA.
// Therefore the 8 sensors use:
//
// A0 A1 A2 A3 A4 A5 D2 D4
//
// Change these if your actual IR array is wired differently.
//
const uint8_t SENSOR_COUNT = 8;
const uint8_t sensorPins[SENSOR_COUNT] = {
  A0, A1, A2, A3, A4, A5, 2, 4
};


// ============================================================
//                   SENSOR CONFIGURATION
// ============================================================
// Set this according to your IR sensor output.
// If black line produces LOW:
//     true
// If black line produces HIGH:
//     false
//
const bool SENSOR_ACTIVE_LOW = true;
// Sensor positions.
// Left -> Right
// You will eventually calibrate these values.
const float sensorWeight[SENSOR_COUNT] = {
  -3.5,
  -2.5,
  -1.5,
  -0.5,
   0.5,
   1.5,
   2.5,
   3.5
};


// ============================================================
//                         LED
// ============================================================
#define END_LED 13


// ============================================================
//                         SPEED
// ============================================================
int speedVal = 150;

// Minimum useful motor speed.
// We will determine this experimentally.
const int MIN_SPEED = 0;
const int MAX_SPEED = 255;

// ============================================================
//                           PID
// ============================================================
float Kp = 30.0;
float Ki = 0.0;
float Kd = 10.0;
float integral = 0.0;
float previousError = 0.0;
const float MAX_INTEGRAL = 20.0;

// ============================================================
//                    SENSOR THRESHOLD
// ============================================================
// THIS IS ONLY A STARTING VALUE.
// You MUST print raw sensor values and determine
// the correct threshold for your particular array.
int sensorThreshold = 500;

// ============================================================
//                    JUNCTION DETECTION
// ============================================================
const int JUNCTION_SENSOR_COUNT = 5;


// ============================================================
//                    TURN CONFIGURATION
// ============================================================
int turnSpeed = 150;
// Initial values only.
// These MUST be calibrated.
unsigned long LEFT_TURN_TIME  = 300;
unsigned long RIGHT_TURN_TIME = 300;
unsigned long UTURN_TIME      = 600;

// ============================================================
//                         PATH
// ============================================================
const int MAX_PATH_LENGTH = 100;
char rawPath[MAX_PATH_LENGTH];
int rawPathLength = 0;
char optimizedPath[MAX_PATH_LENGTH];
int optimizedPathLength = 0;
int actualPathIndex = 0;

// ============================================================
//                       ENUMERATIONS
// ============================================================

enum Direction {LEFT, STRAIGHT, RIGHT, BACK, NONE};
enum RobotState {STARTUP, FOLLOWING_LINE, AT_JUNCTION, TURNING, FINISHED};
RobotState state = STARTUP;

// ============================================================
//                   JUNCTION STRUCTURE
// ============================================================
struct JunctionOptions {
  bool left;
  bool straight;
  bool right;
};

// ============================================================
//                    MOTOR FUNCTIONS
// ============================================================
// This section is adapted directly from your working
// motor-control code.
void setupMotors() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  stopMotors();
}

void setMotorSpeed(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);
  // ---------------- LEFT MOTOR ----------------
  if (leftSpeed > 0) {
    analogWrite(ENA, leftSpeed);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  else if (leftSpeed < 0) {
    analogWrite(ENA, -leftSpeed);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  else {
    analogWrite(ENA, 0);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }

  // ---------------- RIGHT MOTOR ----------------
  if (rightSpeed > 0) {
    analogWrite(ENB, rightSpeed);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }
  else if (rightSpeed < 0) {
    analogWrite(ENB, -rightSpeed);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
  else {
    analogWrite(ENB, 0);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }
}

void forward() {
  setMotorSpeed(speedVal, speedVal);
}
void backward() {
  setMotorSpeed(-speedVal, -speedVal);
}
void left() {
  // Left motor backward
  // Right motor forward
  setMotorSpeed(-speedVal,speedVal);
}
void right() {
  setMotorSpeed(speedVal, -speedVal);
}
void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}


// ============================================================
//                     SENSOR FUNCTIONS
// ============================================================

bool sensorSeesLine(uint8_t sensor) {
  int value = analogRead(sensorPins[sensor]);
  if (SENSOR_ACTIVE_LOW) {
    return value < sensorThreshold;
  }
  else {
    return value > sensorThreshold;
  }
}
void readSensors(bool sensors[]) {
  for (int i = 0; i < SENSOR_COUNT; i++) {
    sensors[i] = sensorSeesLine(i);
  }
}

// ============================================================
//                  SENSOR DEBUGGING
// ============================================================
void printSensorValues() {
  for (int i = 0; i < SENSOR_COUNT; i++) {
    int value = analogRead(sensorPins[i]);
    Serial.print(value);
    if (i < SENSOR_COUNT - 1) Serial.print('\t');
  }
  Serial.println();
}

// ============================================================
//                    LINE POSITION
// ============================================================
float calculateLinePosition() {
  bool sensors[SENSOR_COUNT];
  readSensors(sensors);
  float weightedSum = 0.0;
  float total = 0.0;
  for (int i = 0; i < SENSOR_COUNT; i++) {
    if (sensors[i]) {
      weightedSum += sensorWeight[i];
      total += 1.0;
    }
  }
  /*
   * If no sensor sees the line,
   * retain the previous error.
   */
  if (total == 0) {
    return previousError;
  }
  return weightedSum / total;
}


// ============================================================
//                         PID
// ============================================================

int calculatePID(float error) {
  /*
   * Approximate loop interval.
   *
   * Later we should replace this with
   * actual elapsed time using micros().
   */
  const float dt = 0.01;
  integral += error * dt;
  // Anti-windup
  integral = constrain(integral, -MAX_INTEGRAL, MAX_INTEGRAL);
  float derivative = (error - previousError) / dt;
  float output = Kp * error + Ki * integral + Kd * derivative;
  previousError = error;
  return (int)output;
}

void followLinePID() {
  float error = calculateLinePosition();
  int correction = calculatePID(error);
  int leftSpeed = speedVal + correction;
  int rightSpeed = speedVal - correction;
  leftSpeed = constrain(leftSpeed, MIN_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, MIN_SPEED, MAX_SPEED);
  setMotorSpeed(leftSpeed, rightSpeed);
}

// ============================================================
//                   JUNCTION DETECTION
// ============================================================
int countActiveSensors() {
  bool sensors[SENSOR_COUNT];
  readSensors(sensors);
  int count = 0;
  for (int i = 0; i < SENSOR_COUNT; i++) {
    if (sensors[i]) count++;
  }
  return count;
}

bool isPotentialJunction() {
  int active = countActiveSensors();
  return active >= JUNCTION_SENSOR_COUNT;
}

// ============================================================
//                JUNCTION CLASSIFICATION
// ============================================================
// This is a FIRST VERSION.
// It is NOT yet guaranteed to correctly classify
// every junction in the competition maze.
// We will calibrate this using the actual sensor
// geometry.
JunctionOptions classifyJunction() {
  bool s[SENSOR_COUNT];
  readSensors(s);
  JunctionOptions j;
  // Left branch indication
  j.left = s[0] || s[1] || s[2];
  // Right branch indication
  j.right = s[5] || s[6] || s[7];
  // Straight line indication
  j.straight = s[3] || s[4];
  return j;
}


// ============================================================
//                       GOAL DETECTION
// ============================================================
// Competition goal:
// 400 mm x 400 mm white box.
// This first implementation assumes a broad
// sensor response.
// MUST be calibrated.
bool goalDetected() {
  bool sensors[SENSOR_COUNT];
  readSensors(sensors);
  int active = 0;
  for (int i = 0; i < SENSOR_COUNT; i++) {
    if (sensors[i]) active++;
  }
  return active >= 7;
}

void finishRobot() {
  stopMotors();
  digitalWrite(END_LED, HIGH);
  state = FINISHED;
}

// ============================================================
//                        TURNS
// ============================================================
void turnLeft() {
  state = TURNING;
  setMotorSpeed(-turnSpeed, turnSpeed);
  delay(LEFT_TURN_TIME);
  stopMotors();
  integral = 0;
}
void turnRight() {
  state = TURNING;
  setMotorSpeed(turnSpeed, -turnSpeed);
  delay(RIGHT_TURN_TIME);
  stopMotors();
  integral = 0;
}
void turnBack() {
  state = TURNING;
  setMotorSpeed(turnSpeed, -turnSpeed);
  delay(UTURN_TIME);
  stopMotors();
  integral = 0;
}

// ============================================================
//                   EXECUTE DIRECTION
// ============================================================

void executeDirection(Direction direction) {
  switch (direction) {
    case LEFT:
      turnLeft();
      break;
    case RIGHT:
      turnRight();
      break;
    case BACK:
      turnBack();
      break;
    case STRAIGHT:
      // Continue forward.
      break;
    case NONE:
      stopMotors();
      break;
  }
  state = FOLLOWING_LINE;
}

// ============================================================
//                    PATH RECORDING
// ============================================================
char directionToChar(Direction direction) {
  switch (direction) {
    case LEFT:
      return 'L';
    case STRAIGHT:
      return 'S';
    case RIGHT:
      return 'R';
    case BACK:
      return 'B';
    default:
      return '?';
  }
}

void recordMove(Direction direction) {
  if (rawPathLength >= MAX_PATH_LENGTH) {
    return;
  }
  rawPath[rawPathLength++] = directionToChar(direction);
}


// ============================================================
//                       TRÉMAUX
// ============================================================
//
// Simplified junction-level implementation.
//
// 0 = unexplored
// 1 = visited
//
// A full graph-based implementation should replace
// this after the sensor/junction layer is tested.
//

uint8_t leftVisits = 0;
uint8_t straightVisits = 0;
uint8_t rightVisits = 0;


Direction chooseTremauxDirection(
  JunctionOptions j
) {

  /*
   * Exploration priority:
   *
   * LEFT
   * STRAIGHT
   * RIGHT
   *
   * among unexplored paths.
   */


  if (j.left && leftVisits == 0) {
    leftVisits++;
    return LEFT;
  }

  if (j.straight && straightVisits == 0) {
    straightVisits++;
    return STRAIGHT;
  }

  if (j.right && rightVisits == 0) {
    rightVisits++;
    return RIGHT;
  }

  /*
   * Everything currently available has
   * been explored.
   *
   * Backtrack.
   */

  return BACK;
}

// ============================================================
//                    PATH OPTIMIZATION
// ============================================================

char reduceThree(char a, char b, char c) {

  /*
   * Directions represented as angles.
   *
   * L = -90
   * S =   0
   * R = +90
   * B = 180
   */

  int total = 0;
  char dirs[4] = {'L','S','R','B'};
  int angles[4] = {-90, 0, 90, 180};
  char input[3] = {a, b, c};

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 4; j++) {
      if (input[i] == dirs[j]) {
        total += angles[j];
        break;
      }
    }
  }
  total %= 360;
  if (total < 0) total += 360;
  if (total == 0) return 'S';
  if (total == 90) return 'R';
  if (total == 180) return 'B';
  if (total == 270) return 'L';
  return '?';
}


void optimizePath() {
  char buffer[MAX_PATH_LENGTH];
  int length = 0;
  for (int i = 0; i < rawPathLength; i++) {
    buffer[length++] = rawPath[i];
  }
  bool changed = true;
  while (changed) {
    changed = false;
    for (int i = 0; i < length - 2; i++) {
      /*
       * Only attempt reduction when
       * a BACK occurs.
       */

      if (buffer[i] == 'B' || buffer[i + 1] == 'B' || buffer[i + 2] == 'B') {
        char replacement = reduceThree(buffer[i], buffer[i + 1], buffer[i + 2]);
        if (replacement != '?') {
          buffer[i] = replacement;
          for (int j = i + 1; j < length - 2; j++) {
            buffer[j] = buffer[j + 2];
          }
          length -= 2;
          changed = true;
          break;
        }
      }
    }
  }


  optimizedPathLength = length;
  for (int i = 0; i < length; i++) {
    optimizedPath[i] = buffer[i];
  }
}


// ============================================================
//                  ACTUAL RUN PATH
// ============================================================

Direction charToDirection(char c) {
  switch (c) {
    case 'L':
      return LEFT;
    case 'S':
      return STRAIGHT;
    case 'R':
      return RIGHT;
    case 'B':
      return BACK;
    default:
      return NONE;
  }
}


void executeStoredPath() {
  if (actualPathIndex >= optimizedPathLength) {
    return;
  }
  Direction direction = charToDirection(optimizedPath[actualPathIndex]);
  actualPathIndex++;
  executeDirection(direction);
}


// ============================================================
//                  CHECKPOINT HOOK
// ============================================================
// The competition document specifies checkpoint
// scoring, but does not specify an electronic
// checkpoint-detection mechanism. Therefore this remains a separate module.

void checkCheckpoint() {
  /*
   * IMPLEMENT AFTER TESTING THE ACTUAL
   * CHECKPOINT MARKINGS.
   *
   * Example:
   *
   * if (checkpointDetected()) {
   *     markCheckpoint();
   * }
   */
}

// ============================================================
//                     DEBUG FUNCTIONS
// ============================================================
void printRawPath() {
  Serial.println("RAW PATH:");

  for (int i = 0; i < rawPathLength; i++) {
    Serial.print(rawPath[i]);
    Serial.print(' ');
  }
  Serial.println();
}

void printOptimizedPath() {
  Serial.println("OPTIMIZED PATH:");
  for (int i = 0; i < optimizedPathLength; i++) {
    Serial.print(optimizedPath[i]);
    Serial.print(' ');
  }
  Serial.println();
}

// ============================================================
//                         DRY RUN
// ============================================================

void dryRun() {
  state = FOLLOWING_LINE;
  while (state != FINISHED) {
    // Goal always has priority.
    if (goalDetected()) {
      finishRobot();
      break;
    }
    /*
     * Check checkpoint independently.
     */
    checkCheckpoint();
    /*
     * Normal line following.
     */
    if (!isPotentialJunction()) {
      followLinePID();
      continue;
    }
    /*
     * We have reached a junction.
     */
    state = AT_JUNCTION;
    JunctionOptions junction = classifyJunction();
    /*
     * Trémaux chooses what to do.
     */
    Direction next = chooseTremauxDirection(junction);
    /*
     * Remember the decision.
     */
    recordMove(next);
    /*
     * Physically execute it.
     */
    executeDirection(next);
  }
  /*
   * Goal reached.
   *
   * Now derive the route for the
   * actual run.
   */
  optimizePath();
  printRawPath();
  printOptimizedPath();
}

// ============================================================
//                       ACTUAL RUN
// ============================================================
void actualRun() {
  state = FOLLOWING_LINE;
  actualPathIndex = 0;
  while (state != FINISHED) {
    /*
     * Goal detection has highest priority.
     */
    if (goalDetected()) {
      finishRobot();
      break;
    }
    /*
     * Normally PID keeps the robot
     * on the line.
     */
    if (!isPotentialJunction()) {
      followLinePID();
      continue;
    }
    /*
     * Junction reached.
     *
     * Instead of asking Trémaux what to do,
     * retrieve the previously learned route.
     */
    executeStoredPath();
  }
}

// ============================================================
//                         SETUP
// ============================================================
void setup() {
  Serial.begin(9600);
  // ---------------- MOTOR SETUP ----------------
  setupMotors();
  // ---------------- SENSOR SETUP ----------------
  for (int i = 0; i < SENSOR_COUNT; i++) {
    pinMode(sensorPins[i], INPUT);
  }
  // ---------------- LED ----------------
  pinMode(END_LED, OUTPUT);
  digitalWrite(END_LED, LOW);
  stopMotors();
  Serial.println("MEShMERIZE ROBOT READY");
  delay(1000);
}

// ============================================================
//                          LOOP
// ============================================================
void loop() {
  // if (Serial.available()) {

  //   char cmd = Serial.read();

  //   if (cmd == 'F') forward();
  //   if (cmd == 'B') backward();
  //   if (cmd == 'L') left();
  //   if (cmd == 'R') right();
  //   if (cmd == 'S') stopMotors();
  // }
#if RUN_MODE == DRY_RUN
  dryRun();
#elif RUN_MODE == ACTUAL_RUN
  actualRun();
#endif
  /*
   * Prevent accidental restart.
   */

  stopMotors();
  while (true) { delay(1000);}
}
