/*
 * ============================================================
 * TECHFEST MESHMErIZE
 *
 * Arduino Nano
 * L298N
 * 2 DC Motors
 * 8 IR Sensors
 *
 * FINAL SOFTWARE ARCHITECTURE
 *
 * DRY RUN:
 *   Sensors
 *      -> PID line following
 *      -> Junction detection
 *      -> Junction classification
 *      -> Trémaux-style exploration
 *      -> Record route
 *      -> Reach goal
 *      -> Optimize route
 *
 * ACTUAL RUN:
 *   Sensors
 *      -> PID line following
 *      -> Read optimized route
 *      -> Execute turns
 *      -> Reach goal
 *
 * ============================================================
 */
#include <Arduino.h>
#include "MazeGraph.h"
// ============================================================
// RUN MODE
// ============================================================
#define DRY_RUN     0
#define ACTUAL_RUN  1
#define RUN_MODE DRY_RUN
// ============================================================
// MOTOR PINS
// ============================================================
#define ENA 3
#define IN1 5
#define IN2 6
#define ENB 11
#define IN3 9
#define IN4 10
// ============================================================
// IR SENSOR PINS
// ============================================================
const uint8_t SENSOR_COUNT = 8;
const uint8_t sensorPins[SENSOR_COUNT] = {A0, A1, A2, A3, A4, A5, 2, 4};
// ============================================================
// SENSOR CONFIGURATION
// ============================================================
// true  -> line produces LOW
// false -> line produces HIGH
const bool SENSOR_ACTIVE_LOW = true;
const int sensorThreshold = 500;
// Sensor positions: left -> right
const float sensorWeight[SENSOR_COUNT] = {-3.5,-2.5,-1.5,-0.5,0.5,1.5,2.5,3.5};
// ============================================================
// LED
// ============================================================
#define END_LED 13
// ============================================================
// SPEED
// ============================================================
int speedVal = 150;
const int MIN_SPEED = 0;
const int MAX_SPEED = 255;
int turnSpeed = 150;
// ============================================================
// TURN TIMINGS
// ============================================================
// These are starting values.
// They must be calibrated on the actual robot.
// ============================================================
unsigned long LEFT_TURN_TIME  = 300;
unsigned long RIGHT_TURN_TIME = 300;
unsigned long UTURN_TIME      = 600;
// ============================================================
// PID
// ============================================================
float Kp = 30.0;
float Ki = 0.0;
float Kd = 10.0;
float integral = 0.0;
float previousError = 0.0;
unsigned long previousPIDTime = 0;
bool pidInitialized = false;
const float MAX_INTEGRAL = 20.0;
// ============================================================
// PATH STORAGE
// ============================================================
const int MAX_PATH_LENGTH = 100;
Direction rawPath[MAX_PATH_LENGTH];
int rawPathLength = 0;
Direction optimizedPath[MAX_PATH_LENGTH];
int optimizedPathLength = 0;
int actualPathIndex = 0;
// ============================================================
// ROBOT STATE
// ============================================================
enum RobotState {STARTUP, FOLLOWING_LINE, AT_JUNCTION, TURNING, FINISHED};
RobotState state = STARTUP;

// ============================================================
// JUNCTION STRUCTURE
// ============================================================
struct JunctionOptions {
    bool left;
    bool straight;
    bool right;
};
// ============================================================
// MAZE GRAPH
// ============================================================
// The graph is available for the exploration layer.
// Physical node/edge construction will be populated
// after the real junction geometry is calibrated.
// ============================================================
MazeGraph maze;
// ============================================================
// MOTOR FUNCTIONS
// ============================================================
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
    setMotorSpeed(-speedVal, speedVal);
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
// SENSOR FUNCTIONS
// ============================================================
bool sensorSeesLine(uint8_t sensor) {
    int value = analogRead(sensorPins[sensor]);
    if (SENSOR_ACTIVE_LOW) {
        return value < sensorThreshold;
    }
    return value > sensorThreshold;
}


void readSensors(bool sensors[]) {
    for (int i = 0; i < SENSOR_COUNT; i++) {
        sensors[i] = sensorSeesLine(i);
    }
}

// ============================================================
// LINE POSITION
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
    // If line temporarily disappears,
    // continue using previous error.
    if (total == 0) {
        return previousError;
    }
    return weightedSum / total;
}
// ============================================================
// PID
// ============================================================
int calculatePID(float error) {
    unsigned long now = micros();
    float dt = 0.01;
    if (previousPIDTime != 0) {
        dt = (now - previousPIDTime) / 1000000.0;
        if (dt <= 0 || dt > 0.1) {
            dt = 0.01;
        }
    }
    previousPIDTime = now;
    if (!pidInitialized) {
        previousError = error;
        pidInitialized = true;
    }
    integral += error * dt;
    integral = constrain(integral,-MAX_INTEGRAL,MAX_INTEGRAL);
    float derivative = (error - previousError) / dt;
    float output = Kp * error + Ki * integral + Kd * derivative;
    previousError = error;
    return constrain((int)output, -255, 255);
    }

void resetPID() {
    integral = 0.0;
    previousError = 0.0;
    pidInitialized = false;
    previousPIDTime = micros();
}
// ============================================================
// LINE FOLLOWING
// ============================================================
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
// JUNCTION DETECTION
// ============================================================
int countActiveSensors() {
    bool sensors[SENSOR_COUNT];
    readSensors(sensors);
    int count = 0;
    for (int i = 0; i < SENSOR_COUNT; i++) {
        if (sensors[i]) {
            count++;
        }
    }
    return count;
}

bool isPotentialJunction() {
    int active = countActiveSensors();
    return active >= 5;
}

// ============================================================
// JUNCTION CLASSIFICATION
// ============================================================
// Left  = sensors 0,1,2
// Middle = sensors 3,4
// Right = sensors 5,6,7
// ============================================================
JunctionOptions classifyJunction() {
    bool sensors[SENSOR_COUNT];
    readSensors(sensors);
    JunctionOptions junction;
    junction.left = sensors[0] || sensors[1] || sensors[2];
    junction.straight = sensors[3] || sensors[4];
    junction.right = sensors[5] ||  sensors[6] || sensors[7];
    return junction;
}

// ============================================================
// GOAL DETECTION
// ============================================================
// Competition goal:
// 400 mm x 400 mm white box.
// This uses broad sensor activation as the
// initial software criterion.
// Actual threshold/pattern must be calibrated.
// ============================================================
bool goalDetected() {
    bool sensors[SENSOR_COUNT];
    readSensors(sensors);
    int active = 0;
    for (int i = 0; i < SENSOR_COUNT; i++) {
        if (sensors[i]) {
            active++;
        }
    }
    return active >= 7;
}

// ============================================================
// FINISH
// ============================================================
void finishRobot() {
    stopMotors();
    digitalWrite(END_LED, HIGH);
    state = FINISHED;
}

// ============================================================
// TURN FUNCTIONS
// ============================================================
void turnLeft() {
    state = TURNING;
    setMotorSpeed(-turnSpeed, turnSpeed);
    delay(LEFT_TURN_TIME);
    stopMotors();
    resetPID();
}

void turnRight() {
    state = TURNING;
    setMotorSpeed(-turnSpeed, turnSpeed);
    delay(RIGHT_TURN_TIME);
    stopMotors();
    resetPID();
}

void turnBack() {
    state = TURNING;
    setMotorSpeed(-turnSpeed, turnSpeed);
    delay(UTURN_TIME);
    stopMotors();
    resetPID();
}

// ============================================================
// EXECUTE DIRECTION
// ============================================================
void executeDirection(Direction direction) {
    state = TURNING;
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
            // Continue along the line.
            break;
        case NONE:
            stopMotors();
            break;
    }
    resetPID();
    state = FOLLOWING_LINE;
}

// ============================================================
// PATH RECORDING
// ============================================================
void recordMove(Direction direction) {
    if (rawPathLength >= MAX_PATH_LENGTH) {
        Serial.println("ERROR: PATH MEMORY FULL");
        return;
    }
    rawPath[rawPathLength] = direction;
    rawPathLength++;
}

// ============================================================
// TRÉMAUX EXPLORATION
// ============================================================
// IMPORTANT:
// This is deliberately kept separate from the graph.
// A real Trémaux algorithm needs EDGE-level visit counts,
// not global "left visited" variables.
// The MazeGraph provides the proper edge-level structure.
// Until physical junction -> graph-node association is
// calibrated, we use the deterministic exploration rule
// below as the software fallback.
// ============================================================
Direction chooseTremauxDirection(JunctionOptions junction)
 {
    /*
     * Deterministic exploration priority:
     *
     * LEFT
     * STRAIGHT
     * RIGHT
     *
     * If no unexplored option is available,
     * backtrack.
     *
     * The actual edge-level graph should eventually
     * replace this decision layer once physical
     * junction identity is established.
     */
    static bool leftUsed = false;
    static bool straightUsed = false;
    static bool rightUsed = false;
    if (junction.left && !leftUsed) {
        leftUsed = true;
        return LEFT;
    }
    if (junction.straight && !straightUsed) {
        straightUsed = true;
        return STRAIGHT;
    }
    if (junction.right && !rightUsed) {
        rightUsed = true;
        return RIGHT;
    }
    return BACK;
}

// ============================================================
// PATH OPTIMIZATION
// ============================================================
int directionToAngle(Direction direction) {
    switch (direction) {
        case LEFT:
            return -90;
        case STRAIGHT:
            return 0;
        case RIGHT:
            return 90;
        case BACK:
            return 180;
        default:
            return 0;
    }
}

Direction angleToDirection(int angle) {
    angle %= 360;
    if (angle > 180) {
        angle -= 360;
    }
    if (angle <= -180) {
        angle += 360;
    }
    if (angle == -90) {
        return LEFT;
    }
    if (angle == 0) {
        return STRAIGHT;
    }
    if (angle == 90) {
        return RIGHT;
    }
    if (angle == 180 ||
        angle == -180) {
        return BACK;
    }
    return NONE;
}

void optimizePath() {
    optimizedPathLength = 0;
    if (rawPathLength == 0) {
        return;
    }
    Direction result[MAX_PATH_LENGTH];
    int resultLength = 0;
    for (int i = 0; i < rawPathLength; i++) {  
        result[resultLength++] = rawPath[i];          
        /*
         * Pattern:
         * A -> BACK -> C
         * can be replaced by
         * net(A + BACK + C)
         */
        if (resultLength >= 3 && result[resultLength - 2] == BACK) {           
            int angle = directionToAngle(result[resultLength - 3]) + directionToAngle(result[resultLength - 2]) + directionToAngle(result[resultLength - 1]);
            Direction simplified = angleToDirection(angle);
            if (simplified != NONE) {
                resultLength -= 3;
                result[resultLength++] = simplified;
            }
        }
    }

    optimizedPathLength = resultLength;
    for (int i = 0; i < optimizedPathLength; i++) {
        optimizedPath[i] = result[i];
    }
}
// ============================================================
// ACTUAL RUN
// ============================================================
void executeStoredPath() {
    if (actualPathIndex >= optimizedPathLength) {       
        return;
    }
    Direction direction = optimizedPath[actualPathIndex];
    actualPathIndex++;
    executeDirection(direction);
}

// ============================================================
// CHECKPOINT HOOK
// ============================================================
// The competition document specifies checkpoint
// scoring but does not provide an electronic
// checkpoint-detection mechanism.
// Keep this separate until the physical arena
// markings are known.
// ============================================================
void checkCheckpoint() {
    // Intentionally empty.
    // Implement only after the physical
    // checkpoint markings are known.
}

// ============================================================
// DEBUG FUNCTIONS
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

void printRawPath() {
    Serial.println("RAW PATH:");
    for (int i = 0; i < rawPathLength; i++) {      
        Serial.print(directionToChar(rawPath[i]));
        Serial.print(' ');
    }
    Serial.println();
}

void printOptimizedPath() {
    Serial.println("OPTIMIZED PATH:");
    for (int i = 0; i < optimizedPathLength; i++) {   
        Serial.print(directionToChar(optimizedPath[i]));
        Serial.print(' ');
    }
    Serial.println();
}

// ============================================================
// GRAPH DEBUG
// ============================================================
void printMazeGraph() {
    maze.printGraph();
}

// ============================================================
// DRY RUN
// ============================================================
void dryRun() {
    state = FOLLOWING_LINE;
    while (state != FINISHED) {
        // ---------------- GOAL ----------------
        if (goalDetected()) {
            finishRobot();
            break;
        }
        // ---------------- CHECKPOINT ----------------
        checkCheckpoint();
        // ---------------- LINE FOLLOWING ----------------
        if (!isPotentialJunction()) {
            followLinePID();
            continue;
        }
        // ---------------- JUNCTION ----------------
        state = AT_JUNCTION;
        JunctionOptions junction = classifyJunction();           

        // ---------------- DECISION ----------------
        Direction next = chooseTremauxDirection(junction);

        // ---------------- RECORD ----------------
        recordMove(next);

        // ---------------- EXECUTE ----------------
        executeDirection(next);
    }

    // Goal reached.
    // Generate the shortest equivalent route
    // from the recorded exploration path.
    optimizePath();
    printRawPath();
    printOptimizedPath();
}

// ============================================================
// ACTUAL RUN
// ============================================================
void actualRun() {
    state = FOLLOWING_LINE;
    actualPathIndex = 0;
    while (state != FINISHED) {
        // ---------------- GOAL ----------------
        if (goalDetected()) {
            finishRobot();
            break;
        }
        // ---------------- LINE FOLLOWING ----------------
        if (!isPotentialJunction()) {
            followLinePID();
            continue;
        }
        // ---------------- STORED ROUTE ----------------
        state = AT_JUNCTION;
        executeStoredPath();
    }
}

// ============================================================
// SETUP
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
    // ---------------- INITIAL STATE ----------------
    stopMotors();
    resetPID();
    state = STARTUP;
    Serial.println("MESHMERIZE ROBOT READY");     
    delay(1000);
}

// ============================================================
// LOOP
// ============================================================
void loop() {
#if RUN_MODE == DRY_RUN
    dryRun();
#elif RUN_MODE == ACTUAL_RUN
    actualRun();
#endif
    // Prevent accidental restart after the run has finished.
    stopMotors();
    while (true) {
        delay(1000);
    }
}
