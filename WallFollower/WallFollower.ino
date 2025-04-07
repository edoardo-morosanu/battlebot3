#include <Adafruit_NeoPixel.h>

#define TRIG_FRONT 4
#define ECHO_FRONT 7
#define TRIG_LEFT 8
#define ECHO_LEFT 13
#define TRIG_RIGHT 11
#define ECHO_RIGHT 12

#define MOTOR_A_PIN1 10
#define MOTOR_A_PIN2 9
#define MOTOR_B_PIN1 6
#define MOTOR_B_PIN2 5
#define ROTARY_RIGHT A0

// --- Gripper and Line Sensor Definitions --- //
#define GRIPPER_PIN 3   // GPIO pin for servo/gripper control
#define TIME_OPEN 1550  // Pulse width for open position (microseconds)
#define TIME_CLOSE 975  // Pulse width for closed position (microseconds)

// --- Light Definitions --- //
#define NEOPIXEL_PIN 2
#define NUM_PIXELS 4
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUM_PIXELS, NEOPIXEL_PIN, NEO_RGB + NEO_KHZ800);
const int LIGHT_UPDATE_DELAY = 50;

#define LINE_SENSOR_PIN A4   // Analog pin for line sensor
#define BLACK_THRESHOLD 800  // Threshold value for black detection (adjust as needed)

// --- Robot Constants --- //
const float WHEEL_DIAMETER = 6.7;  // in cm
const float WHEELBASE = 14.5;      // in cm

// --- Wall Following Constants --- //
const int BASE_SPEED = 255;  // Reduced from 255 for better control
const int MIN_SPEED = 130;   // Minimum speed during corrections
const int MAX_SPEED = 255;   // Maximum speed

// --- Distance Constants --- //
const int FRONT_OBSTACLE_THRESHOLD = 19;  // cm for front obstacle detection
const int IDEAL_WALL_DISTANCE = 12;       // Target distance from right wall in cm
const int WALL_DISTANCE_TOLERANCE = 2;    // Acceptable variance in cm
const int RIGHT_TOO_CLOSE = IDEAL_WALL_DISTANCE - WALL_DISTANCE_TOLERANCE;
const int RIGHT_TOO_FAR = IDEAL_WALL_DISTANCE + WALL_DISTANCE_TOLERANCE;
const int NO_WALL_THRESHOLD = 17;  // Distance indicating no wall present

// --- PID Control Constants --- //
const float KP = 4.0;  // Proportional gain (adjust based on testing)
const float KI = 0.0;  // Integral gain (start with 0)
const float KD = 1.5;  // Derivative gain (adjust based on testing)

// --- Runtime Variables --- //
float previousError = 0;          // For PID control
float integral = 0;               // For PID control
bool gripperClosed = false;       // To track gripper state
unsigned long raceStartTime = 0;  // To track when race started
bool squareCheckEnabled = false;  // Only start checking for squares after delay
unsigned long lastMoveTime = 0;
int lastRotaryValue = -1;


// --- Robot State Management --- //
enum RobotState {
  RACE_NOT_STARTED,
  LINE_FOLLOWING,
  WALL_FOLLOWING
};
RobotState currentState = RACE_NOT_STARTED;
int lineFollowCounter = 0;  // Counter to track line following duration

// --- Timing for maneuvers --- //
const int TURN90_DELAY = 400;         // milliseconds to execute a 90° turn
const int BACKWARD_DELAY = 600;       // milliseconds for backing up
const int FORWARD_DELAY = 200;        // shorter clearance after turns
const int SQUARE_CHECK_DELAY = 5000;  // 5 seconds before checking for squares
const int LINE_FOLLOW_DURATION = 50;  // Number of loops to follow line before checking for transition

// --- NeoPixel Function --- //
void setLights(int startIndex, int endIndex, int r, int g, int b) {
  for(int i = startIndex; i <= endIndex; i++) {
    // Ensure index is within bounds
    if (i >= 0 && i < NUM_PIXELS) {
        pixels.setPixelColor(i, pixels.Color(r, g, b));
    }
  }
  pixels.show();
}

// --- Improved Gripper Control Function --- //
void controlGripper(bool open) {
  Serial.print("Setting gripper to ");
  Serial.println(open ? "OPEN" : "CLOSED");
  setLights(0, NUM_PIXELS - 1, 255, 255, 255);

  // Send multiple pulses to ensure the servo responds
  for (int i = 0; i < 5; i++) {
    digitalWrite(GRIPPER_PIN, HIGH);
    if (open) {
      delayMicroseconds(TIME_OPEN);
    } else {
      delayMicroseconds(TIME_CLOSE);
    }
    digitalWrite(GRIPPER_PIN, LOW);
    delay(20);  // Small delay between pulses
  }

  gripperClosed = !open;

  // Give the servo time to move
  delay(300);
}

// --- Line Sensor Function --- //
bool detectBlack(int sensorPin) {
  int sensorValue = analogRead(sensorPin);
  Serial.print("Line Sensor: ");
  Serial.println(sensorValue);

  return (sensorValue > BLACK_THRESHOLD);
}

// --- Calibration Function --- //
void calibrateSensors() {
  Serial.println("Calibrating sensors...");
  setLights(0, NUM_PIXELS - 1, 255, 255, 0);

  // Simple calibration - just take a few readings to stabilize the sensors
  for (int i = 0; i < 5; i++) {
    getDistance(TRIG_FRONT, ECHO_FRONT);
    getDistance(TRIG_LEFT, ECHO_LEFT);
    getDistance(TRIG_RIGHT, ECHO_RIGHT);
    delay(100);
  }

  Serial.println("Calibration complete");
}

// --- Check for Black Square --- //
void checkForSquare() {
  // Only check after the specified delay
  if (!squareCheckEnabled && (millis() - raceStartTime > SQUARE_CHECK_DELAY)) {
    squareCheckEnabled = true;
    Serial.println("Square detection enabled");
  }

  if (squareCheckEnabled && detectBlack(LINE_SENSOR_PIN)) {
    Serial.println("BLACK SQUARE DETECTED!");

    setLights(0, NUM_PIXELS - 1, 255, 0, 255);

    // Stop robot
    stopRobot();
    delay(200);

    // Open gripper
    controlGripper(true);

    // Back up a bit
    backward(BASE_SPEED, BASE_SPEED);
    delay(BACKWARD_DELAY);
    stopRobot();

    // Wait for a while at the square
    Serial.println("Task complete. Robot stopped.");
    delay(5000);
  }
}

// --- Ultrasonic Distance Measurement --- //
float getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  float durationUs = pulseIn(echoPin, HIGH, 30000);
  if (durationUs == 0) {
    return 999;  // error value
  }

  // Apply simple noise filtering
  float distance = 0.017 * durationUs;  // convert to cm

  // Clamp maximum reading
  if (distance > 150) {
    distance = 150;
  }

  return distance;
}

// --- Motor Control Functions --- //
void forward(int leftSpeed, int rightSpeed) {
  // Ensure speeds are within bounds
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);
  setLights(0, NUM_PIXELS, 0, 255, 255);

  // Drive forward: left motor forward, right motor forward
  analogWrite(MOTOR_A_PIN1, leftSpeed);
  analogWrite(MOTOR_A_PIN2, 0);
  analogWrite(MOTOR_B_PIN1, rightSpeed);
  analogWrite(MOTOR_B_PIN2, 0);

  Serial.print("Forward: L=");
  Serial.print(leftSpeed);
  Serial.print(", R=");
  Serial.println(rightSpeed);
}

void backward(int leftSpeed, int rightSpeed) {
  controlGripper(false);
  // Ensure speeds are within bounds
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  // Drive backward: left motor backward, right motor backward
  analogWrite(MOTOR_A_PIN1, 0);
  analogWrite(MOTOR_A_PIN2, leftSpeed);
  analogWrite(MOTOR_B_PIN1, 0);
  analogWrite(MOTOR_B_PIN2, rightSpeed);

  Serial.print("Backward: L=");
  Serial.print(leftSpeed);
  Serial.print(", R=");
  Serial.println(rightSpeed);
}

void stopRobot() {
  analogWrite(MOTOR_A_PIN1, 0);
  analogWrite(MOTOR_A_PIN2, 0);
  analogWrite(MOTOR_B_PIN1, 0);
  analogWrite(MOTOR_B_PIN2, 0);
  Serial.println("Stop");
}

// --- Turning Functions --- //
void turn90Right() {
  controlGripper(false);
  Serial.println("Turning 90° right");
  setLights(0, NUM_PIXELS, 0, 0, 255);
  // stopRobot();
  // delay(50);

  int phaseFull = TURN90_DELAY * 0.7;
  int phaseSlow = TURN90_DELAY - phaseFull;

  // Phase 1: Full-speed left wheel, reduced speed right wheel
  forward(BASE_SPEED, BASE_SPEED / 4);
  delay(phaseFull);

  // Phase 2: Slow down as turn completes
  forward(BASE_SPEED / 2, BASE_SPEED / 3);
  delay(phaseSlow);

  // stopRobot();
  // delay(50);
}

void turn90Left() {
  controlGripper(false);
  Serial.println("Turning 90° left");
  setLights(0, NUM_PIXELS, 0, 0, 255);
  // stopRobot();
  // delay(50);

  int phaseFull = TURN90_DELAY * 0.8;
  int phaseSlow = TURN90_DELAY - phaseFull;

  // Phase 1: Both wheels move but left wheel slower for smoother turn
  forward(0, BASE_SPEED);
  delay(phaseFull);

  // Phase 2: Slow down as turn completes
  forward(0, BASE_SPEED / 2);
  delay(phaseSlow);

  // stopRobot();
  // delay(50);
}

void turnLeftStart() {
  Serial.println("Turning 90° left");
  setLights(0, NUM_PIXELS-1, 255, 165, 0);
  stopRobot();
  delay(50);
  bool onLine = detectBlack(LINE_SENSOR_PIN);
  int phaseFull = TURN90_DELAY + 200;
  int phaseSlow = TURN90_DELAY + 100;

  // Phase 1:  Full-speed right pivot
  forward(0, BASE_SPEED);
  delay(phaseFull);

  // // Phase 2: Slow finish
  forward(0, BASE_SPEED / 2);
  delay(phaseSlow);

  // stopRobot();
  // delay(50);
}

void turn180() {
  controlGripper(false);
  Serial.println("Executing 180° turn");
  setLights(0, NUM_PIXELS-1, 255, 0, 0);
  stopRobot();
  delay(50);

  // Step 1: Back up to clear the wall
  backward(BASE_SPEED / 4, BASE_SPEED);
  delay(BACKWARD_DELAY);
  stopRobot();
  delay(50);

  // Step 2: Execute a pivot in place for a 180° turn
  int totalTime = TURN90_DELAY * 2;
  int phaseFull = totalTime * 0.7 / 2;
  int phaseSlow = totalTime - phaseFull;

  // Full-speed pivot
  analogWrite(MOTOR_A_PIN1, 0);
  analogWrite(MOTOR_A_PIN2, 0);
  analogWrite(MOTOR_B_PIN1, BASE_SPEED);
  analogWrite(MOTOR_B_PIN2, BASE_SPEED);
  delay(phaseFull);

  // Slow finish pivot
  analogWrite(MOTOR_A_PIN1, 0);
  analogWrite(MOTOR_A_PIN2, 0);
  analogWrite(MOTOR_B_PIN1, BASE_SPEED / 2);
  analogWrite(MOTOR_B_PIN2, BASE_SPEED / 2);
  delay(phaseSlow);

  backward(BASE_SPEED, BASE_SPEED);
  delay(500);

  stopRobot();
  delay(50);
}

// --- New Line Following Function --- //
void followLine() {
  Serial.println("Following line - turning left");
  forward(BASE_SPEED, BASE_SPEED);
  delay(800);
  currentState = WALL_FOLLOWING;
}

// --- Wall Following Function --- //
void followRightWall(float frontDistance, float rightDistance) {
  controlGripper(false);
  // First, check for black square
  checkForSquare();

  // Calculate error: positive when too far from wall, negative when too close
  float error = IDEAL_WALL_DISTANCE - rightDistance;

  // PID calculation
  integral = integral + error;
  integral = constrain(integral, -100, 100);  // Anti-windup
  float derivative = error - previousError;

  // Calculate correction using PID formula
  float correction = (KP * error) + (KI * integral) + (KD * derivative);
  previousError = error;

  // If no wall on right (or very far), make a gentle right turn to find wall
  if (rightDistance > NO_WALL_THRESHOLD) {
    Serial.println("No right wall detected, turning gently right");
    // Left wheel at high speed, right at lower speed for a gradual turn
    forward(BASE_SPEED, BASE_SPEED / 3);
    delay(100);  // Short duration to prevent over-turning
    return;
  }

  // If obstacle ahead, handle accordingly
  if (frontDistance < FRONT_OBSTACLE_THRESHOLD) {
    Serial.println("Front obstacle detected, turning left");
    turn90Left();
    return;
  }

  // Normal wall following with correction
  int leftSpeed, rightSpeed;

  // If too close to the wall, make a more aggressive correction
  if (rightDistance < RIGHT_TOO_CLOSE) {
    Serial.println("Too close to wall, correcting left");
    // Slow down left wheel, maintain right wheel to turn away from wall
    leftSpeed = BASE_SPEED * 0.5;  // Reduce left wheel speed to 60%
    rightSpeed = BASE_SPEED;
  }
  // If too far from the wall, make a correction to the right
  else if (rightDistance > RIGHT_TOO_FAR) {
    Serial.println("Too far from wall, correcting right");
    // Maintain left wheel, slow down right wheel to turn toward wall
    leftSpeed = BASE_SPEED;
    rightSpeed = BASE_SPEED * 0.5;  // Reduce right wheel speed to 60%
  }
  // Within acceptable distance range, apply normal PID correction
  else {
    // Standard PID control (reversed from original so + correction turns left)
    leftSpeed = BASE_SPEED - correction;
    rightSpeed = BASE_SPEED + correction;
  }

  // Ensure minimum speeds to prevent stalling
  leftSpeed = constrain(leftSpeed, MIN_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, MIN_SPEED, MAX_SPEED);

  // Move forward with adjusted speeds
  forward(leftSpeed, rightSpeed);
}

// --- Start Race Sequence --- //
void startRace() {
  float frontDistance = getDistance(TRIG_FRONT, ECHO_FRONT);
  
  while (frontDistance > 25) {
    frontDistance = getDistance(TRIG_FRONT, ECHO_FRONT);
    delay(50);
  }
  while (frontDistance < 25) {
    frontDistance = getDistance(TRIG_FRONT, ECHO_FRONT);
    delay(50);
  }
  Serial.println("Starting race sequence...");

  // 1. Move forward before calibration
  Serial.println("Moving forward...");
  forward(BASE_SPEED, BASE_SPEED);
  delay(600);
  forward(BASE_SPEED / 2, BASE_SPEED / 2);
  delay(400);
  stopRobot();
  delay(200);

  // 2. Calibrate sensors
  calibrateSensors();
  delay(200);

  // 3. Close gripper with improved function
  Serial.println("Closing gripper...");
  controlGripper(false);

  // 4. Turn left to follow the line
  Serial.println("Turning left to find line...");
  turnLeftStart();

  // 5. Ready to go - mark race as started and record time
  currentState = LINE_FOLLOWING;
  raceStartTime = millis();
  Serial.println("Race started! Following line...");
}

// --- Main Loop --- //
void loop() {
  // State machine to control robot behavior
  switch (currentState) {
    case RACE_NOT_STARTED:
      startRace();
      break;

    case LINE_FOLLOWING:
      followLine();
      break;

    case WALL_FOLLOWING:
      // Get sensor readings
      float frontDistance = getDistance(TRIG_FRONT, ECHO_FRONT);
      float leftDistance = getDistance(TRIG_LEFT, ECHO_LEFT);
      float rightDistance = getDistance(TRIG_RIGHT, ECHO_RIGHT);

      int value = analogRead(ROTARY_RIGHT);
      Serial.println(value);
      if (value != lastRotaryValue) {
        lastMoveTime = millis();
        lastRotaryValue = value;
      }

      if (millis() - lastMoveTime > 1000) {
        backward(BASE_SPEED, BASE_SPEED);
        delay(1000);
        lastMoveTime = millis();
      }

      delay(100);

      // Skip cycle if any reading is invalid
      if (frontDistance == 999 || leftDistance == 999 || rightDistance == 999) {
        Serial.println("Invalid sensor reading, retrying");
        delay(50);
        return;
      }

      // Log sensor values
      Serial.print("Front: ");
      Serial.print(frontDistance);
      Serial.print(" cm, Left: ");
      Serial.print(leftDistance);
      Serial.print(" cm, Right: ");
      Serial.print(rightDistance);
      Serial.println(" cm");

      // if (frontDistance < FRONT_OBSTACLE_THRESHOLD && leftDistance < NO_WALL_THRESHOLD && rightDistance < NO_WALL_THRESHOLD) {
      //   Serial.println("Dead-end detected, turning 180°");
      //   turn180();
      //   return;
      // }

      // If an obstacle is ahead
      if (frontDistance < FRONT_OBSTACLE_THRESHOLD) {
        Serial.println("Obstacle ahead!");

        // If right side is clear, turn right
        if (rightDistance > NO_WALL_THRESHOLD) {
          turn90Right();
        }
        // Otherwise, turn left
        else if (leftDistance > NO_WALL_THRESHOLD) {
          turn90Left();
        }
      }

      if (frontDistance < FRONT_OBSTACLE_THRESHOLD && leftDistance < NO_WALL_THRESHOLD && rightDistance < NO_WALL_THRESHOLD) {
        Serial.println("Dead-end detected, turning 180°");
        turn180();
        return;
      }

      // If there is no wall on the right, turn right
      if (rightDistance > NO_WALL_THRESHOLD) {
        Serial.println("Open space on the right, turning right");
        turn90Right();
        return;
      }

      // Right wall following algorithm
      followRightWall(frontDistance, rightDistance);
      break;
  }

  // Small delay to allow sensor and motor updates
  delay(50);
}

// --- Setup --- //
void setup() {
  // Initialize motor pins
  pinMode(MOTOR_A_PIN1, OUTPUT);
  pinMode(MOTOR_A_PIN2, OUTPUT);
  pinMode(MOTOR_B_PIN1, OUTPUT);
  pinMode(MOTOR_B_PIN2, OUTPUT);

  // Initialize sensor pins
  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);

  // Initialize gripper pin
  pinMode(GRIPPER_PIN, OUTPUT);
  digitalWrite(GRIPPER_PIN, LOW);


  // Initialize line sensor pin
  pinMode(LINE_SENSOR_PIN, INPUT);

  // Initialize rotary sensor pin
  pinMode(ROTARY_RIGHT, INPUT);

  pixels.begin();
  pixels.clear();
  pixels.show();

  // Initialize serial communication
  Serial.begin(9600);
  Serial.println("Right Wall Following Robot with Gripper - Setup");

  // Stabilize sensors and allow time for placement
  delay(1000);

  // Reset PID variables
  previousError = 0;
  integral = 0;

  // Leave gripper open by default
  controlGripper(true);

  Serial.println("Robot ready to start");
}
