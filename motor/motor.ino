#include <Servo.h>

// Motor control pins with PWM support
#define MOTORA_PIN1 5  // Motor A control pin 1 (PWM)
#define MOTORA_PIN2 6  // Motor A control pin 2 (PWM)
#define MOTORB_PIN1 11  // Motor B control pin 1 (PWM)
#define MOTORB_PIN2 10 // Motor B control pin 2 (PWM)

// Gripper configuration
#define GRIPPER_PIN 3  // Servo control pin
#define GRIPPER_OPEN_ANGLE 100
#define GRIPPER_CLOSED_ANGLE 40
Servo gripper;

// Speed control (0-255)
const int _motorSpeed = 255;  // Maximum speed
const int _turnSpeed = 200;   // Speed during turns
const int _timeToMoveForward = 2000;
const int _timeToTurnRight = 460;

void setup() {
    // Set motor control pins as outputs
    pinMode(MOTORA_PIN1, OUTPUT);
    pinMode(MOTORA_PIN2, OUTPUT);
    pinMode(MOTORB_PIN1, OUTPUT);
    pinMode(MOTORB_PIN2, OUTPUT);
    
    // Initialize gripper servo
    gripper.attach(GRIPPER_PIN);
    gripper.write(GRIPPER_OPEN_ANGLE);  // Start with open gripper
}

void forward() {
    openGripper();
    closeGripper();
    analogWrite(MOTORA_PIN1, _motorSpeed);
    analogWrite(MOTORA_PIN2, 0);
    analogWrite(MOTORB_PIN1, _motorSpeed);
    analogWrite(MOTORB_PIN2, 0);
}

void openGripper() {
    gripper.write(GRIPPER_OPEN_ANGLE);
    delay(500);  // Give time for servo to move
}

void closeGripper() {
    gripper.write(GRIPPER_CLOSED_ANGLE);
    delay(500);  // Give time for servo to move
}

void turnRight() {
    closeGripper();  // Close gripper before turning
    
    analogWrite(MOTORA_PIN1, _turnSpeed);
    analogWrite(MOTORA_PIN2, 0);
    analogWrite(MOTORB_PIN1, 0);
    analogWrite(MOTORB_PIN2, _turnSpeed);

    openGripper();  // Open gripper after turning
}

void stop() {
    analogWrite(MOTORA_PIN1, 0);
    analogWrite(MOTORA_PIN2, 0);
    analogWrite(MOTORB_PIN1, 0);
    analogWrite(MOTORB_PIN2, 0);
}

void loop() {
    forward();
    delay(_timeToMoveForward);
    turnRight();
    delay(_timeToTurnRight);
    stop();
    delay(_timeToMoveForward);
}