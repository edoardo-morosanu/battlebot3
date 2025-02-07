#define TRIG_PIN   2 // TRIG
#define ECHO_PIN   3 // ECHO
#define MOTORA_PIN1 5  // Motor A control pin 1 (PWM)
#define MOTORA_PIN2 6  // Motor A control pin 2 (PWM)
#define MOTORB_PIN1 11  // Motor B control pin 1 (PWM)
#define MOTORB_PIN2 10 // Motor B control pin 2 (PWM)


// Speed control (0-255)
const int _motorSpeed = 255;  // Maximum speed
const int _turnSpeed = 200;   // Speed during turns
const int _timeToTurnRight = 1000;

float _duration_us, _distance_cm;
const float _constant = 343 * 100 * (1/2) * (1/1000000);

void setup() {
  // Set motor control pins as outputs
  pinMode(MOTORA_PIN1, OUTPUT);
  pinMode(MOTORA_PIN2, OUTPUT);
  pinMode(MOTORB_PIN1, OUTPUT);
  pinMode(MOTORB_PIN2, OUTPUT);

  // Configure the trigger pin to output mode
  pinMode(TRIG_PIN, OUTPUT);
  // Configure the echo pin to input mode
  pinMode(ECHO_PIN, INPUT);
}

void forward() {
  analogWrite(MOTORA_PIN1, _motorSpeed);
  analogWrite(MOTORA_PIN2, 0);
  analogWrite(MOTORB_PIN1, _motorSpeed);
  analogWrite(MOTORB_PIN2, 0);
}

void turnRight() {
  analogWrite(MOTORA_PIN1, _turnSpeed);
  analogWrite(MOTORA_PIN2, 0);
  analogWrite(MOTORB_PIN1, 0);
  analogWrite(MOTORB_PIN2, _turnSpeed);
}

void stop() {
  analogWrite(MOTORA_PIN1, 0);
  analogWrite(MOTORA_PIN2, 0);
  analogWrite(MOTORB_PIN1, 0);
  analogWrite(MOTORB_PIN2, 0);
}

void loop() {
  // send the pulse using the trigger
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  forward();

  // Measure the pulse duration from the ECHO pin
  _duration_us = pulseIn(ECHO_PIN, HIGH);

  // calculate the distance
  _distance_cm = _constant * _duration_us;

  if (_distance_cm <= 15) {
    turnRight();
    delay(_timeToTurnRight);
  }
}