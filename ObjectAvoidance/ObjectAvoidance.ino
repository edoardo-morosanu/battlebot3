#define TRIG_PIN   8 // TRIG
#define ECHO_PIN   9 // ECHO
#define MOTORA_PIN1 5  // Motor A control pin 1 (PWM)
#define MOTORA_PIN2 6  // Motor A control pin 2 (PWM)
#define MOTORB_PIN1 11  // Motor B control pin 1 (PWM)
#define MOTORB_PIN2 10 // Motor B control pin 2 (PWM)
#define R1_PIN 2  // Encoder pin for Motor A
#define R2_PIN 3  // Encoder pin for Motor B

// Speed control and encoder variables
const int _motorSpeed = 255;  // Maximum speed
const int _turnSpeed = 255;   // Speed during turns
const int _timeToTurnRight = 800;
const int CORRECTION_FACTOR = 5; // Adjust this value as needed

volatile int ticksLeft = 0;
volatile int ticksRight = 0;
bool isTurning = false;

void setup() {
  // Motor control pins
  pinMode(MOTORA_PIN1, OUTPUT);
  pinMode(MOTORA_PIN2, OUTPUT);
  pinMode(MOTORB_PIN1, OUTPUT);
  pinMode(MOTORB_PIN2, OUTPUT);

  Serial.begin(9600);
  Serial.println("Robot starting up...");
  
  // Ultrasonic sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  // Encoder pins with interrupt
  pinMode(R1_PIN, INPUT);
  pinMode(R2_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(R1_PIN), tickRight, CHANGE);
  attachInterrupt(digitalPinToInterrupt(R2_PIN), tickLeft, CHANGE);
}

void forward() {
  int leftSpeed = _motorSpeed;
  int rightSpeed = _motorSpeed;
  
  // Only apply corrections when not turning
  if (!isTurning) {
    // Calculate difference between encoders
    int diff = ticksLeft - ticksRight;

    Serial.print("Encoders L/R: ");
    Serial.print(ticksLeft);
    Serial.print("/");
    Serial.print(ticksRight);
    Serial.print(" Diff: ");
    Serial.println(diff);
    
    // Adjust speeds based on difference
    if (diff > 0) {
      leftSpeed = _motorSpeed - (diff * CORRECTION_FACTOR);
      rightSpeed = _motorSpeed;
      Serial.println("Correcting left motor");
    } else if (diff < 0) {
      leftSpeed = _motorSpeed;
      rightSpeed = _motorSpeed + (diff * CORRECTION_FACTOR);
      Serial.println("Correcting right motor");
    }
    
    // Constrain speeds to valid PWM range
    leftSpeed = constrain(leftSpeed, 0, 255);
    rightSpeed = constrain(rightSpeed, 0, 255);
  }
  
  Serial.print("Motor speeds L/R: ");
  Serial.print(leftSpeed);
  Serial.print("/");
  Serial.println(rightSpeed);

  analogWrite(MOTORA_PIN1, leftSpeed);
  analogWrite(MOTORA_PIN2, 0);
  analogWrite(MOTORB_PIN1, rightSpeed);
  analogWrite(MOTORB_PIN2, 0);
}

void turnRight() {
  isTurning = true;  // Set turning state
  Serial.println("Turning right - corrections disabled");
  analogWrite(MOTORA_PIN1, _turnSpeed);
  analogWrite(MOTORA_PIN2, 0);
  analogWrite(MOTORB_PIN1, 0);
  analogWrite(MOTORB_PIN2, _turnSpeed);
}

void stop() {
  isTurning = false;  // Reset turning state
  Serial.println("Stopping - corrections enabled");
  analogWrite(MOTORA_PIN1, 0);
  analogWrite(MOTORA_PIN2, 0);
  analogWrite(MOTORB_PIN1, 0);
  analogWrite(MOTORB_PIN2, 0);
}

void tickLeft() {
  noInterrupts();
  ticksLeft++; 
  interrupts();
}

void tickRight() {
  noInterrupts();
  ticksRight++;
  interrupts();
}

void loop() {
  // Send ultrasonic pulse
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Measure distance
  float durationUs = pulseIn(ECHO_PIN, HIGH);
  float distanceCm = 0.017 * durationUs;
  
  // Move forward and adjust speed based on encoder counts
  forward();
  
  // Obstacle avoidance
  if (distanceCm <= 15 && distanceCm != 0) {
    ticksLeft = 0;
    ticksRight = 0;
    
    turnRight();
    delay(_timeToTurnRight);
    
    stop();
    delay(500);  // Increase delay to allow complete stop
    
    // Reset encoders again after turning
    ticksLeft = 0;
    ticksRight = 0;
  }
}