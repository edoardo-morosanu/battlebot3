// CONSTANTS
#define DELAY_HIGH_BTN    5
#define DELAY_NORMAL_BTN  4
#define DELAY_LOW_BTN     3
#define RED_LIGHT         6


// GLOBALS
int _blink_delay;

void setup() {
  // INPUTS
  pinMode(DELAY_LOW_BTN, INPUT);
  pinMode(DELAY_NORMAL_BTN, INPUT);
  pinMode(DELAY_HIGH_BTN, INPUT);
  // OUTPUTS
  pinMode(RED_LIGHT, OUTPUT);
}

void loop() {
  /// BLINK ///
  digitalWrite(RED_LIGHT, HIGH);
  delay(_blink_delay);
  digitalWrite(RED_LIGHT, LOW);
  delay(_blink_delay);
  if (!digitalRead(DELAY_LOW_BTN)) // BUTTON 3 PRESSED
  {
    _blink_delay = 250;
  }
  if (!digitalRead(DELAY_NORMAL_BTN)) // BUTTON 2 PRESSED
  {
      _blink_delay = 500;
  }
  if (!digitalRead(DELAY_HIGH_BTN)) // BUTTON 1 PRESSED
  {
      _blink_delay = 1000;
  }
}