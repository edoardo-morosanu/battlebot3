///VARIABLE DEFINITIONS///
#define BUTTON  5
#define RED     6
#define YELLOW  7
#define GREEN   8

void setup() {
  ///PIN MODES DEFINITIONS///
  pinMode(BUTTON, INPUT); // button mapped as input
  pinMode(RED, OUTPUT); // led mapped as output
  pinMode(YELLOW, OUTPUT); // led mapped as output
  pinMode(GREEN, OUTPUT); // led mapped as output
  ///TURN OFF THE LEDS///
  digitalWrite(RED, HIGH);
  digitalWrite(YELLOW, HIGH);
  digitalWrite(GREEN, HIGH);
}

void loop() {
  digitalWrite(RED, LOW);
  if (!digitalRead(BUTTON)) { // if button pressed
    delay(3000);
    digitalWrite(RED, HIGH); // turn off RED
    digitalWrite(GREEN, LOW); // turn on GREEN
    delay(4000);
    digitalWrite(GREEN, HIGH); // turn off GREEN
    digitalWrite(YELLOW, LOW); // turn on YELLOW
    delay(1000);
    digitalWrite(YELLOW, HIGH); // turn off YELLOW
  }

}
