int openHandPin = 4;  // Pin for open hand gesture
int peacePin = 5;     // Pin for peace gesture

void setup() {
  Serial.begin(9600);  // Start the serial communication
  pinMode(openHandPin, OUTPUT);
  pinMode(peacePin, OUTPUT);
}

void loop() {

  if (Serial.available() > 0) {
    String gesture = Serial.readStringUntil('\n');  // Read the incoming gesture

    if (gesture == "open_hand") {
      digitalWrite(openHandPin, HIGH);  // Turn on the pin for open hand
      digitalWrite(peacePin, LOW);      // Ensure the peace pin is off
    } 
    else if (gesture == "peace") {
      digitalWrite(peacePin, HIGH);     // Turn on the pin for peace
      digitalWrite(openHandPin, LOW);   // Ensure the open hand pin is off
    } 
    else {
      digitalWrite(openHandPin, LOW);   // Turn off both pins if no gesture or unknown gesture
      digitalWrite(peacePin, LOW);
    }
  } 
  else {
    // If no data is received, turn off both pins
    digitalWrite(openHandPin, LOW);
    digitalWrite(peacePin, LOW);
  }

  delay(100);  // Small delay to avoid rapid toggling, adjust as necessary
}
