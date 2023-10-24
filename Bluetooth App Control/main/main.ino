char Incoming_value = 0;  // Value read for serial input RX
const int blueLED = 9;    // Blue LED connected on port 9

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);      // Set Baud rate
  pinMode(blueLED, OUTPUT);  // Declare blueled pin as output
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available() > 0) {
    Incoming_value = Serial.read();
    Serial.print(Incoming_value);
    Serial.print("\n");
  }

  if (Incoming_value == '1') {
    digitalWrite(blueLED, HIGH);
  }

  else if (Incoming_value == '0') {
    digitalWrite(blueLED, LOW);
  }
}
