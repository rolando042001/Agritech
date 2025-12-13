#define RELAY_PIN 2 

void setup() {
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH);

  Serial.begin(9600);
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();

    if (cmd == 'R') {
      digitalWrite(RELAY_PIN, LOW);
      Serial.println("Relay ON");
    }
    else if (cmd == 'S') {
      digitalWrite(RELAY_PIN, HIGH);
      Serial.println("Relay OFF");
    }
  }
}
