#define S1 22
#define S2 25




void setup() {
  Serial.begin(9600);

}

void loop() {
  if (digitalRead(3)) {
    Serial.println("move right");

    digitalWrite(S1, HIGH);
    digitalWrite(S2, HIGH);
  }

  
  if (digitalRead(18)) {
    Serial.println("move left");

    digitalWrite(S1, HIGH);
    digitalWrite(S2, LOW);
  }

  if (!digitalRead(3) && !digitalRead(18)) {
    Serial.println("stop");

    digitalWrite(S1, LOW);
    digitalWrite(S2, LOW);
  }



  digitalWrite(S1, LOW);
  digitalWrite(S2, HIGH);

  delay(1000);

  digitalWrite(S2, LOW);

  delay(1000);
  
}
