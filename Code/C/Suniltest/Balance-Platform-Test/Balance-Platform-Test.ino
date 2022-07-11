#define S1 52
#define S2 4




void setup() {
  Serial.begin(9600);

}

void loop() {
  if (digitalRead(3)) {
    Serial.println("move right");

    analogWrite(S1, HIGH);
    digitalWrite(S2, HIGH);
  }

  
  if (digitalRead(18)) {
    Serial.println("move left");

    analogWrite(S1, HIGH);
    digitalWrite(S2, LOW);
  }

  if (!digitalRead(3) && !digitalRead(18)) {
    Serial.println("stop");

    analogWrite(S1, LOW);
  }



  //digitalWrite(S1, LOW);
  //digitalWrite(S2, HIGH);

  //delay(1000);

  //digitalWrite(S2, LOW);

  //delay(1000);
  
}
