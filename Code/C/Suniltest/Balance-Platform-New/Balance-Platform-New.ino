
#define PIN_1 2
#define PIN_2 3
#define PIN_3 4
#define PIN_4 5










void setup() {
  Serial.begin(9600);
  
  pinMode(PIN_1, OUTPUT);
  pinMode(PIN_2, OUTPUT);
  pinMode(PIN_3, OUTPUT);
  pinMode(PIN_4, OUTPUT);


  

}


void setBackward() {
  
  digitalWrite(PIN_1, LOW);
  digitalWrite(PIN_2, HIGH);
  digitalWrite(PIN_3, HIGH);
  digitalWrite(PIN_4, LOW);
  
}



void setForward() {
  
  digitalWrite(PIN_1, HIGH);
  digitalWrite(PIN_2, LOW);
  digitalWrite(PIN_3, LOW);
  digitalWrite(PIN_4, HIGH);
  
}





void loop() {

  static float speed = .5; // 0 to 1


  if (calculateOn(speed)) {
    setForward();
    Serial.println("forward");
  }
  else {
    setBackward();
    Serial.println("backward");
  }


  
  delay(10);



}
