
#define S1 10
#define S2 9

#define LEFT_BUTTON_PIN 3
#define RIGHT_BUTTON_PIN 2

#define POTENTIOMETER_PIN 0 // A0







void setup() {
  Serial.begin(9600);
  
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);

  pinMode(LEFT_BUTTON_PIN,  INPUT);
  pinMode(RIGHT_BUTTON_PIN, INPUT);

  pinMode(POTENTIOMETER_PIN, INPUT);
  

}






void loop() {

  static float speed = .5; // 0 to 1
  static float direction = 1; // 0 = backward, 1 = forward


  if (digitalRead(LEFT_BUTTON_PIN)) {
    speed = .5;
    direction = 0;
  }
  if (digitalRead(RIGHT_BUTTON_PIN)) {
    speed = .45;
    direction = 1;
  }
  
  if (digitalRead(LEFT_BUTTON_PIN) && digitalRead(RIGHT_BUTTON_PIN)) {
    speed = 0;
    //direction = .5;
  }

  if (!digitalRead(LEFT_BUTTON_PIN) && !digitalRead(RIGHT_BUTTON_PIN)) {
    speed = 0;
    //direction = .5;
  }

  int potPosition = analogRead(POTENTIOMETER_PIN);

  if (potPosition < 115) {
    direction = 0;
  }
  
  if (potPosition > 229) {
    direction = 1;
  }

  

  analogWrite(S1, speed * 255);
  analogWrite(S2, direction * 255);

  Serial.print("speed: "); Serial.print(speed);
  Serial.print("\t");
  Serial.print("direction: "); Serial.print(direction);
  Serial.print("\t");
  Serial.print("potentiometer: "); Serial.println(potPosition);

  
  delay(1);



}
