
#define S1 10
#define S2 9

#define LEFT_BUTTON_PIN 3
#define RIGHT_BUTTON_PIN 2









void setup() {
  Serial.begin(9600);
  
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);

  pinMode(LEFT_BUTTON_PIN,  INPUT);
  pinMode(RIGHT_BUTTON_PIN, INPUT);


  

}






void loop() {

  static float speed = .5; // 0 to 1
  static bool direction = 1; // 0 == backward, 1 == forward


  if (digitalRead(LEFT_BUTTON_PIN)) {
    speed = 1;
    direction = 0;
  }
  if (digitalRead(RIGHT_BUTTON_PIN)) {
    speed = 1;
    direction = 1;
  }
  
  if (digitalRead(LEFT_BUTTON_PIN) && digitalRead(RIGHT_BUTTON_PIN)) {
    speed = 0;
  }

  if (!digitalRead(LEFT_BUTTON_PIN) && !digitalRead(RIGHT_BUTTON_PIN)) {
    speed = 0;
  }

  digitalWrite(S1, speed);
  digitalWrite(S2, direction);

  Serial.print("speed: "); Serial.print(speed);
  Serial.print("\t");
  Serial.print("direction: "); Serial.println(direction);

  
  delay(10);



}
