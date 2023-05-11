
#define BUTTON_1_PIN 5
#define BUTTON_2_PIN 4

#define LEFT_VALVE_PIN 3
#define RIGHT_VALVE_PIN 2


void setup() {
  Serial.begin(9600);

  pinMode(BUTTON_1_PIN, INPUT);
  pinMode(BUTTON_2_PIN, INPUT);
  
  pinMode(LEFT_VALVE_PIN, OUTPUT);
  pinMode(RIGHT_VALVE_PIN, OUTPUT);


  


}

void loop() {

  static float leftValveOpen = .5;
  static float rightValveOpen = .5;

  float moveSpeed = .05;

  


  if (digitalRead(BUTTON_1_PIN)) {
    Serial.println("button 1");

    leftValveOpen -= moveSpeed;
    rightValveOpen += moveSpeed;
  }
  if (digitalRead(BUTTON_2_PIN)) {
    Serial.println("button 2");

    leftValveOpen += moveSpeed;
    rightValveOpen -= moveSpeed;
  }

  if (leftValveOpen < 0) leftValveOpen = 0;
  if (leftValveOpen > 1) leftValveOpen = 1;
  if (rightValveOpen < 0) rightValveOpen = 0;
  if (rightValveOpen > 1) rightValveOpen = 1;



  bool leftOn = leftCalculateOn(leftValveOpen);
  bool rightOn = rightCalculateOn(rightValveOpen);

  Serial.println(leftOn);


  //digitalWrite(LEFT_VALVE_PIN, HIGH);
  //digitalWrite(RIGHT_VALVE_PIN, HIGH);
  

}
