void setup() {
  Serial.begin(9600);
  while(!Serial);

  

}

void loop() {
  // put your main code here, to run repeatedly:
  int levels = 12;

  static int currentLevel = 6;

  if (currentLevel < 0) currentLevel = 0;
  if (currentLevel > levels) currentLevel = levels;

  int onCounter = levels;
  int offCounter = levels - currentLevel;

  static int counter = 0;
  counter++;

  static bool on = false;

  if (on) {
    if (counter == onCounter) {
      counter = 0;
      on = false;
    }
  }

  else {
    if (counter == offCounter) {
      counter = 0;
      on = true;
    }
  }


  //Serial.println(on);

  static int lastTime = 0;
  Serial.println(millis() - lastTime);
  lastTime = millis();

}
