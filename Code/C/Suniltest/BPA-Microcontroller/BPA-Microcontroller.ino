
#define TARGET_PRESSURE_PIN 1 // gets signal representing target pressure
#define CURRENT_PRESSURE_PIN 4 // gets signal representing current pressure of the BPA
#define VALVE_PIN 2 // this pin sends pwm signal to valve for how open/closed it should be


#define VALVE_EQUALIBRIUM .5 // pressure stays the same when valve is open at this value



void setup() {
  Serial.begin(9600);
  while(!Serial);
  //Serial.println("BPA microcontroller test");


  

}

void loop() {
  static float targetPressure;
  static float lastTargetPressure = 0;
  if (digitalRead(TARGET_PRESSURE_PIN)) {
    lastTargetPressure = targetPressure;
    targetPressure = analogRead(TARGET_PRESSURE_PIN);
  }

  static float currentPressure = 0;

  static float valveOpen = .5; // 0 = closed, 1 = open, .5 = halway closed/open



  if (currentPressure < targetPressure) {
    valveOpen += .1;
  }

  else if (currentPressure > targetPressure) {
    valveOpen -= .1;
  }

  else {
    valveOpen = VALVE_EQUALIBRIUM;
  }


  if (valveOpen > 1) valveOpen = 1;
  if (valveOpen < 0) valveOpen = 0;

  // ------------- // set valve open amount to valve

   analogWrite(VALVE_PIN, valveOpen * 255);

   Serial.print("current: "); Serial.print(currentPressure); Serial.print(" target: "); Serial.println(targetPressure);

  // -------------- // simulation

  currentPressure += (valveOpen - VALVE_EQUALIBRIUM) * 3;

  if (random(100) < 2) {
    targetPressure = random(200) - 100;
  }


  

}
