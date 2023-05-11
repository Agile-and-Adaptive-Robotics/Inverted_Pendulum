#define RIGHT_BUTTON_PIN 3 // ** change these **
#define LEFT_BUTTON_PIN 2

#define RIGHT_VALVE_PIN 9
#define LEFT_VALVE_PIN 10

#define PRESSURE_SENSOR_PIN A7 // analog pin

const float MIN_PRESSURE = 0; // ** change these with minimum and maximum pressures that the pressure sensor returns **
const float MAX_PRESSURE = 1023;





void setup() {
  Serial.begin(9600);
  
  pinMode(RIGHT_BUTTON_PIN, INPUT);
  pinMode(LEFT_BUTTON_PIN, INPUT);

  pinMode(RIGHT_VALVE_PIN, OUTPUT);
  pinMode(LEFT_VALVE_PIN, OUTPUT);

}

bool inRange(float x, float a, float b) { // returns if x is in between a and b
  if (a < b) {
    if (a <= x && x <= b) return true;
  } else {
    if (b <= x && x <= a) return true;
  }
  return false;
}

void loop() {
  static float targetPressure;

  // ------------- Button inputs -------------- //

  float moveSpeed = 8; // ** increase this if it's inflating too slow ** (derivative of target pressure kinda)

  if (digitalRead(RIGHT_BUTTON_PIN)) { // increase target pressure
    targetPressure += moveSpeed;
  }
  
  if (digitalRead(LEFT_BUTTON_PIN)) { // decrease target pressure
    targetPressure -= moveSpeed;
  }

  if (targetPressure < MIN_PRESSURE) targetPressure = MIN_PRESSURE;
  if (targetPressure > MAX_PRESSURE) targetPressure = MAX_PRESSURE;

  // ------------- Valve control -------------- //
  
  int currentPressure = analogRead(PRESSURE_SENSOR_PIN);

  float allowedRange = 15; // acceptable range/2 for the pressure
  if (inRange(currentPressure, targetPressure - allowedRange, targetPressure + allowedRange)) { // stop
    digitalWrite(RIGHT_VALVE_PIN, HIGH);
    digitalWrite(LEFT_VALVE_PIN, LOW);
  }

  else if (currentPressure < targetPressure) { // inflate
    digitalWrite(RIGHT_VALVE_PIN, HIGH);
    digitalWrite(LEFT_VALVE_PIN, HIGH);
  }

  else if (currentPressure > targetPressure) { // deflate
    digitalWrite(RIGHT_VALVE_PIN, LOW);
    digitalWrite(LEFT_VALVE_PIN, LOW);
  }

  
  // ------------- Debugging -------------- //

  Serial.print("target pressure: "); Serial.print(targetPressure);// these should also work with the Serial Plotter I think 
  Serial.print("\t");
  Serial.print("current pressure: "); Serial.println(currentPressure);
  
  // ------------- Wait till next check (can delete if you want but it might cause inconsistencies when messing with other parts of code) -------------- //

  delay(0);

}

// right on , left off : hold pressure
// right on , left on : increase pressure
// right off , left off: increase
