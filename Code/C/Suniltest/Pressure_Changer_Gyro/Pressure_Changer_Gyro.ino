// ------------------ // Gyroscope stuff

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_L3GD20_U.h>

Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(20);


// ------------------ // Constants

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

  gyro.enableAutoRange(true);

  if (!gyro.begin(gyro.L3DS20_RANGE_2000DPS)) {
    Serial.println("gyro not working");
  }
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
  int checksPerSecond = 500;
  
  static float targetPressure;

  // ------------- Button inputs -------------- //
  

  float moveSpeed = PI/1000; // ** increase this if it's inflating too slow ** (derivative of target pressure kinda)

  if (digitalRead(RIGHT_BUTTON_PIN)) { // increase target pressure
    targetPressure += moveSpeed;
  }
  
  if (digitalRead(LEFT_BUTTON_PIN)) { // decrease target pressure
    targetPressure -= moveSpeed;
  }

  //if (targetPressure < MIN_PRESSURE) targetPressure = MIN_PRESSURE;
  //if (targetPressure > MAX_PRESSURE) targetPressure = MAX_PRESSURE;


  // ------------- Calculate angle -------------- //

  static int loopCounter = 0;
  loopCounter++;

  static float angle = 0; // angle assumes that it is straight up on start
  float angleVelocity = 0;
  const float targetAngle = PI / 2; // angle straight up (90 degrees)

  float correction = 0.0523;


  sensors_event_t event;
  gyro.getEvent(&event);
  
  if (loopCounter > 10) {
    angleVelocity = event.gyro.z - correction;
  } else {
    angleVelocity = 0;
  }

  static float lastTime = 0;

  float currentTime = millis();

  

  angle -= angleVelocity * ((currentTime - lastTime) / 1000);


  lastTime = currentTime;

  // ------------- Valve control -------------- //
  
  //int currentPressure = analogRead(PRESSURE_SENSOR_PIN);
  float currentPressure = angle;
  

  float allowedRange = 0.015; // acceptable range/2 for the pressure
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
  Serial.print(angle * (180 / PI));
  Serial.print("\t");
  Serial.print("target angle: "); 
  Serial.println(targetPressure * (180 / PI));
  Serial.print("\t");

  int rollAmount = 100;
  static float lastVelocities[100];
  float rollingTotal = 0;

  for (int i = rollAmount - 1; i > 0; i--) {
    lastVelocities[i] = lastVelocities[i-1];
    rollingTotal += lastVelocities[i];
  } lastVelocities[0] = angleVelocity;
    rollingTotal += lastVelocities[0];

  float rollingVelocity = rollingTotal / rollAmount;



  static float oldAverage = .05;
  
  float beta = 0.001;
  float newAverage = oldAverage * (1 - beta) + angleVelocity * (beta);
  
  oldAverage = newAverage;

  
  //Serial.print(angleVelocity * 1000);

  //Serial.print("\t");
  
  //Serial.println((newAverage)*1000);// these should also work with the Serial Plotter I think 
 // Serial.print("\t");
 // Serial.print("current pressure: "); Serial.println(currentPressure);
  
  // ------------- Wait till next check (can delete if you want but it might cause inconsistencies when messing with other parts of code) -------------- //

  delay(0);

}

// right on , left off : hold pressure
// right on , left on : increase pressure
// right off , left off: increase