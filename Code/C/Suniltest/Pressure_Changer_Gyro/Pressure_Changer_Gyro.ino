// ------------------ // Gyroscope stuff

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_L3GD20_U.h>

Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(20);


// ------------------ // Constants

#define RIGHT_BUTTON_PIN 18 // ** change these **
#define LEFT_BUTTON_PIN 19

#define RIGHT_VALVE_PIN 16
#define LEFT_VALVE_PIN 17

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

  if (!gyro.begin()) {
    Serial.println("gyro not working");
    while(1); // stop program
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
  
  static float targetAngle;

  // ------------- Button inputs -------------- //
  

  float moveSpeed = PI/1000; // ** increase this if it's inflating too slow ** (derivative of target pressure kinda)

  if (digitalRead(RIGHT_BUTTON_PIN)) { // increase target pressure
    targetAngle += moveSpeed;
  }
  
  if (digitalRead(LEFT_BUTTON_PIN)) { // decrease target pressure
    targetAngle -= moveSpeed;
  }

  //if (targetAngle < MIN_PRESSURE) targetAngle = MIN_PRESSURE;
  //if (targetAngle > MAX_PRESSURE) targetAngle = MAX_PRESSURE;


  // ------------- Calculate angle -------------- //

  static int loopCounter = 0;
  loopCounter++;

  static float angle = 0; // angle assumes that it is straight up on start
  float angleVelocity = 0;

  float correction = 0.0496;


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
  float currentAngle = angle;
  

  float allowedRange = 0.015; // acceptable range/2 for the pressure
  if (inRange(currentAngle, targetAngle - allowedRange, targetAngle + allowedRange)) { // stop
    digitalWrite(RIGHT_VALVE_PIN, HIGH);
    digitalWrite(LEFT_VALVE_PIN, LOW);
  }

  else if (currentAngle < targetAngle) { // inflate
    digitalWrite(RIGHT_VALVE_PIN, HIGH);
    digitalWrite(LEFT_VALVE_PIN, HIGH);
  }

  else if (currentAngle > targetAngle) { // deflate
    digitalWrite(RIGHT_VALVE_PIN, LOW);
    digitalWrite(LEFT_VALVE_PIN, LOW);
  }

  
  // ------------- Debugging -------------- //
  
  Serial.print(angle * (180 / PI));
  Serial.print("\t");
  Serial.print("target angle: "); 
  Serial.println(targetAngle * (180 / PI));
  Serial.print("\t");


  // ------------- Wait till next check (can delete if you want but it might cause inconsistencies when messing with other parts of code) -------------- //

  delay(0);

}

// right on , left off : hold pressure
// right on , left on : increase pressure
// right off , left off: increase
