// ------------------ // Gyroscope stuff

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_L3GD20_U.h>

Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(20);


// ------------------ // Constants

<<<<<<< HEAD
#define RIGHT_BUTTON_PIN 3 // ** change these **
#define LEFT_BUTTON_PIN 2

#define RIGHT_VALVE_PIN 9
#define LEFT_VALVE_PIN 10

#define PRESSURE_SENSOR_PIN A7 // analog pin

const float MIN_PRESSURE = 0; // ** change these with minimum and maximum pressures that the pressure sensor returns **
const float MAX_PRESSURE = 1023;

=======
#define RIGHT_BUTTON_PIN 8 // ** change these **
#define LEFT_BUTTON_PIN 9

#define LEFT_VALVE_PIN_1 2
#define LEFT_VALVE_PIN_2 3

#define RIGHT_VALVE_PIN_1 4
#define RIGHT_VALVE_PIN_2 5

#define LEFT_PRESSURE_PIN 1  // analog
#define RIGHT_PRESSURE_PIN 0 // analog
>>>>>>> 2798c53721327a473aeaa6999945be2c2fa075a9


void setup() {
  Serial.begin(9600);
  
  pinMode(RIGHT_BUTTON_PIN, INPUT);
  pinMode(LEFT_BUTTON_PIN, INPUT);

<<<<<<< HEAD
  pinMode(RIGHT_VALVE_PIN, OUTPUT);
  pinMode(LEFT_VALVE_PIN, OUTPUT);

  gyro.enableAutoRange(true);

  if (!gyro.begin(gyro.L3DS20_RANGE_2000DPS)) {
    Serial.println("gyro not working");
=======
  pinMode(LEFT_VALVE_PIN_1, OUTPUT);
  pinMode(LEFT_VALVE_PIN_2, OUTPUT);

  pinMode(RIGHT_VALVE_PIN_1, OUTPUT);
  pinMode(RIGHT_VALVE_PIN_2, OUTPUT);

  gyro.enableAutoRange(true);

  if (!gyro.begin()) {
    Serial.println("gyro not working");
    while(1); // stop program
>>>>>>> 2798c53721327a473aeaa6999945be2c2fa075a9
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
  
<<<<<<< HEAD
  static float targetPressure;
=======
  static float targetAngle = PI / 2;
>>>>>>> 2798c53721327a473aeaa6999945be2c2fa075a9

  // ------------- Button inputs -------------- //
  

<<<<<<< HEAD
  float moveSpeed = PI/1000; // ** increase this if it's inflating too slow ** (derivative of target pressure kinda)

  if (digitalRead(RIGHT_BUTTON_PIN)) { // increase target pressure
    targetPressure += moveSpeed;
  }
  
  if (digitalRead(LEFT_BUTTON_PIN)) { // decrease target pressure
    targetPressure -= moveSpeed;
  }

  //if (targetPressure < MIN_PRESSURE) targetPressure = MIN_PRESSURE;
  //if (targetPressure > MAX_PRESSURE) targetPressure = MAX_PRESSURE;

=======
  float moveSpeed = PI/800; // ** increase this if it's inflating too slow ** (derivative of target pressure kinda)

  if (digitalRead(RIGHT_BUTTON_PIN)) { // increase target pressure
    targetAngle += moveSpeed;
  }
  
  if (digitalRead(LEFT_BUTTON_PIN)) { // decrease target pressure
    targetAngle -= moveSpeed;
  }

>>>>>>> 2798c53721327a473aeaa6999945be2c2fa075a9

  // ------------- Calculate angle -------------- //

  static int loopCounter = 0;
  loopCounter++;

<<<<<<< HEAD
  static float angle = 0; // angle assumes that it is straight up on start
  float angleVelocity = 0;
  const float targetAngle = PI / 2; // angle straight up (90 degrees)

  float correction = 0.0523;
=======
  static float angle = PI / 2; // angle assumes that it is straight up on start
  float angleVelocity = 0;

  float correction = 0.0495;
>>>>>>> 2798c53721327a473aeaa6999945be2c2fa075a9


  sensors_event_t event;
  gyro.getEvent(&event);
  
  if (loopCounter > 10) {
    angleVelocity = event.gyro.z - correction;
  } else {
    angleVelocity = 0;
  }

  static float lastTime = 0;
<<<<<<< HEAD

  float currentTime = millis();

  

  angle -= angleVelocity * ((currentTime - lastTime) / 1000);


=======
  float currentTime = millis();
  float angleChange = angleVelocity * ((currentTime - lastTime) / 1000);
  int roundedness = 5000;
  int roundedAngleChange = (int)((angleChange * roundedness) + .5);
  angle -= (float)(roundedAngleChange) / roundedness;
>>>>>>> 2798c53721327a473aeaa6999945be2c2fa075a9
  lastTime = currentTime;

  // ------------- Valve control -------------- //
  
  //int currentPressure = analogRead(PRESSURE_SENSOR_PIN);
<<<<<<< HEAD
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
=======
  float currentAngle = angle;
  

  float allowedRange = 0.03; // acceptable range/2 for the pressure
  if (targetAngle - allowedRange < currentAngle && currentAngle < targetAngle + allowedRange) { // stop
    digitalWrite(LEFT_VALVE_PIN_1, HIGH);
    digitalWrite(LEFT_VALVE_PIN_2, LOW);
    
    digitalWrite(RIGHT_VALVE_PIN_1, HIGH);
    digitalWrite(RIGHT_VALVE_PIN_2, LOW);
  }

  else if (currentAngle > targetAngle) { // rotate right
    digitalWrite(LEFT_VALVE_PIN_1, LOW);
    digitalWrite(LEFT_VALVE_PIN_2, LOW);
    
    digitalWrite(RIGHT_VALVE_PIN_1, HIGH);
    digitalWrite(RIGHT_VALVE_PIN_2, HIGH);
  }

  else if (currentAngle < targetAngle) { // rotate left
    digitalWrite(LEFT_VALVE_PIN_1, HIGH);
    digitalWrite(LEFT_VALVE_PIN_2, HIGH);
    
    digitalWrite(RIGHT_VALVE_PIN_1, LOW);
    digitalWrite(RIGHT_VALVE_PIN_2, LOW);
>>>>>>> 2798c53721327a473aeaa6999945be2c2fa075a9
  }

  
  // ------------- Debugging -------------- //
<<<<<<< HEAD
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
  
=======
  
  Serial.print(angle * (180 / PI));
  Serial.print("\t");
  Serial.print("target angle: "); 
  Serial.print(targetAngle * (180 / PI));
  Serial.println("\t");
  

  //Serial.print("left: "); Serial.print(analogRead(LEFT_PRESSURE_PIN));
  //Serial.print("\t");
  //Serial.print("right: "); Serial.println(analogRead(RIGHT_PRESSURE_PIN));

>>>>>>> 2798c53721327a473aeaa6999945be2c2fa075a9
  // ------------- Wait till next check (can delete if you want but it might cause inconsistencies when messing with other parts of code) -------------- //

  delay(0);

}

// right on , left off : hold pressure
// right on , left on : increase pressure
// right off , left off: increase
