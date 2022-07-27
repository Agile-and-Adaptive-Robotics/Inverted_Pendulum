// ------------------ // Gyroscope stuff

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_L3GD20_U.h>

Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(20);


// ------------------ // Constants

#define RIGHT_BUTTON_PIN 7 // ** change these **
#define LEFT_BUTTON_PIN 6

#define LEFT_VALVE_PIN_1 10
#define LEFT_VALVE_PIN_2 9

#define RIGHT_VALVE_PIN_1 12
#define RIGHT_VALVE_PIN_2 11

#define LEFT_PRESSURE_PIN 1  // analog
#define RIGHT_PRESSURE_PIN 0 // analog


void setup() {
  Serial.begin(9600);
  
  pinMode(RIGHT_BUTTON_PIN, INPUT);
  pinMode(LEFT_BUTTON_PIN, INPUT);

  pinMode(LEFT_VALVE_PIN_1, OUTPUT);
  pinMode(LEFT_VALVE_PIN_2, OUTPUT);

  pinMode(RIGHT_VALVE_PIN_1, OUTPUT);
  pinMode(RIGHT_VALVE_PIN_2, OUTPUT);

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
  
  static float targetAngle = PI / 2;

  // ------------- Button inputs -------------- //
  

  float moveSpeed = PI/800; // ** increase this if it's inflating too slow ** (derivative of target pressure kinda)

  if (digitalRead(RIGHT_BUTTON_PIN)) { // increase target pressure
    targetAngle += moveSpeed;
  }
  
  if (digitalRead(LEFT_BUTTON_PIN)) { // decrease target pressure
    targetAngle -= moveSpeed;
  }


  // ------------- Calculate angle -------------- //

  static int loopCounter = 0;
  loopCounter++;

  static float angle = PI / 2; // angle assumes that it is straight up on start
  float angleVelocity = 0;

  float correction = 0.0476;


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
  }

  
  // ------------- Debugging -------------- //
  /*
  Serial.print(angle * (180 / PI));
  Serial.print("\t");
  Serial.print("target angle: "); 
  Serial.println(targetAngle * (180 / PI));
  Serial.print("\t");
  */

  Serial.print("left: "); Serial.print(analogRead(LEFT_PRESSURE_PIN));
  Serial.print("\t");
  Serial.print("right: "); Serial.println(analogRead(RIGHT_PRESSURE_PIN));

  // ------------- Wait till next check (can delete if you want but it might cause inconsistencies when messing with other parts of code) -------------- //

  delay(0);

}

// right on , left off : hold pressure
// right on , left on : increase pressure
// right off , left off: increase
