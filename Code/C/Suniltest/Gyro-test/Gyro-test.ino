
// ------------------ // Gyroscope stuff

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_L3GD20_U.h>

Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(20);

// ------------------ // Encoder Stuff

#include <Arduino.h>
#include <RotaryEncoder.h>


RotaryEncoder encoder(11, 12, RotaryEncoder::LatchMode::TWO03);

int lastPosition = 0;
int currentPosition = 0;


// ------------------ //


int checksPerSecond = 10;

float angle = PI / 2;
float angleVelocity = 0;
const float targetAngle = PI / 2; // angle straight up (90 degrees)

float correction = 0;

float lastVelocities[5] = {0, 0, 0, 0, 0};
const int lastVelocitiesLength = 5;

// ------------------ // valve open amounts -- floats from 0 to 1

// 1 = fully open, 0 = closed
float leftValveOpen = 0;
float rightValveOpen = 0;

const float EQUALIBRIUM_OPEN = .5;

const int LEFT_VALVE_PIN = 3;
const int RIGHT_VALVE_PIN = 5;


// ------------------------------------------------- //

bool inRange(float x, float a, float b) {
  if (a < b) {
    if (a <= x && x <= b) return true;
    else return false;
  } else {
    if (b <= x && x <= a) return true;
    else return false;
  }
}

void setup() {
  Serial.begin(9600);
  while(!Serial);

  Serial.println("starting");

  pinMode(LEFT_VALVE_PIN, OUTPUT);
  pinMode(RIGHT_VALVE_PIN, OUTPUT);

  pinMode(11, INPUT);
  pinMode(12, INPUT);

  gyro.enableAutoRange(true);

  if (!gyro.begin()) {
    Serial.println("gyro not working");
  }
  
}


// ------------------------------------------------- //


void loop() {

  // ------------------ // calculate angle

  sensors_event_t event;
  gyro.getEvent(&event);
  
  angleVelocity = event.gyro.z - correction;
  
  for (int i = lastVelocitiesLength - 1; i > 0; i--) {
    lastVelocities[i] = lastVelocities[i - 1];
  } lastVelocities[0] = angleVelocity;
  
  bool stagnantVelocity = true;

  for (int j = 0; j < lastVelocitiesLength - 1; j++) {
    if (!inRange(lastVelocities[j], lastVelocities[j + 1] - .01, lastVelocities[j + 1] + .01)) {
      stagnantVelocity = false;

    }
  }

  if (stagnantVelocity) {
    correction += angleVelocity;
    angleVelocity = 0;
  }

  angle += angleVelocity / checksPerSecond;

  //Serial.println(angle * (180 / PI));
  //Serial.println(correction);
  
  // ------------------ // calculate pwm to send to valves

  /*
   to move to the right, leftValveOpen decreases (left valve closes, left BPA inflates) and opposite for right valve
   opposite is done to move to the left
  */
  
  float correctionAmount = (targetAngle - angle) / 2;

  //angle += correctionAmount / 10; // changing angle just for simulation, in real life this would change due valves closing/opening


  leftValveOpen = EQUALIBRIUM_OPEN - correctionAmount, 0, 1; // close more (inflate) to move to the right (positive correctionAmount)
  rightValveOpen = EQUALIBRIUM_OPEN + correctionAmount, 0, 1; // open more (deflate) to move to right (with positive correctionAmount)

  // ------ // make sure pressure isn't too high

  float leftPressure = 50; // SET THIS LATER, 50 is just a placeholder
  float rightPressure = 50;

  float maxPressure = 90;

  if (leftPressure > maxPressure) {
    leftValveOpen += .1;
  }
  if (rightPressure > maxPressure) {
    rightValveOpen += .1;
  }

  // ------ // make sure pressure isn't too high

  if (leftValveOpen < 0) leftValveOpen = 0;
  if (leftValveOpen > 1) leftValveOpen = 1;
  if (rightValveOpen < 0) rightValveOpen = 0;
  if (rightValveOpen > 1) rightValveOpen = 1;
  
  // ------------------// change pwm of valves

  Serial.print((angle * (180 / PI))); Serial.print(" : "); Serial.print(leftValveOpen); Serial.print(" : "); Serial.print(rightValveOpen); Serial.println();
  //Serial.println(correctionAmount);
  
  analogWrite(LEFT_VALVE_PIN, (leftValveOpen * leftValveOpen) * 255);
  analogWrite(RIGHT_VALVE_PIN, (rightValveOpen * rightValveOpen) * 255);

  // ------------------// encoder testing

  encoder.tick();

  int currentPosition = encoder.getPosition();

  if (currentPosition != lastPosition) {
    //Serial.println((int)(encoder.getDirection()));
    lastPosition = currentPosition;
  }


  delay(1000 / checksPerSecond);
}
