#include <Arduino.h>
#include <RotaryEncoder.h>


RotaryEncoder encoder(11, 12, RotaryEncoder::LatchMode::TWO03);

int lastPosition = 0;
int currentPosition = 0;


// ------------------------------------------------- //


float angle = PI / 2; // angle of the pendulum -- dictated by encoder
const float targetAngle = PI / 2; // angle straight up (90 degrees)


// valve open amounts -- floats from 0 to 1
// 1 = fully open, 0 = closed
float leftValveOpen = 0;
float rightValveOpen = 0;

const float EQUALIBRIUM_OPEN = .5;

/*
 if a valve is open at the equalibrium open value then 
 the air passing in is escaping at the same rate as it is passing out

 if valveOpen is less than equalibrium it is expanding
 if valveOpen is more than equalibrium it is relaxing
 */


//const int LEFT_VALVE_PIN = 3;
//const int RIGHT_VALVE_PIN = 5;


// ------------------------------------------------- //


void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);
  while(!Serial);

  Serial.println("starting");

  pinMode(3, OUTPUT);
  pinMode(5, OUTPUT);

  pinMode(7, INPUT);
  pinMode(8, INPUT);

  pinMode(11, INPUT);
  pinMode(12, INPUT);
}

// ------------------------------------------------- //


void loop() {
  // put your main code here, to run repeatedly:


  // ------------------// random pushes (for simulation)
  int randNum = random(100);
  if (randNum < 10) {
    float angleChange = random(100) - 50;
    angleChange *= PI / 100;
    angle += angleChange;
  }

  
  // ------------------// calculate pwm to send to valves

  
  float correctionAmount = (targetAngle - angle) / 2;

  angle += correctionAmount / 10; // changing angle just for simulation

  leftValveOpen = constrain(EQUALIBRIUM_OPEN - correctionAmount, 0, 1);
  rightValveOpen = constrain(EQUALIBRIUM_OPEN + correctionAmount, 0, 1);
  
  // ------------------// change pwm of valves

  //Serial.println(leftValveOpen);

  int direction = (int)(encoder.getDirection());

  static float leftLEDBrightness = .5;
  static float rightLEDBrightness = .5;

  float dimSpeed = .0002;
  
  if (digitalRead(7) && leftLEDBrightness < 1) {
    leftLEDBrightness += dimSpeed;
    rightLEDBrightness -= dimSpeed;
    
    constrain(leftLEDBrightness, 0, 1);
    constrain(rightLEDBrightness, 0, 1);
    
    analogWrite(3, (int)(leftLEDBrightness * 255));
    analogWrite(5, (int)(rightLEDBrightness * 255));
  }

  
  if (digitalRead(8) && rightLEDBrightness < 1) {
    rightLEDBrightness += dimSpeed;
    leftLEDBrightness -= dimSpeed;

    constrain(rightLEDBrightness, 0, 1);
    constrain(leftLEDBrightness, 0, 1);
    
    analogWrite(5, (int)(rightLEDBrightness * 255));
    analogWrite(3, (int)(leftLEDBrightness * 255));
  }

  //digitalWrite(3, leftValveOpen * 255);
  //digitalWrite(5, rightValveOpen * 255);

  // ------------------// encoder testing

  encoder.tick();

  

  currentPosition = encoder.getPosition();

  if (currentPosition != lastPosition) {
    Serial.println(currentPosition);
    lastPosition = currentPosition;
  }

}
