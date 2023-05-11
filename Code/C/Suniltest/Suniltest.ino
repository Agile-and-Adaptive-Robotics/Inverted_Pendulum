/*
#include <Arduino.h>
#include <RotaryEncoder.h>

RotaryEncoder encoder(6, 9, RotaryEncoder::LatchMode::TWO03);


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

/*
const int LEFT_VALVE_PIN = 3;
const int RIGHT_VALVE_PIN = 5;


// ------------------------------------------------- //


// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.

  Serial.begin(9600);


  pinMode(LEFT_VALVE_PIN, OUTPUT);
  pinMode(RIGHT_VALVE_PIN, OUTPUT);
}





// ------------------------------------------------- //

// the loop function runs over and over again forever
void loop() {

  // ------------------// encoder testing

  encoder.tick();
  currentPosition = encoder.getPosition();

  Serial.println(currentPosition);

  
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
  
  //analogWrite(LEFT_VALVE_PIN, constrain(leftValveOpen - EQUALIBRIUM_OPEN, 0, 1) * 255);
  //analogWrite(RIGHT_VALVE_PIN, constrain(rightValveOpen - EQUALIBRIUM_OPEN, 0, 1) * 255);

  
  
  delay(10);
}*/
