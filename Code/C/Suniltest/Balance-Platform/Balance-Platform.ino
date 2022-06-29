// ------------------ // pin numbers

#define MOTOR_PIN_1 22
#define MOTOR_PIN_2 23

#define ENCODER_PIN_1 19
#define ENCODER_PIN_2 20


#define BUTTON_1_PIN 3
#define BUTTON_2_PIN 18
#define BUTTON_3_PIN 2

// potentiometer is on analog 0

#define POTENTIOMETER_PIN 0


// constants for various balance platform modes

#define STILL 0
#define RANDOM 1
#define HIGH_TO_LOW 2
#define MANUAL 3 // dictated by potentiometer



// ------------------ // Encoder Stuff

#include <Arduino.h>
#include <RotaryEncoder.h>


RotaryEncoder encoder(ENCODER_PIN_1, ENCODER_PIN_2, RotaryEncoder::LatchMode::TWO03);




void setup() {
  Serial.begin(9600);

  while(!Serial);

  Serial.println("starting balance platform");



}

bool inRange(float x, float a, float b) {
  if (a < b) {
    if (a <= x && x <= b) return true;
  } else {
    if (b <= x && x <= a) return true;
  }
  return false;
}

void loop() {

  static bool platformDirection = 0; // 0 = backwards, 1 = forwards NEED TO MAKE SURE FORWARD == ENCODER POSITION +
  
  static int targetEncoderPosition = 0;

  
  const int encoderRange = 20;

  // ------- // switch movement mode
  
  static char mode = MANUAL;

  if (digitalRead(BUTTON_1_PIN)) mode = RANDOM; targetEncoderPosition = 0; // moves to center to start
  if (digitalRead(BUTTON_2_PIN)) mode = HIGH_TO_LOW; targetEncoderPosition = 10; // moves to high point
  if (digitalRead(BUTTON_3_PIN)) mode = MANUAL; targetEncoderPosition = 0; // moves to center to start

  // ------- // calculate encoder position

  static long encoderPosition = 0;

  long lastEncoderPosition = encoderPosition;

  encoder.tick();
  encoderPosition = encoder.getPosition();

  Serial.println(encoderPosition);
  

  // ------- // move based on mode
  

  if (mode == RANDOM) {
    if (inRange(targetEncoderPosition, lastEncoderPosition, encoderPosition)) { // if encoder position has passed target encoder position since last check
      targetEncoderPosition = random(encoderRange) - (encoderRange / 2);
    }
    
  }

  if (mode == HIGH_TO_LOW) {
    if (inRange(targetEncoderPosition, lastEncoderPosition, encoderPosition)) {
      targetEncoderPosition = -targetEncoderPosition; // relies on target encoder position being set to the highest/lowest encoder position desired on switch to HIGH_TO_LOW
    }
  }

  if (mode == MANUAL) { // this is unfinished but I need to see it working to make changes
    static int potentiometerValue = 0;

    potentiometerValue = analogRead(POTENTIOMETER_PIN);

    targetEncoderPosition = potentiometerValue;
  }


  if (encoderPosition < targetEncoderPosition) {
    platformDirection = 1;
  } else {
    platformDirection = 0;
  }

  if (mode == STILL) {
    // don't move ?
  } else {
    if (platformDirection) { // moving forward
      digitalWrite(MOTOR_PIN_1, HIGH);
      digitalWrite(MOTOR_PIN_2, LOW);
    } else { // moving backward
      digitalWrite(MOTOR_PIN_1, LOW);
      digitalWrite(MOTOR_PIN_2, HIGH);
    }
  }

  delay(100);  

}
