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

  //Serial.println("starting balance platform");



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

  static int stopCounter = 0;
  stopCounter++;

  static bool platformDirection = 0; // 0 = backwards, 1 = forwards, NEED TO MAKE SURE FORWARD == ENCODER POSITION +
  bool lastPlatformDirection = platformDirection;
  
  static int targetEncoderPosition = 0;

  
  const int encoderRange = 100;

  // ------- // switch movement mode
  
  static char mode = RANDOM;

  if (digitalRead(BUTTON_1_PIN)) { // moves to center to start
    mode = RANDOM;
    targetEncoderPosition = 0;
  }
  if (digitalRead(BUTTON_2_PIN)) { // moves to high point
    mode = HIGH_TO_LOW;
    targetEncoderPosition = 50;
  }
  if (digitalRead(BUTTON_3_PIN)) { // moves to center to start
    mode = MANUAL; 
    targetEncoderPosition = 0;
  }

  // ------- // calculate encoder position

  static long encoderPosition = 0;

  long lastEncoderPosition = encoderPosition;

  //encoderPosition = analogRead(1) - 480;


  encoderPosition += (platformDirection - .3333) * 6;
  

  // ------- // move based on mode
  

  if (mode == RANDOM) {
    if (inRange(targetEncoderPosition, lastEncoderPosition, encoderPosition)) { // if encoder position has passed target encoder position since last check
      targetEncoderPosition = random(encoderRange) - (encoderRange / 2);

    }
    
  }

  if (mode == HIGH_TO_LOW) {
    if (inRange(targetEncoderPosition, lastEncoderPosition, encoderPosition)) {

      if (targetEncoderPosition > 0) {
        targetEncoderPosition = -50;
      } else {
        targetEncoderPosition = 50;
      }

    }
  }

  if (mode == MANUAL) { // this is unfinished but I need to see it working to make changes
    static int potentiometerValue = 0;

    potentiometerValue = analogRead(POTENTIOMETER_PIN) - 500;
    
    targetEncoderPosition = potentiometerValue / 10;
  }

  if (encoderPosition == targetEncoderPosition) {
    
  } else {
  
    if (encoderPosition < targetEncoderPosition) {
      platformDirection = 1;
    } else {
      platformDirection = 0;
    }
    
  }


  if (mode == STILL) { // don't move
    //stopCounter = 0;
  
    digitalWrite(9, LOW);
    digitalWrite(7, LOW);
    
    digitalWrite(MOTOR_PIN_1, LOW);
    digitalWrite(MOTOR_PIN_2, LOW);
  } else {
    if (platformDirection) { // moving forward
      
      Serial.print("moving forward to "); 
      Serial.print(targetEncoderPosition); 
      Serial.print(" currently at: "); 
      Serial.println(encoderPosition);
      
      digitalWrite(MOTOR_PIN_1, HIGH);
      digitalWrite(MOTOR_PIN_2, LOW);

      digitalWrite(9, HIGH);
      digitalWrite(7, LOW);
    } else { // moving backward
      
      Serial.print("moving backward to ");
      Serial.print(targetEncoderPosition);
      Serial.print(" currently at: ");
      Serial.println(encoderPosition);
      
      digitalWrite(MOTOR_PIN_1, LOW);
      digitalWrite(MOTOR_PIN_2, HIGH);

      digitalWrite(9, LOW);
      digitalWrite(7, HIGH);
    }
  }

  if (platformDirection != lastPlatformDirection) {
    stopCounter = 0;
  }

  //delay(10);  

}
