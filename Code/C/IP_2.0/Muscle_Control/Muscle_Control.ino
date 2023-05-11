

#include <Adafruit_FXOS8700.h>;

Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);


int levels = 12;
int delayTime = 8;

// ------------ Muscle Class ------------ //
/*
 * setPWM(int value)   : sets the target length with a float from 0 to 1
 * changePWM(int value): changes the target length
 * setValve()          : should be called every loop to control the valve manifold
 * Muscle(int p)       : constructor, sets pin number and initializes pin 
 */

class Muscle {
  public:
    
    int pin;

    float minimumPWM; // PWM at which angle is at minimum value (-90 degrees)
    float maximumPWM; // PWM at which angle is at maximum value ( 90 degrees)

    float idle;

    
    float PWM = .5;

    int onTotal = 1;
    int offTotal = 1;

    bool on = true;
    int counter = 0;

    float jointPWMs[4];


    
    void setPWM(float value) {
      if (value > 1) value = 1;
      if (value < 0) value = 0;
      PWM = value;
      
      value *= levels;

      onTotal = value;
      offTotal = levels - value;

    }

    
    void changePWM(float value) {
      if (PWM + value > 1) value = 1 - PWM;
      if (PWM + value < 0) value = 0 - PWM;
      PWM += value;
      value = PWM;
      
      value *= levels;

      onTotal = value;
      offTotal = levels - value;

    }

  
    void setValve() {
      counter++;

      if (on) {
        if (counter >= onTotal) {
          on = false;
          counter = 0;
        }
      }

      else {
        if (counter >= offTotal) {
          on = true;
          counter = 0;
        }
      }

      if (offTotal == 0) on = true;
      if (onTotal == 0) on = false;

      digitalWrite(pin, on);
    }

/*
    float calculatePWM(float angleX, float angleZ) {
      float xPWM = (xStrength * angleX) + xIdle;
      float zPWM = (zStrength * angleZ) + zIdle;

      float combinedPWM = (xPWM + zPWM) / 2;
      if (combinedPWM > 1) combinedPWM = 1;
      if (combinedPWM < 0) combinedPWM = 0;
      
      return combinedPWM;
    }
*/
    
    float calculatePWM() {
      float totalPWM = 0;
      for (int i = 0; i < 4; i++) {
        totalPWM += constrain(jointPWMs[i], 0, 1);
      }

      return map(totalPWM / 4, 0, 1, minimumPWM, maximumPWM);
    }


    Muscle(int p, float minimum,float maximum, float i) {
      pin = p;
      pinMode(pin, OUTPUT);
      digitalWrite(pin, LOW);

      minimumPWM = minimum;
      maximumPWM = maximum;

      idle = i;
    }
    
};








// 

//              pin
//                  min
//                       max
//                            idle
Muscle posterior(2, 0.0, 1.0, 0.5); // posterior is connected to pin 2
Muscle anterior (5, 0.0, 1.0, 0.5); // anterior  is connected to pin 5
Muscle peroneus (4, 0.0, 1.0, 0.5); // peroneus  is connected to pin 4

Muscle dilongus (6, 0.0, 1.0, 0.5); // dilongus  is connected to pin 6
Muscle halongus (7, 0.0, 1.0, 0.5); // halongus  is connected to pin 7
Muscle diflexor (8, 0.0, 1.0, 0.5); // diflexor  is connected to pin 8
Muscle haflexor (9, 0.0, 1.0, 0.5); // haflexor  is connected to pin 9


float[2] calculatePWMForAngle(float initialAngle, float targetAngle) {

  

  return {0, 0};
}




void setup() {
  Serial.begin(9600);
  while(!Serial);
  Serial.println("beginning");

  posterior.setPWM(.5);
  anterior.setPWM(.5);
  peroneus.setPWM(.5);

  if (!accelmag.begin()) {
    Serial.println("accelerometer not working");
    while(1);
  }

  accelmag.setSensorMode(ACCEL_ONLY_MODE);
  
}

void printDebug(String info, float statement) {
  Serial.print(info);
  Serial.print(":");
  Serial.print(statement);
  Serial.print("\t");
}
