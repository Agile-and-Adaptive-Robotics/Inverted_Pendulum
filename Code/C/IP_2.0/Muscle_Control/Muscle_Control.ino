

#include <Adafruit_FXOS8700.h>;

Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);

class AccelerationData {
  public:
  float x;
  float y;
  float z;

  void calculateAcceleration() {
    
    sensors_event_t aevent, mevent;
    accelmag.getEvent(&aevent, &mevent);

    x = aevent.acceleration.x;
    y = aevent.acceleration.y;
    z = aevent.acceleration.z;
  }
  
};

AccelerationData acceleration;

int levels = 12;
int delayTime = 8;

// ------------ Muscle Class ------------ //
/*
 * setPWM(int value)   : sets the target length with float from 0 to 1
 * changePWM(int value): changes the target length
 * setValve()          : should be called every loop to control the valve manifold
 * Muscle(int p)       : constructor, sets pin number and initializes pin 
 */

class Muscle {
  public:
    
    int pin;

    float xWeight;
    float yWeight;
    float zWeight;
    
    float PWM = .5;

    int onTotal = 1;
    int offTotal = 1;

    bool on = true;
    int counter = 0;


    
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


    Muscle(int p, float x, float y, float z) {
      pin = p;
      pinMode(pin, OUTPUT);
      digitalWrite(pin, LOW);

      xWeight = x;
      yWeight = y;
      zWeight = z;
    }
    
};
//                  xWeight
//                         yWeight
//                                zWeight
Muscle posterior(2,  1.00,  0.00,  0.00); // posterior is connected to pin 5
Muscle anterior (5,  0.00,  0.00, -0.00); // anterior  is connected to pin 6
Muscle peroneus (4,  0.00,  0.00,  0.00); // peroneus  is connected to pin 7



void setup() {
  Serial.begin(9600);
  while(!Serial);
  Serial.println("beginning");

  posterior.setPWM(.25);
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

void loop() {

  

  
  // ------ decide what to set BPAs to ------ //
/*
  sensors_event_t event;
  gyro.getEvent(&event);

  float angleXVelocity = event.gyro.x - .01;
  float angleYVelocity = event.gyro.y;
  float angleZVelocity = event.gyro.z - .01;

  static int lastAngleTime = 0;
  float deltaAngleTime = millis() - lastAngleTime;
  lastAngleTime = millis();

  static float angleX = 0;
  static float angleY = 0;
  static float angleZ = 0;
  
  angleX += angleXVelocity * (deltaAngleTime / 1000000);
  angleY += angleYVelocity * (deltaAngleTime / 1000000);
  angleZ += angleZVelocity * (deltaAngleTime / 1000000);


  float targetAngleX = PI / 2;
  float targetAngleY = 0;
  float targetAngleZ = PI / 2;

  float deltaAngleX = targetAngleX - angleX;
  float deltaAngleY = targetAngleY - angleY;
  float deltaAngleZ = targetAngleZ - angleZ;
  */

  acceleration.calculateAcceleration();

  // ---- calculate angle ----//

  float totalAcceleration = abs(acceleration.x) + abs(acceleration.y) + abs(acceleration.z);

  float angleX = acceleration.x / totalAcceleration;
  float angleZ = acceleration.y / totalAcceleration;
    
  
  // ------ set BPAs to desired PWMs ------ //

  float moveSpeed = .02;

  static float desiredAngle = 0;

  if (digitalRead(8)) {
    //desiredAngle += moveSpeed;
    posterior.changePWM(.001);
    anterior.changePWM(-.001);
    peroneus.changePWM(-.001);
  }
  if (digitalRead(9)) {
    //desiredAngle -= moveSpeed;
    posterior.changePWM(-.001);
    anterior.changePWM(.001);
    peroneus.changePWM(.001);
  }

  float halfSin = sin(desiredAngle) / 8;
  float halfCos = cos(desiredAngle) / 8;
  
  //posterior.setPWM(.4 + halfSin);
  //anterior.setPWM(.4 + halfCos - halfSin);
  //peroneus.setPWM(.7 - halfCos - halfSin);
  
  posterior.setValve();
  anterior.setValve();
  peroneus.setValve();

  


  // ------ debug ------ //

  //printDebug("posterior", posterior.on + 3);
  //printDebug("anterior", anterior.on + 5);
  //printDebug("peroneus", peroneus.on + 7);
  //printDebug("x", angleX * PI);
  //printDebug("z", angleZ * PI);
  //Serial.println();
  

  

  // ------ delay till next ------ //

  static int lastTime = 0;
  int currentTime = millis();
  int deltaTime = currentTime - lastTime;
  if (delayTime - deltaTime > 0) delay(delayTime - deltaTime);
  lastTime = millis();
}
