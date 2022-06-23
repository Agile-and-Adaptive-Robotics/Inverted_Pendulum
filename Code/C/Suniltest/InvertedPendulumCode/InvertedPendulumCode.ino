
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_L3GD20_U.h>


Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(20);

int checksPerSecond = 100;

float angle = PI / 2;
float angleVelocity = 0;

float correction = 0;

float lastVelocities[5] = {0, 0, 0, 0, 0};
const int lastVelocitiesLength = 5;

void setup() {
  Serial.begin(9600);

  gyro.enableAutoRange(true);

  if (!gyro.begin()) {
    Serial.println("gyro no working");
  }

  sensors_event_t event;
  gyro.getEvent(&event);


  delay(100);
  
  correction = .12;

  Serial.println(correction);
  
  //sensor_t sensor;
  //gyro.getSensor(&sensor);
  //Serial.println("------------------------------------");
  //Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  //Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  //Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  //Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" rad/s");
  //Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" rad/s");
  //Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" rad/s");  
  //Serial.println("------------------------------------");
  //Serial.println("");

}


void loop() {

  sensors_event_t event;
  gyro.getEvent(&event);

  

  angleVelocity = event.gyro.z - correction;

  // ---------- // calculate if angle velocity is stagnant

  for (int i = lastVelocitiesLength - 1; i > 0; i++) {
    lastVelocities[i] = lastVelocities[i - 1];
  } lastVelocities[0] = angleVelocity;

  bool stagnantVelocity = true;

  for (int j = 0; j < lastVelocitiesLength - 1; j++) {
    if (lastVelocities[j] != lastVelocities[j + 1]) {
      stagnantVelocity = false;
    }
  }

  if (stagnantVelocity) {
    correction += angleVelocity;
    angleVelocity = 0;
  }

  angle += (angleVelocity) / checksPerSecond;

  //Serial.println(angle * (180 / PI));
  //Serial.println(stagnantVelocity);

  delay(1000 / checksPerSecond);
}
