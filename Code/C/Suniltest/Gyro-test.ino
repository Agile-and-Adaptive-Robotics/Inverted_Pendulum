
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_L3GD20_U.h>


Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(20);


void setup() {
  Serial.begin(9600);

  gyro.enableAutoRange(true);

  if (!gyro.begin()) {
    Serial.println("gyro no working");
  }

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

int checksPerSecond = 100;

float angle = PI / 2;
float angleVelocity = 0;

void loop() {

  sensors_event_t event;
  gyro.getEvent(&event);

  

  angleVelocity = event.gyro.z;

  float correction = .14;

  angle += (angleVelocity) / checksPerSecond;

  Serial.println(angle * (180 / PI));
  //Serial.println(angleVelocity);

  delay(1000 / checksPerSecond);
}
