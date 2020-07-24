// plan
  // design controller to figure out which commands to output
  // valve manifold only on/off (bang-bang)
    // PWM
  // Cascaded control
  // timer interrupt not void loop to read pressure sensors & IMU
  // IMU & pressure sensors & encoder & SD card slot
  // hopefully figure out OOP

#include <Adafruit_FXAS21002C.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// Assign devices
Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);

void displaySensorDetails(void) {
  sensor_t sensor;
  gyro.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print("Sensor:       ");
  Serial.println(sensor.name);
  Serial.print("Driver Ver:   ");
  Serial.println(sensor.version);
  Serial.print("Unique ID:    0x");
  Serial.println(sensor.sensor_id, HEX);
  Serial.print("Max Value:    ");
  Serial.print(sensor.max_value);
  Serial.println(" rad/s");
  Serial.print("Min Value:    ");
  Serial.print(sensor.min_value);
  Serial.println(" rad/s");
  Serial.print("Resolution:   ");
  Serial.print(sensor.resolution);
  Serial.println(" rad/s");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void setup() {
  Serial.begin(9600);
  displaySensorDetails();

}

void loop() {
  // put your main code here, to run repeatedly:

}
