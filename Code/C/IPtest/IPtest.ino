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
  // nothing yet
}

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
