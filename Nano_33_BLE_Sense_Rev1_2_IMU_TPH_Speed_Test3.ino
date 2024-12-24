/*
Nano BLE 33 Sense Rev1-2 Sensor Speed Test V3.

Copyright® Samuel M. Goldwasser, 1994-2024, all rights reserved.  Permission is granted for public use or modification as
long as the Copyright notice is included.

This a simple utility to exercise the Nano BLE 33 Sense Rev1 or Rev2 magnetic field sensor with the objective
of finding out why Rev1 appears to be twice as fast as Rev2 for these measurements.  The serial port may be
used to display the loop time and measurement values.  The PWR LED goes out during IMU initialization and the
BUILTIN LED flashes at 1/8th the loop repetition rate while capturing magnetic field data.

The required Nano BLE 33 Sense libraries are all either built into the Arduino IDE or Arduino Cloud Editor, or readily found
via a Web search.  Note that the only difference between the Rev1 and Rev2 sketches is the library for the the IMU.
*/

#define Rev2                        // Select Nano BLE 33 Sense Rev1 or Rev2 here

#define data1 1                     // Send loop time in milliseconds to serial port if 1

#include <Arduino.h>

// Nano 33 BLE Sense Version for libraries
#ifdef Rev1
#include <Arduino_LSM9DS1.h>        // Accelerometer, magnetometer and gyroscope
#endif

#ifdef Rev2
#include <Arduino_BMI270_BMM150.h>  // Accelerometer, magnetometer and gyroscope
#endif

int count = 0;
int ms = 0;          // Current time in ms
int previous_ms = 0; // Previous time in ms

void setup() {

  pinMode(LED_BUILTIN, OUTPUT);  // BUILTIN LED for a cycle of 1/8th loop time
  pinMode(LED_PWR, OUTPUT);      // PWR LED off during IMU initialization

  // Serial port
   if (data1 == 1) {
    Serial.begin(9600);
    while (!Serial);
    Serial.println();
    Serial.println();
    Serial.println("Arduino Nano 33 BLE Sense Rev1-2 Speed Test.");
    Serial.println();
  }

  digitalWrite(LED_PWR, 0);
  if (!IMU.begin()) while (1); // Initialize IMU
  digitalWrite(LED_PWR, 1);
}

  void loop() {
    
    float mx, my, mz;
    char buffer[40];

    // Loop time readout
    if (data1 == 1) {
      previous_ms = ms;
      ms = millis();
      Serial.print("Loop Time: ");
      sprintf(buffer, "%3d", ms - previous_ms);
      Serial.print(buffer);
      Serial.print( " ms | ");
    }

    // Magnetic field
    while (!IMU.magneticFieldAvailable());
    IMU.readMagneticField(mx, my, mz);
    if (data1 == 1) {
      Serial.print("Field (Gauss) X: ");
      sprintf(buffer, "%5.2f", mx / 100);
      Serial.print(buffer);
      Serial.print("  Y: ");
      sprintf(buffer, "%5.2f", my / 100);
      Serial.print(buffer);
      Serial.print("  Z: ");
      sprintf(buffer, "%5.2f", mz / 100);
      Serial.print(buffer);
      Serial.println("");
    }

    if ((count & 7) == 7) digitalWrite(LED_BUILTIN, !(digitalRead(LED_BUILTIN))); // Diagnostic 1/8th loop rate indicator
    count ++;
}
