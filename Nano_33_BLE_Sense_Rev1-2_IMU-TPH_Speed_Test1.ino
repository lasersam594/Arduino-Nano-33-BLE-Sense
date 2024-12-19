/*
Nano BLE 33 Sense Sensor Test V12.

Copyright® Samuel M. Goldwasser, 1994-2024, all rights reserved.  Permission is granted for public use or modification as
long as the Copyright notice is included.

This a simple utility to exercise the Nano BLE 33 Sense Rev1 or Rev2 magnetic field and T/P/H sensors with the objective
of finding out why Rev1 appears to be 2 to 3 times faster than Rev2 for these measurements.  The serial port may be
used to display the measurement values.  The power LED flashes at 1/2 the loop repetition rate.

The required Nano BLE 33 Sense libraries are all either built into the Arduino IDE or Arduino Cloud Editor, or readily
found via a Web search.  Note that the primary difference between the Rev1 and Rev2 sketches are the libraries for the
IMU and T/H.

Gyroscope (Degs/s) Roll, Pitch, Yaw; Magnetic Field (Gauss) X, Y, Z; Temperature (DegC),
Pressure (mm/Hg), Humidity (%) are all optionally sent via the serial port as data-only, or with labels.

In addition, the on-board PWR_LED flashes at 1/8th the loop speed and RGB_LED provide visual output of Gyro action.

1. Gyroscope: Displays the absolute value for Roll, Pitch, and Yaw as the brightness of each if the RGB leds.  Optional Gyro
   calibration to compensate for board-specific roll, pitch, and yaw offsets.  If enabled, Nano must remain stationary at
   startup while the RGB LEDs are blinking.  The default duration is ~12 blinks.  This may only be needed for Rev1 boards.
2. Heartbeat: The PWR_LED flashes at 1/8th of the loop speed.

Suggestions for (modest!) improvements welcome.

If this is the first time using a Nano 33 BLE Sense, install the necessary board and libraries in the Arduino IDE:

1. Go to Tools > Board > Boards Manager or click the Boards icon, type the keyword "ble" in the search box, install "Arduino
   Mbed OS Nano Boards".
2. Download the relevant sensor libraries from their GitHub repositories.  For this sketch they will either be LSM9DS1.h
   and HTS221.h (Rev1) or LPS22HB.h and APDS9960A.h (Rev2).  A Web search will find them by name.  Go to Sketch > Include
   Library > Add Zip Library, and point to the files downloaded above.
3. Go to Tools > Board, and select Arduino MBed OS Nano Boards > Arduino Nano 33 BLE.
4. Also select the board version, data formatting and Gyro AutoCal options in the #defines, below.

This sketch should then compile without errors (though there may be warnings that can be ignored). ;-)
*/

// Select Nano BLE 33 Sense Rev1 or Rev2
#define Rev1              // Select based on specific board

// User parameters
#define data1 1           // Sends data to serial port if 1, LEDs-only on Nano if 0
#define verbose1 1        // Display labels if 1, data-only if 0
#define senddiag1 0       // Include diagnostic information iff 1.  TBD, currently one integer (diag) is coded.
#define GyroAutoCal 1     // Perform automatic Gyro offset compensation at startup: The board must be stationary
                          // while the LEDs are blinking.  If not enabled, use #define GR/GP/GY_COR values.

// Gyro offset parameters and variables
#define CalValues 50      // Number of Gyro samples to average for calibration

float RollOffsetSum = 0;  // Temporary variables for Gyro AutoCal sums
float PitchOffsetSum = 0;
float YawOffsetSum = 0;

float GR_COR = 0;         // Gyro offset correction values
float GP_COR = 0;
float GY_COR = 0;

int CalCount = CalValues;
int GyroAutoCalFlag = 0;
float pgr, pgp, pgy;

// Fixed calibration values may be needed if Gyro AutoCal is not enabled (probably only for Rev1)

/*
GR_COR = 6.5;
GP_COR = 0;
GY_COR = 2.5;
*/

// Color palette for audio in RGB_LEDs
#define BLACK 0, 0, 0
#define GRAY 7, 7, 7
#define MAGENTA 25, 0, 25
#define BLUE 0, 0, 75
#define CYAN 0, 50, 50
#define GREEN 0, 192, 0
#define YELLOW 128, 92, 0
#define ORANGE 200, 40, 0
#define RED 255, 0, 0
#define WHITE 255, 255, 255

#include <Arduino.h>

// Nano 33 BLE Sense Version for libraries and loop speed
#ifdef Rev1
#include <Arduino_LSM9DS1.h>  // Accelerometer, magnetometer and gyroscope
#include <Arduino_HTS221.h>   // Temperature and humidity
#define timeoutvalue  17
#endif

#ifdef Rev2
#include <Arduino_BMI270_BMM150.h>  // Accelerometer, magnetometer and gyroscope
#include <Arduino_HS300x.h>         // Temperature and humidity
#define timeoutvalue  17
#endif

// Common library
#include <Arduino_LPS22HB.h>        // Pressure

int count = 0;
int i = 0;
int timeout = 0;

void setup() {

  // Set the LEDs pins as outputs
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_PWR, OUTPUT);
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);

  // Turn the LED_BUILTIN and LED_PWR on and set the RGB LEDs at low brightness
  analogWrite(LED_BUILTIN, 0);
  digitalWrite(LED_PWR, 1);
  analogWrite(LEDR, 200);
  analogWrite(LEDG, 200);
  analogWrite(LEDB, 200);

  if (data1 == 1) {
    Serial.begin(9600);
    while (!Serial);
    Serial.println();
    Serial.println();
    Serial.println("Arduino Nano 33 BLE Sense Rev1-2 Speed Test1.");
  }

  // Startup

  if (!IMU.begin()) {
    if (data1 == 1) Serial.println("Failed to initialize IMU!");
    while (1);
  }

  // Version-specific temperature and humidity sensor libraries

#ifdef Rev1
  if (!HTS.begin()) {
#endif

#ifdef Rev2
  if (!HS300x.begin()) {
#endif

      if (data1 == 1) Serial.println("Failed to initialize temperature and humidity sensor!");
      while (1);
    }

    if (!BARO.begin()) {
      if (data1 == 1) Serial.println("Failed to initialize pressure sensor!");
      while (1);
    }

    // Banner blurb
    if ((verbose1 == 1) && (data1 == 1)) {
      Serial.println();
      Serial.println("Functions:");
      Serial.println();
      Serial.println("  - Gyro angle in degrees/second. LED threshold 25.");
      Serial.println("  - Magnetic field in Gauss.");
      Serial.println("  - Temperature in degrees Centigrade.");
      Serial.println("  - Pressure in mm/Hg.");
      Serial.println("  - Humidity in rel %.");
      Serial.println();
      Serial.println("Data:");
      Serial.println("");
    }
  }

  void loop() {
    float ax, ay, az, gr, gp, gy, mx, my, mz;
    char buffer[40];
    int led, ledr, ledp, ledy;
    int diag = 0;

    // Accelerometer

/*
    while (!IMU.accelerationAvailable()) {}
    IMU.readAcceleration(ax, ay, az);
    if (data1 == 1) {
      if (verbose1 == 1) Serial.print("  Axl (Gs) X: ");
      sprintf(buffer, "%5.2f", ax);
      Serial.print(buffer);
      if (verbose1 == 1) Serial.print(" Y: ");
      sprintf(buffer, "%5.2f", ay);
      Serial.print(buffer);
      if (verbose1 == 1) Serial.print(" Z: ");
      sprintf(buffer, "%5.2f", az);
      Serial.print(buffer);
      if (verbose1 == 1) Serial.print(" | ");
    }
*/
    // Gyroscope

    while (!IMU.gyroscopeAvailable()) {}
    pgr = gr; pgp = gp; pgy = gy;
    IMU.readGyroscope(gr, gp, gy);
    if (data1 == 1) {
      if (verbose1 == 1) Serial.print("Gyro (Degs/s) R: ");
      sprintf(buffer, "%8.2f", gr - GR_COR);
      Serial.print(buffer);
      if (verbose1 == 1) Serial.print("  P: ");
      sprintf(buffer, "%8.2f", gp - GP_COR);
      Serial.print(buffer);
      if (verbose1 == 1) Serial.print("  Y: ");
      sprintf(buffer, "%8.2f", gy - GY_COR);
      Serial.print(buffer);
      if (verbose1 == 1) Serial.print(" | ");
    }

// Gyro AutoCal
    if (GyroAutoCal == 1) {
      if (CalCount == CalValues) {
        CalCount--;           // Skip corrupted first value
        GyroAutoCalFlag = 1;  // Disable RGB_LED output while GyroAutoCal in progress
      }
      else if (CalCount > 1) {
        if (((fabs(gr - pgr) > 2)) || ((fabs(gp - pgp) > 2)) || ((fabs(gr - pgr) > 2))) { // Start over if too much gyro activity
          CalCount = CalValues;
          RollOffsetSum = 0;
          PitchOffsetSum = 0;
          YawOffsetSum = 0;
        } 
        if ((CalCount & 3) == 2) RGB_LED_Color(GRAY); // Heartbeat while AutoCal in progress
        else RGB_LED_Color(BLACK);
        RollOffsetSum += gr;
        PitchOffsetSum += gp;
        YawOffsetSum += gy;
        CalCount--;
      } else if (CalCount == 1) { // Compute average offsets
        GR_COR = RollOffsetSum / CalValues;
        GP_COR = PitchOffsetSum / CalValues;
        GY_COR = YawOffsetSum / CalValues;
        CalCount = 0;
        GyroAutoCalFlag = 0;
        RGB_LED_Color(BLACK);
      }
    }
    
    ledr = fabs(gr - GR_COR) / 2;
    ledp = fabs(gp - GP_COR) / 2;
    ledy = fabs(gy - GY_COR) / 2;

    if (GyroAutoCalFlag == 0) RGB_LED_Color(ledr, ledp, ledy);
    else if ((ledr > 0) || (ledp > 0) || (ledy > 0)) timeout = 16;
    if (timeout > 0) timeout--;

    // Magnetic field

    while (!IMU.magneticFieldAvailable()) {}
    IMU.readMagneticField(mx, my, mz);
    if (data1 == 1) {
      if (verbose1 == 1) Serial.print("Field (Gauss) X: ");
      sprintf(buffer, "%5.2f", mx / 100);
      Serial.print(buffer);
      if (verbose1 == 1) Serial.print("  Y: ");
      sprintf(buffer, "%5.2f", my / 100);
      Serial.print(buffer);
      if (verbose1 == 1) Serial.print("  Z: ");
      sprintf(buffer, "%5.2f", mz / 100);
      Serial.print(buffer);
      if (verbose1 == 1) Serial.print(" |");
    }

    // Temperature, humidity, and pressure

#ifdef Rev1
    float temperature = HTS.readTemperature();
    float humidity = HTS.readHumidity();
#endif

#ifdef Rev2
    float temperature = HS300x.readTemperature();
    float humidity = HS300x.readHumidity();
#endif

    float pressure = BARO.readPressure();

    if (data1 == 1) {
      if (verbose1 == 1) Serial.print(" T:");
      sprintf(buffer, " %5.2f", temperature);
      Serial.print(buffer);
      if (verbose1 == 1) Serial.print(" DegC; P: ");
      sprintf(buffer, "%6.2f", pressure * 7.50062);
      Serial.print(buffer);
      if (verbose1 == 1) Serial.print(" mm/Hg; H: ");
      sprintf(buffer, "%5.2f", humidity);
      Serial.println(buffer);
    }

    analogWrite(LED_BUILTIN, 0);

    // Heartbeat (disabled)

    count++;
    if (count >= timeoutvalue) {
      if (timeout == 0) {
   //   analogWrite(LED_BUILTIN, 255);  // Pulse BUILTIN led
        count = 0;
      }
   // else analogWrite(LED_BUILTIN, 0);
    }

    delay(1);
    if ((count & 7) == 7) digitalWrite(LED_PWR, !(digitalRead(LED_PWR))); // Diagnostic loop rate indicator
}

  void RGB_LED_Color(int r, int g, int b) {
    analogWrite(LEDR, 255 - r);
    analogWrite(LEDG, 255 - g);
    analogWrite(LEDB, 255 - b);
  }

