/*
Nano BLE 33 Sense Sensor Test V16.

Copyright® Samuel M. Goldwasser, 1994-2024, all rights reserved.  Permission is granted for public use or modification as
long as the Copyright notice is included.

This a simple utility to exercise most of the Nano BLE 33 Sense Rev1 or Rev2 sensors using the on-board LEDs and serial port.
The required Nano BLE 33 libraries are all either built into the Arduino IDE or Arduino Cloud Editor, or readily found via
a Web search.  Note that the primary difference between the Rev1 and Rev2 sketches are the libraries for the IMU and T/H.

Accelerometer (Gs) X, Y, Z; Gyroscope (Degs/s) Roll, Pitch, Yaw; Magnetic Field (Gauss) X, Y, Z; Temperature (DegC),
Pressure (mm/Hg), Humidity (%), Proximity (Prox), RGB Light Detect (R, G, B) values, and peak Mic values are all optionally
sent via the serial port as data-only, or with labels.

In addition, the on-board BUILTIN_LED, PWR_LED, and RGB_LED provide visual output:

1. Gyroscope: Displays the absolute value for Roll, Pitch, and Yaw as the brightness of each if the RGB leds.  Optional Gyro
   calibration to compensate for board-specific roll, pitch, and yaw offsets.  If enabled, Nano must remain stationary at
   startup while the RGB LEDs are blinking.  The default duration is ~12 blinks.  This may only be needed for Rev1 boards.
2. Proximity: Displays the distance as the brightness of the BUILTIN_LED (bright is closest).
3. Static Tilt (accelerometer Z value): Turns on the PWR_LED if more than approximately 45 degrees.
4. Microphone: Displays the peak intensity of the audio on a color scale using the RGB leds only when Gyro is not active.
5. Heartbeat: The BUILTIN_LED flashes at an approximately 1 Hz rate if there is no display activity.

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

Note: As coded, the Rev2 board runs substantially slower than Rev1 when some combination of the magnetic field, temperature,
humidity, and possibly pressure are used.  The cause is not clear, but to alleviate it, the IMU.magneticFieldAvailable() wait
is commented out (which seems to have no effect) and T,P,H are only sampled every skipcount times through the main loop.
*/

// Select Nano BLE 33 Sense Rev1 or Rev2
#define Rev2              // Select based on specific board

// User parameters
#define data1 0           // Sends data to serial port if 1, LEDs-only on Nano if 0
#define verbose1 0        // Display labels if 1, data-only if 0
#define senddiag1 0       // Include diagnostic information iff 1.  TBD, currently one integer (diag) is coded.
#define GyroAutoCal 1     // Perform automatic Gyro offset compensation at startup: The board must be stationary
                          //  while the LEDs are blinking.  If not enabled, use #define GR/GP/GY_COR values.

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
#include <Arduino_LSM9DS1.h>        // Accelerometer, magnetometer and gyroscope
#include <Arduino_HTS221.h>         // Temperature and humidity
#define timeoutvalue  17
#define skipcount 25
#endif

#ifdef Rev2
#include <Arduino_BMI270_BMM150.h>  // Accelerometer, magnetometer and gyroscope
#include <Arduino_HS300x.h>         // Temperature and humidity
#define timeoutvalue  17
#define skipcount 25
#endif

// Common libraries
#include <Arduino_LPS22HB.h>        // Pressure
#include <Arduino_APDS9960.h>       // RGB light and proximity
#include <PDM.h>                    // Microphone

#include <SPI.h>

float temperature = 0;
float pressure = 0;
float humidity = 0;
int proximity = 0;
int count = 0;
int i = 0;
int timeout = 0;
int loopcount = skipcount;
int sum = 0;

short sampleBuffer[1024];   // buffer to read audio samples into, each sample is 16-bits
volatile int samplesRead;   // number of samples read

#define LED_USER 17

void setup() {

  // Fixed calibration values may be needed if Gyro AutoCal is not enabled (more likely for Rev1)

  /*
  // Sample #1
  #ifdef Rev1
    GR_COR = 6.5;
    GP_COR = 0;
    GY_COR = 2.5;
  #endif
  */
  /*
  // Sample #2
  #ifdef Rev1
    GR_COR = 4;
    GP_COR = 1.3;
    GY_COR = 5.6;
  #endif
  */
  /*
  // All Rev2s tested were < 1
  #ifdef Rev2
    GR_COR = 0;
    GP_COR = 0;
    GY_COR = 0;
  #endif
  */

  // Set SPI clock speed
  SPISettings(64000000, MSBFIRST, SPI_MODE0); // It is not clear if this does anything

  // Set the LEDs pins as outputs
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_PWR, OUTPUT);
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);

  // Turn the LED_BUILTIN and LED_PWR on and set the RGB LEDs at low brightness
  analogWrite(LED_BUILTIN, 255);
  digitalWrite(LED_PWR, 1);
  analogWrite(LEDR, 200);
  analogWrite(LEDG, 200);
  analogWrite(LEDB, 200);

  if (data1 == 1) {
    Serial.begin(9600);
    while (!Serial);
    Serial.println();
    Serial.println();
    Serial.println("Arduino Nano 33 BLE Sense Sensor Exerciser.");
  }

  // configure the data receive callback
  PDM.onReceive(onPDMdata);

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

  if (!APDS.begin()) {
    if (data1 == 1) Serial.println("Error initializing APDS9960 sensor!");
  }

  // Microphone (one channel, mono mode. The only sample rates that work so far are 16.000 kHz and 41.667 kHz.  Go figure. ;-)
  if (!PDM.begin(1, 41667)) {
    Serial.println("Failed to start PDM!");
    while (1);
  }

  PDM.setBufferSize(1024);  // 512 is default; 1024 works but 2048 hangs
  PDM.setGain(25);          // Optionally set gain, defaults to 20

  // Banner blurb
  if ((verbose1 == 1) && (data1 == 1)) {
    Serial.println();
    Serial.println("Functions:");
    Serial.println();
    Serial.println("  - Acceleration in Gs.");
    Serial.println("  - Angle in degrees/second. LED threshold 25.");
    Serial.println("  - Magnetic field in Gauss.");
    Serial.println("  - Temperature in degrees Centigrade.");
    Serial.println("  - Pressure in mm/Hg.");
    Serial.println("  - Humidity in rel %.");
    Serial.println("  - Proximity in arbitrary units.");
    Serial.println("  - RGB light intensity in arbitrary units.");
    Serial.println("  - Peak soundlevel in arbitrary units.");
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

  led = (az * 255);
  if (led < 180) digitalWrite(LED_PWR, HIGH);  // Turn on LED_PWR if tilt is more than ~45 degrees
  else digitalWrite(LED_PWR, LOW);             // analogWrite(LED_PWR, led); Using analogWrite hangs here, even with a cosntant???

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
      CalCount--;          // Skip corrupted first value
      GyroAutoCalFlag = 1; // Disable RGB_LED output while GyroAutoCal in progress
    }
    else if (CalCount > 1) {
      loopcount = skipcount - 2;
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
    }
    else if (CalCount == 1) { // Compute average offsets
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

  if ((ledr > 8) || (ledp > 8) || (ledy > 8)) {
    RGB_LED_Color(ledr, ledp, ledy);
    timeout = 16;
  }
  else if (timeout > 0) timeout--;
    
  // Magnetic field

  // while (!IMU.magneticFieldAvailable()) {} // Disabled due to slowness with Rev2; Doesn't appear to make any difference in readings
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

  loopcount ++;
  if (loopcount >= skipcount) {  // Only check T,P,H every skipcount passes

  #ifdef Rev1
    temperature = HTS.readTemperature();
    humidity = HTS.readHumidity();
  #endif

  #ifdef Rev2
    temperature = HS300x.readTemperature();
    humidity = HS300x.readHumidity();
  #endif

    pressure = BARO.readPressure();

    loopcount = 0;
  }

  if (data1 == 1) {
    if (verbose1 == 1) Serial.print(" T:");
    sprintf(buffer, " %5.2f", temperature);
    Serial.print(buffer);
    if (verbose1 == 1) Serial.print(" DegC; P: ");
    sprintf(buffer, "%6.2f", pressure * 7.50062);
    Serial.print(buffer);
    if (verbose1 == 1) Serial.print(" mm/Hg; H: ");
    sprintf(buffer, "%5.2f", humidity);
    Serial.print(buffer);
    if (verbose1 == 1) Serial.print("% | Prox:");
  }

  // Proximity

  if (APDS.proximityAvailable()) proximity = APDS.readProximity();
  analogWrite(LED_BUILTIN, 230 - proximity);

  if (data1 == 1) {
    sprintf(buffer, " %3d", proximity);
    Serial.print(buffer);
  }

  // RGB light detect

  int r, g, b;

  while (!APDS.colorAvailable()) delay(5);
  APDS.readColor(r, g, b);

  if (data1 == 1) {
    if (verbose1 == 1) Serial.print(" | Light R: ");
    sprintf(buffer, "%3d", r / 16);
    Serial.print(buffer);
    if (verbose1 == 1) Serial.print(" G: ");
    sprintf(buffer, "%3d", g / 16);
    Serial.print(buffer);
    if (verbose1 == 1) Serial.print(" B: ");
    sprintf(buffer, "%3d", b / 16);
    Serial.print(buffer);
  }

  // Microphone

  if (samplesRead) {  // wait for samples to be read
    int i = 0;
    sum = 0;

    for (i = 0; i < samplesRead; i++)
      if (fabs(sampleBuffer[i]) > sum) sum = fabs(sampleBuffer[i]);  // Peak detect

      // Display the peak sound value in RGB_LED
    if (((fabs(gr - GR_COR) < 1) && (fabs(gp - GP_COR) < 1) && (fabs(gy - GY_COR)) < 1) && (GyroAutoCalFlag == 0)) {  // Only if no Gyro activity and GyroAutoCAl not in progress
      if (sum >= 1000) RGB_LED_Color(WHITE);
      else if (sum >= 600) RGB_LED_Color(RED);
      else if (sum >= 400) RGB_LED_Color(ORANGE);
      else if (sum >= 325) RGB_LED_Color(YELLOW);
      else if (sum >= 250) RGB_LED_Color(GREEN);
      else if (sum >= 175) RGB_LED_Color(CYAN);
      else if (sum >= 100) RGB_LED_Color(BLUE);
      else if (sum >= 50) RGB_LED_Color(MAGENTA);
      else if (sum >= 25) RGB_LED_Color(GRAY);
      else if (sum >= 0) RGB_LED_Color(BLACK);
    }
    if (sum >= 25) timeout = timeoutvalue * 2;
  }

  if (data1 == 1) {
    if (verbose1 == 1) Serial.print(" | Mic: ");
    sprintf(buffer, "%4d", sum);
    Serial.print(buffer);
    if (senddiag1 == 0) Serial.println("");
  }

  // clear the read count
  samplesRead = 0;  // Clear sample buffer

  // Optional diagnostic field

  if (senddiag1 == 1) {
    if (data1 == 1) {
      if (verbose1 == 1) Serial.print(" | Diag: ");
      sprintf(buffer, "%4d", diag);
      Serial.print(buffer);
      Serial.println("");
    }
  }

  // Heartbeat
  count++;
  if (count >= timeoutvalue) {
    if ((proximity > 230) && (ledr <= 8) && (ledp <= 8) && (ledy <= 8) && (sum < 25) && (timeout == 0)) {
      analogWrite(LED_BUILTIN, 255);  // Pulse BUILTIN led
      count = 0;
    }
  }
  delay(timeoutvalue);
  // digitalWrite(LED_PWR, !(digitalRead(LED_PWR))); // Diagnostic loop rate indicator
}

void RGB_LED_Color(int r, int g, int b) {
  analogWrite(LEDR, 255 - r);
  analogWrite(LEDG, 255 - g);
  analogWrite(LEDB, 255 - b);
}

void onPDMdata() {
  // query the number of bytes available
  int bytesAvailable = PDM.available();

  // read into the sample buffer
  PDM.read(sampleBuffer, bytesAvailable);

  // 16-bit, 2 bytes per sample
  samplesRead = bytesAvailable / 2;
}
