/*
  Nano BLE 33 Sense Sensor Test V21.

  Copyright® Samuel M. Goldwasser, 1994-2025, all rights reserved.  Permission is granted for public use or modification as
  long as the Copyright notice is included.

  This a simple utility to exercise most of the Nano BLE 33 Sense Rev1 or Rev2 sensors using the on-board LEDs and serial port.
  The required Nano BLE 33 libraries are all either built into the Arduino IDE or Arduino Cloud Editor, or readily found via
  a Web search.  Note that the primary difference between the Rev1 and Rev2 sketches are the libraries for the IMU and T/H.

  Accelerometer (Gs) X, Y, Z; Gyroscope (Degs/s) Roll, Pitch, Yaw; Magnetic Field (Gauss) X, Y, Z; Temperature (DegC),
  Pressure (mm/Hg), Humidity (%), Proximity (Prox), RGB Light Detect (R, G, B) values, and peak Mic values are all optionally
  sent via the serial port as data-only, or with labels.

  In addition, the on-board BUILTIN LED, PWR LED, and RGB LED provide visual output:

  1. Gyroscope: Displays the values for roll, pitch, and yaw as the brightness of each if the RGB leds as roll (+red/-cyan),
     pitch (+green/-magenta), and yaw (+blue/-yellow).  Optional gyro calibration to compensate for board-specific roll, pitch,
     and yaw offsets.  If GyroAutoCall is enabled, the board must remain stationary at startup while the RGB LEDs are blinking.
     The default duration is ~12 blinks, under 1 second.  This may only be needed for Rev1 boards.
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
  4. Go to Tools > Port, and select the correct port.
  5. Also select the board Rev, data formatting and Gyro AutoCal options in the #defines, below.

  This sketch should then compile without errors (though there may be warnings that can be ignored). ;-)

  Note: As coded, the Rev2 board runs substantially slower than Rev1 when some combination of the magnetic field, temperature,
  humidity, and possibly pressure are used.  The cause is not clear, but to alleviate it, the IMU.magneticFieldAvailable() wait
  is commented out (which seems to have no effect) and T,P,H are only sampled every skipcount times through the main loop.
*/

// Select Nano BLE 33 Sense Rev1 or Rev2
#define Rev2

// User parametersm
#define data1 0           // Sends data to serial port if 1, LEDs-only on Nano if 0
#define verbose1 0        // Display labels if 1, data-only if 0
#define senddiag1 0       // Include diagnostic information iff 1.  TBD, currently one integer (diag) is coded.
#define GyroAutoCal 1     // Perform automatic Gyro offset compensation at startup: The board must be stationary
                          //  while the RGB LEDs are blinking.  If not enabled, use #define GR/GP/GY_COR values.

// Sketch version number for banner. ;-)
#define Version 21

// Gyro offset parameters and variables
#define CalValues 50      // Number of Gyro samples to average for calibration

float RollOffsetSum = 0;  // Temporary variables for Gyro AutoCal sums
float PitchOffsetSum = 0;
float YawOffsetSum = 0;
float GR_COR = 0;         // Gyro offset correction values
float GP_COR = 0;
float GY_COR = 0;

int Ver = Version;

int CalCount = CalValues;
float pgr, pgp, pgy;

// Color palette for Gyro motion and audio in RGB_LEDs
#define BLACK 0, 0, 0
#define GRAY 7, 7, 7
#define MAGENTA 255, 0, 255
#define BLUE 0, 0, 255
#define CYAN 0, 255, 125
#define GREEN 0, 255, 0
#define YELLOW 255, 255, 0
#define ORANGE 255, 50, 0
#define RED 255, 0, 0
#define WHITE 255, 255, 255

#define scale 0.25

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

float ax, ay, az, gr, gp, gy, grcor, gpcor, gycor, mx, my, mz;
float temperature = 0;
float pressure = 0;
float humidity = 0;
int proximity = 0;
int count = 0;
int i = 0;
int timeout = 0;
int loopcount = skipcount;
int sum = 0;

short sampleBuffer[1024];       // buffer to read audio samples into, each sample is 16-bits
volatile int samplesRead = 0;   // number of samples read

int led, ledr, ledg, ledb;

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
  RGB_LED_Color(GRAY, 1.0);

  if (data1 == 1) {
    Serial.begin(9600);
    while (!Serial);
    Serial.println();
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

  if (!APDS.begin()) {
    if (data1 == 1) Serial.println("Failed to initialize APDS9960 sensor!");
  }

  // configure the data receive callback
  PDM.onReceive(onPDMdata);

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
    
    #ifdef Rev1
      Serial.print("**** Arduino Nano 33 BLE Sense Rev1 Sensor Test Version ");
    #endif

    #ifdef Rev2
      Serial.print("**** Arduino Nano 33 BLE Sense Rev2 Sensor Test Version ");
    #endif

    Serial.print(Ver);
    Serial.println(" ****");
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
    if (GyroAutoCal == 0) delay(1000);
  }

  // Gyro AutoCal
  if (GyroAutoCal == 1) Do_GyroAutoCal(25); // Argument is the delay in ms inside GyroAutoCal loop
}

void loop() {
  char buffer[40];
  int diag = 0;

  // Accelerometer

  while (!IMU.accelerationAvailable());
  IMU.readAcceleration(ax, ay, az);
  if (data1 == 1) {
    if (verbose1 == 1) Serial.print("  Acc (Gs) X: ");
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
  IMU.readGyroscope(gr, gp, gy);

  grcor = (gr - GR_COR);
  gpcor = (gp - GP_COR);
  gycor = (gy - GY_COR);

  if (data1 == 1) {
    if (verbose1 == 1) Serial.print("Gyro (Degs/s) R: ");
    sprintf(buffer, "%8.2f", grcor);
    Serial.print(buffer);
    if (verbose1 == 1) Serial.print(" P: ");
    sprintf(buffer, "%8.2f", gpcor);
    Serial.print(buffer);
    if (verbose1 == 1) Serial.print(" Y: ");
    sprintf(buffer, "%8.2f", gycor);
    Serial.print(buffer);
    if (verbose1 == 1) Serial.print("");
  }
    
  RGB_Gyro_Colors(grcor, gpcor, gycor, scale);
     
  // Magnetic field

  // while (!IMU.magneticFieldAvailable()) {} // Disabled due to slowness with Rev2; Doesn't appear to make any difference in readings
  IMU.readMagneticField(mx, my, mz);
  if (data1 == 1) {
    if (verbose1 == 1) Serial.print(" | Field (Gauss) X: ");
    sprintf(buffer, "%5.2f", mx / 100);
    Serial.print(buffer);
    if (verbose1 == 1) Serial.print(" Y: ");
    sprintf(buffer, "%5.2f", my / 100);
    Serial.print(buffer);
    if (verbose1 == 1) Serial.print(" Z: ");
    sprintf(buffer, "%5.2f", mz / 100);
    Serial.print(buffer);
    if (verbose1 == 1) Serial.print(" |");
  }

  // Temperature, humidity , and pressure

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

  int i = 0;
  sum = 0;

  if (samplesRead) {  // wait for samples to be read 
    for (i = 0; i < samplesRead; i++)
      if (fabs(sampleBuffer[i]) > sum) sum = fabs(sampleBuffer[i]);  // Peak detect

    // Display the peak sound value in RGB_LEDs
    if (((fabs(grcor) < 1) && (fabs(gpcor) < 1) && (fabs(gycor)) < 1)) {  // Only if no Gyro activity
      if (sum >= 1000) RGB_LED_Color(WHITE, 1.0);
      else if (sum >= 600) RGB_LED_Color(RED, 1.0);
      else if (sum >= 400) RGB_LED_Color(ORANGE, 0.8);
      else if (sum >= 325) RGB_LED_Color(YELLOW, 0.5);
      else if (sum >= 250) RGB_LED_Color(GREEN, 0.75);
      else if (sum >= 175) RGB_LED_Color(CYAN, 0.2);
      else if (sum >= 100) RGB_LED_Color(BLUE, 0.3);
      else if (sum >= 50) RGB_LED_Color(MAGENTA, 0.2);
      else if (sum >= 25) RGB_LED_Color(GRAY, 1.0);
      else if (sum >= 0) RGB_LED_Color(BLACK, 0.0);
    }
    if (sum >= 25) timeout = timeoutvalue;
  }

  if (data1 == 1) {
    if (verbose1 == 1) Serial.print(" | Mic: ");
    sprintf(buffer, "%4d", sum);
    Serial.print(buffer);
    if (senddiag1 == 0) Serial.println("");
  }
  
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
    if ((proximity > 230) && (sum < 25) && (timeout == 0)) {
      analogWrite(LED_BUILTIN, 255);  // Pulse BUILTIN led
      count = 0;
    }
  }
  samplesRead = 0;  // Clear microphone sample buffer
  delay(timeoutvalue);
  // digitalWrite(LED_PWR, !(digitalRead(LED_PWR))); // Diagnostic loop rate indicator
}

void RGB_LED_Color(int r, int g, int b, float intensity) {
    analogWrite(LEDR, (255 - (r * intensity)));
    analogWrite(LEDG, (255 - (g * intensity)));
    analogWrite(LEDB, (255 - (b * intensity)));
  }

void onPDMdata() {
  // query the number of bytes available
  int bytesAvailable = PDM.available();

  // read into the sample buffer
  PDM.read(sampleBuffer, bytesAvailable);

  // 16-bit, 2 bytes per sample
  samplesRead = bytesAvailable / 2;
}

void Do_GyroAutoCal (int Delay) {
  RGB_LED_Color(BLACK, 0);
  while (CalCount > 0) {
    while (!IMU.gyroscopeAvailable());
    pgr = gr; pgp = gp; pgy = gy;
    IMU.readGyroscope(gr, gp, gy);
    if (CalCount == CalValues) {
      CalCount--;          // Skip corrupted first value
    }
    else if (CalCount > 1) {
      delay(Delay);
      if (((fabs(gr - pgr) > 8)) || ((fabs(gp - pgp) > 8)) || ((fabs(gr - pgr) > 8))) { // Start over if too much gyro activity
        CalCount = CalValues;
        RollOffsetSum = 0;
        PitchOffsetSum = 0;
        YawOffsetSum = 0;
      } 
      else {
        RollOffsetSum += gr; // Update sums
        PitchOffsetSum += gp;
        YawOffsetSum += gy;
        if ((CalCount & 3) == 2) RGB_LED_Color(GRAY, 1.0); // Heartbeat while AutoCal in progress
        else RGB_LED_Color(BLACK, 0);
        CalCount--;
      }
    }
    else if (CalCount == 1) { // Compute average offsets
      GR_COR = RollOffsetSum / CalValues;
      GP_COR = PitchOffsetSum / CalValues;
      GY_COR = YawOffsetSum / CalValues;
      CalCount = 0;
      RGB_LED_Color(BLACK, 0);
    }
  }
}

void RGB_Axis_Colors(int Pos_R, int Pos_G, int Pos_B, int Neg_R, int Neg_G, int Neg_B, float axis) {
  if (axis > 0) {
    ledr += ((axis * Pos_R) / 255);
    ledg += ((axis * Pos_G) / 255);
    ledb += ((axis * Pos_B) / 255);
    }
  else {
    ledr -= ((axis * Neg_R) / 255);
    ledg -= ((axis * Neg_G) / 255);
    ledb -= ((axis * Neg_B) / 255);
    }
}

void RGB_Gyro_Colors (int roll, int pitch, int yaw, float atten) {
  if ((fabs(roll) > 1) || (fabs(pitch) > 1) || (fabs(yaw) > 1)) { // Update if above threahold
  ledr = 0;
  ledg = 0;
  ledb = 0;
  RGB_Axis_Colors(RED, CYAN, roll);
  RGB_Axis_Colors(GREEN, MAGENTA, pitch);
  RGB_Axis_Colors(BLUE, YELLOW, yaw);
  RGB_LED_Color(ledr, ledg, ledb, atten);
  timeout = 16;
  }
  if (timeout > 0) timeout--;
}
