/*
  Bluetooth Receive Sensor Data 1

  Copyright® Samuel M. Goldwasser, 1994-2025, all rights reserved.  Permission is granted for public use or modification as
  long as the Copyright notice is included.

  This is a pair sketches to exercise selected Nano BLE 33 Sense Rev1 or Rev2 sensors using the on-board LEDs and serial
  port, and to send selected sets of values to another BLE enabled board.  Tested with the Nano 33 BLE Sense Rev1 and Rev2
  for the sender and the Seed Studio XIAO nRF53840 Sense for the receiver,  But many others boards should work with trivial
  modifications.  Famous last words! ;-)

  The required libraries are all either built into the Arduino IDE or Arduino Cloud Editor, or readily found via a Web
  search.  Note that the primary difference between the Rev1 and Rev2 sketches are the libraries for the IMU and T/H.

  Accelerometer (Gs) X, Y, Z; Gyroscope (Degs/s) Roll, Pitch, Yaw; Magnetic Field (Gauss) X, Y, Z; Temperature (DegC),
  Pressure (mm/Hg), Humidity (%), Proximity (Prox), RGB Light Detect (R, G, B), and peak audio level (Mic) are optionally
  acquired and may be sent via the serial port as data-only, or with labels, and via Bluetooth to another board.

  The way it is set up by default, all the data listed above is acquired and displayed, but only Gyro (roll, pitch, yaw),
  Proximity, and peak audio level are sent via Bluetooth.  Strightforward modifications will be required of both the sending
  and receiving sketches to add some or all of the others.

  In addition to the data, the on-board BUILTIN (or USER) LED, PWR LED (if available), and RGB LEDs provide visual output:

  1. Gyroscope: Displays the values for roll, pitch, and yaw as the brightness of each if the RGB leds as roll (+red/-cyan),
     pitch (+green/-magenta), and yaw (+blue/-yellow).  Optional gyro calibration to compensate for board-specific roll, pitch,
     and yaw offsets.  If GyroAutoCall is enabled, the board must remain stationary at startup while the RGB LEDs are blinking.
     The default duration is ~12 blinks, under 1 second.  This may only be needed for Rev1 boards.
  2. Proximity: Displays the distance as the brightness of the BUILTIN_LED (bright is closest).
  3. Static Tilt (accelerometer Z value): Turns on the PWR_LED (if available) if more than approximately 45 degrees.
  4. Microphone: Displays the peak intensity of the audio on a color scale using the RGB LEDs ONLY when the Gyro is not active.
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

// User parameters
#define data1 1        // Sends data to serial port if 1, LEDs-only on Nano if 0
#define verbose1 1     // Display labels if 1, data-only if 0
#define senddiag1 0    // Include diagnostic information iff 1.  TBD, currently one integer (diag) is coded.
#define GyroAutoCal 1  // Perform automatic Gyro offset compensation at startup: The board must be stationary \
                       //  while the RGB LEDs are blinking.  If not enabled, use #define GR/GP/GY_COR values.

// Sketch version number for banner. ;-)
#define Version 1

#define Accelerometer   // Send X, Y, Z acceleration
#define Gyroscope       // Send Roll, Pitch, Yaw
#define Magnetic_Field  // Send X, Y, Z magnetic field
#define Environment     // Send T, P, H
#define Proximity       // Proximity value
#define Light_Detector  // RGB light values
// #define Input_Level           // Send generic input level
#define Microphone  // Send peak audio level
// #define Random_Number         // Send random number
// #define Diagnostics           // Send diagnostic data

// Gyro offset parameters and variables
#define CalValues 50  // Number of Gyro samples to average for calibration

float RollOffsetSum = 0;  // Temporary variables for Gyro AutoCal sums
float PitchOffsetSum = 0;
float YawOffsetSum = 0;
float GR_COR = 0;  // Gyro offset correction values
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
#include <ArduinoBLE.h>

// Nano 33 BLE Sense Version for libraries and loop speed
#ifdef Rev1
#include <Arduino_LSM9DS1.h>  // Accelerometer, magnetometer and gyroscope
#include <Arduino_HTS221.h>   // Temperature and humidity
#define timeoutvalue 8
#define skipcount 25
#endif

#ifdef Rev2
#include <Arduino_BMI270_BMM150.h>  // Accelerometer, magnetometer and gyroscope
#include <Arduino_HS300x.h>         // Temperature and humidity
#define timeoutvalue 8
#define skipcount 25
#endif

// Common libraries
#include <Arduino_LPS22HB.h>   // Pressure
#include <Arduino_APDS9960.h>  // RGB light and proximity
#include <PDM.h>               // Microphone
#include <SPI.h>

float ax, ay, az, gr, gp, gy, grcor, gpcor, gycor, mx, my, mz;
float temperature = 0;
float pressure = 0;
float humidity = 0;
int proximity = 255;
int count = 0;
int i = 0;
int timeout = 0;
int loopcount = skipcount;
int sum = 0;
int32_t grint, gpint, gyint, SNSend;
int SN = 0;
int SNPrint = 0;
uint16_t level = 0;
int Activity_Flags = 0;
int Send_Flags = 0;

// Activity and send selection flags.  Must shift up by 20 to stuff in SN word.
#define Accelerometer_Flag         0x1
#define Gyroscope_Flag             0x2
#define Magnetic_Field_Flag        0x4
#define Environment_Flag           0x8
#define Proximity_Flag            0x10
#define Light_Detector_Flag       0x20
#define Microphone_Flag           0x40
#define Input_Level_Flag          0x80
#define Random_Number_Flag       0x100
#define Diagnostics_Flag         0x200
#define Heartbeat_Flag           0x800

short sampleBuffer[1024];      // buffer to read audio samples into, each sample is 16-bits
volatile int samplesRead = 0;  // number of samples read

int led, ledr, ledg, ledb;

// Bluetooth® Low Energy inertial plus service (Custom UUIDs)

#define BLE_UUID_INERTIAL_PLUS_SERVICE "DA3F7226-D807-40E6-A24C-E9F16EDFCD3B"

#ifdef Accelerometer
#define BLE_UUID_ACCEL_X "DA3F7227-D807-40E6-A24C-E9F16EDFCD30"
#define BLE_UUID_ACCEL_Y "DA3F7227-D807-40E6-A24C-E9F16EDFCD31"
#define BLE_UUID_ACCEL_Z "DA3F7227-D807-40E6-A24C-E9F16EDFCD32"
#endif

#ifdef Gyroscope
#define BLE_UUID_GYRO_ROLL "DA3F7227-D807-40E6-A24C-E9F16EDFCD33"
#define BLE_UUID_GYRO_PITCH "DA3F7227-D807-40E6-A24C-E9F16EDFCD34"
#define BLE_UUID_GYRO_YAW "DA3F7227-D807-40E6-A24C-E9F16EDFCD35"
#endif

#ifdef Magnetic_Field
#define BLE_UUID_FIELD_X "DA3F7227-D807-40E6-A24C-E9F16EDFCD36"
#define BLE_UUID_FIELD_Y "DA3F7227-D807-40E6-A24C-E9F16EDFCD37"
#define BLE_UUID_FIELD_Z "DA3F7227-D807-40E6-A24C-E9F16EDFCD38"
#endif

#ifdef Environment
#define BLE_UUID_EVIRN_T "DA3F7227-D807-40E6-A24C-E9F16EDFCD39"
#define BLE_UUID_EVIRN_P "DA3F7227-D807-40E6-A24C-E9F16EDFCD3A"
#define BLE_UUID_EVIRN_H "DA3F7227-D807-40E6-A24C-E9F16EDFCD3B"
#endif

#ifdef Proximity
#define BLE_UUID_PROXIMITY "DA3F7227-D807-40E6-A24C-E9F16EDFCD3C"
#endif

#ifdef Light_Detector
#define BLE_UUID_LIGHT_R "DA3F7227-D807-40E6-A24C-E9F16EDFCD3D"
#define BLE_UUID_LIGHT_G "DA3F7227-D807-40E6-A24C-E9F16EDFCD3E"
#define BLE_UUID_LIGHT_B "DA3F7227-D807-40E6-A24C-E9F16EDFCD3F"
#endif

#ifdef Microphone
#define BLE_UUID_MICROPHONE "DA3F7227-D807-40E6-A24C-E9F16EDFCD40"
#endif

#ifdef Input_Level
#define BLE_UUID_INPUT_LEVEL "DA3F7227-D807-40E6-A24C-E9F16EDFCD41"
#endif

#ifdef Random_Number
#define BLE_UUID_RANDOM_NUMBER "DA3F7227-D807-40E6-A24C-E9F16EDFCD43"
#endif

#ifdef Diagnostics
#define BLE_UUID_DIAGNOSTICS "DA3F7227-D807-40E6-A24C-E9F16EDFCD34"
#endif

#define BLE_UUID_SEQUENCE_NUMBER "DA3F7227-D807-40E6-A24C-E9F16EDFCD4F"

BLEService inertial_plus(BLE_UUID_INERTIAL_PLUS_SERVICE);

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
  SPISettings(64000000, MSBFIRST, SPI_MODE0);  // It is not clear if this does anything

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

  // initialize the Bluetooth® Low Energy hardware
  if (!BLE.begin()) {
    if (data1 == 1) Serial.println("Starting Bluetooth® Low Energy module failed!");
    while (1);
  }

  // set advertised local name and service UUID:
  BLE.setLocalName("Gyro Monitor Sender");
  BLE.setAdvertisedService(inertial_plus);

  // start advertising
  BLE.advertise();
  if (data1 == 1) Serial.println("Bluetooth® device active, waiting for connections...");

  if (data1 == 1) {
    Serial.println();
    Serial.println("Bluetooth® Low Energy Central - Gyro Data Send");
  }


  // Startup

#if defined(Accelerometer) || defined(Gyroscope) || defined(Magnetic_Field)
  if (!IMU.begin()) {
    if (data1 == 1) Serial.println("Failed to initialize IMU!");
    while (1)
      ;
  }
#ifdef Accelerometer
Send_Flags |= Accelerometer_Flag;
#endif

#ifdef Gyroscope
Send_Flags |= Gyroscope_Flag;
#endif

#ifdef Magnetic_Field
Send_Flags |= Magnetic_Field_Flag;
#endif

#endif

  // Version-specific temperature and humidity sensor libraries

#ifdef Environment

#ifdef Rev1
  if (!HTS.begin()) {
#endif

#ifdef Rev2
    if (!HS300x.begin()) {
#endif

      if (data1 == 1) Serial.println("Failed to initialize temperature and humidity sensor!");
      while (1)
        ;
    }

    if (!BARO.begin()) {
      if (data1 == 1) Serial.println("Failed to initialize pressure sensor!");
      while (1)
        ;
    }

    if (!APDS.begin()) {
      if (data1 == 1) Serial.println("Failed to initialize APDS9960 sensor!");
    }
    Send_Flags |= Environment_Flag;
#endif

#ifdef Proximity
    Send_Flags |= Proximity_Flag;
#endif

#ifdef Light_Detector
    Send_Flags |= Light_Detector_Flag;
#endif

#ifdef Microphone
    // configure the data receive callback
    PDM.onReceive(onPDMdata);

    // Microphone (one channel, mono mode. The only sample rates that work so far are 16.000 kHz and 41.667 kHz.  Go figure. ;-)
    if (!PDM.begin(1, 41667)) {
      Serial.println("Failed to start PDM!");
      while (1);
    }

    PDM.setBufferSize(1024);  // 512 is default; 1024 works but 2048 hangs
    PDM.setGain(25);          // Optionally set gain, defaults to 20
    Send_Flags |= Microphone_Flag;
#endif

#ifdef Input_Level
    Send_Flags |= Input_Level_Flag;
#endif

#ifdef Random_Number
    Send_Flags |= Random_Number_Flag;
#endif

#ifdef Diagnostics
    Send_Flags |= Diagnostics_Flag;
#endif

    // Banner blurb

    if ((verbose1 == 1) && (data1 == 1)) {
      Serial.println();

#ifdef Rev1
      Serial.print("**** Arduino Nano 33 BLE Sense Rev1 Sensor Test Version ");
#endif

#ifdef Rev2
      Serial.print("**** Bluetooth Send Sensor Data Version ");
#endif

      Serial.print(Ver);
      Serial.println(" ****");
      Serial.println();
      Serial.println("Functions:");
      Serial.println();

#ifdef Accelerometer
      Serial.println("  - Acceleration in Gs.");
#endif

#ifdef Gyroscope
      Serial.println("  - Gyro angle in degrees/second. LED threshold 25.");
#endif

#ifdef Magnetic_Field
      Serial.println("  - Magnetic field in Gauss.");
#endif

#ifdef Environment
      Serial.println("  - Temperature in degrees Centigrade.");
      Serial.println("  - Pressure in mm/Hg.");
      Serial.println("  - Humidity in rel %.");
#endif

#ifdef Proximity
      Serial.println("  - Proximity in arbitrary units.");
#endif

#ifdef Light_Intensity
      Serial.println("  - RGB light intensity in arbitrary units.");
#endif

#ifdef Microphone
      Serial.println("  - Peak soundlevel in arbitrary units.");
#endif

      Serial.println();
      Serial.println("Data:");
      Serial.println("");
      if (GyroAutoCal == 0) delay(1000);
    }

    // Gyro AutoCal
    if (GyroAutoCal == 1) Do_GyroAutoCal(25);  // Argument is the delay in ms inside GyroAutoCal loop
  }
  char buffer[40];
  int diag = 0;

  void loop() {

    // start scanning for peripherals
    BLE.scanForUuid(BLE_UUID_INERTIAL_PLUS_SERVICE);

    // check if a peripheral has been discovered
    BLEDevice peripheral = BLE.available();

    while (peripheral) {
      // discovered a peripheral, print out address, local name, and advertised service
      if (data1 == 1) {
        Serial.print("Found ");
        Serial.print(peripheral.address());
        Serial.print(" '");
        Serial.print(peripheral.localName());
        Serial.print("' ");
        Serial.print(peripheral.advertisedServiceUuid());
        Serial.println();
      }

      if (peripheral.localName() != "Gyro Monitor Receiver") return;

      // stop scanning, OK to proceed
      BLE.stopScan();
      SendSensorData(peripheral);
      return;
    }
  }

  void SendSensorData(BLEDevice peripheral) {
    // connect to the peripheral
    if (data1 == 1) Serial.println("Connecting ...");

    if (peripheral.connect()) {
      if (data1 == 1) Serial.println("Connected");
      digitalWrite(LED_BUILTIN, HIGH);
    } else {
      if (data1 == 1) Serial.println("Failed to connect!");
      digitalWrite(LED_BUILTIN, LOW);
      return;
    }
    // discover peripheral attributes
    if (data1 == 1) Serial.println("Discovering attributes ...");
    if (peripheral.discoverAttributes()) {
      if (data1 == 1) Serial.println("Attributes discovered");
      Serial.println();
      Serial.println("Running");
      Serial.println();
    } else {
      if (data1 == 1) Serial.println("Attribute discovery failed!");
      peripheral.disconnect();
      return;
    }

    // retrieve the characteristics

#ifdef Gyroscope
    BLECharacteristic Gyro_Roll = peripheral.characteristic(BLE_UUID_GYRO_ROLL);
    BLECharacteristic Gyro_Pitch = peripheral.characteristic(BLE_UUID_GYRO_PITCH);
    BLECharacteristic Gyro_Yaw = peripheral.characteristic(BLE_UUID_GYRO_YAW);

    if (!Gyro_Roll) {
      if (data1 == 1) Serial.println("Peripheral does not have Roll Characteristic!");
      peripheral.disconnect();
      return;
    } else if (data1 == 1) Serial.println("Peripheral has Roll Characteristic!");
    if (!Gyro_Roll.canWrite()) {
      if (data1 == 1) Serial.println("Peripheral does not have a writable Roll characteristic!");
      peripheral.disconnect();
      return;
    }

    if (!Gyro_Pitch) {
      if (data1 == 1) Serial.println("Peripheral does not have Pitch Characteristic!");
      peripheral.disconnect();
      return;
    } else if (data1 == 1) Serial.println("Peripheral has Pitch Characteristic!");
    if (!Gyro_Pitch.canWrite()) {
      if (data1 == 1) Serial.println("Peripheral does not have a writable Pitch characteristic!");
      peripheral.disconnect();
      return;
    }

    if (!Gyro_Yaw) {
      if (data1 == 1) Serial.println("Peripheral does not have Yaw Characteristic!");
      peripheral.disconnect();
      return;
    } else if (data1 == 1) Serial.println("Peripheral has Yaw Characteristic!");
    if (!Gyro_Yaw.canWrite()) {
      if (data1 == 1) Serial.println("Peripheral does not have a writable Yaw characteristic!");
      peripheral.disconnect();
      return;
    }
#endif

#ifdef Microphone

    BLECharacteristic Audio_Level = peripheral.characteristic(BLE_UUID_MICROPHONE);

    if (!Audio_Level) {
      if (data1 == 1) Serial.println("Peripheral does not have Audio_Level Characteristic!");
      peripheral.disconnect();
      return;
    } else if (data1 == 1) Serial.println("Peripheral has Audio_Level Characteristic!");
    if (!Audio_Level.canWrite()) {
      if (data1 == 1) Serial.println("Peripheral does not have a writable Audio_Level characteristic!");
      peripheral.disconnect();
      return;
    }
#endif

    BLECharacteristic Sequence_Number = peripheral.characteristic(BLE_UUID_SEQUENCE_NUMBER);

    if (!Sequence_Number) {
      if (data1 == 1) Serial.println("Peripheral does not have Sequence_Number Characteristic!");
      peripheral.disconnect();
      return;
    } else if (data1 == 1) Serial.println("Peripheral has Sequence_Number Characteristic!");
    if (!Sequence_Number.canWrite()) {
      if (data1 == 1) Serial.println("Peripheral does not have a writable SN characteristic!");
      peripheral.disconnect();
      return;
    }

    if (data1 == 1) {
      Serial.print("Connected to Gyro Monitor Receiver: ");
      Serial.println(peripheral.address());  // Send the peripheral's MAC address
      if (data1 == 1) Serial.println();
    }

    analogWrite(LED_BUILTIN, 0);

    while (peripheral.connected()) {

      // Sequence Number
      SNPrint = SN;
      if (verbose1 == 1) Serial.print("SN: ");
      sprintf(buffer, "%6d", SNPrint);
      Serial.print(buffer);

#ifdef Accelerometer
      // Accelerometer
      while (!IMU.accelerationAvailable())
        ;
      IMU.readAcceleration(ax, ay, az);
      if (data1 == 1) {
        if (verbose1 == 1) Serial.print(" | Acc (Gs) X: ");
        sprintf(buffer, "%5.2f", ax);
        Serial.print(buffer);
        if (verbose1 == 1) Serial.print(" Y: ");
        sprintf(buffer, "%5.2f", ay);
        Serial.print(buffer);
        if (verbose1 == 1) Serial.print(" Z: ");
        sprintf(buffer, "%5.2f", az);
        Serial.print(buffer);
      }

      led = (az * 255);
      if (led < 180) digitalWrite(LED_PWR, HIGH);  // Turn on LED_PWR if tilt is more than ~45 degrees
      else digitalWrite(LED_PWR, LOW);             // analogWrite(LED_PWR, led); Using analogWrite hangs here, even with a cosntant???
#endif

#ifdef Gyroscope
      // Gyroscope

      while (!IMU.gyroscopeAvailable()) {}
      IMU.readGyroscope(gr, gp, gy);

      grcor = (gr - GR_COR);
      gpcor = (gp - GP_COR);
      gycor = (gy - GY_COR);

      if (data1 == 1) {
        if (verbose1 == 1) Serial.print(" | Gyro (Degs/s) R: ");
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
#endif

#ifdef Magnetic_Field
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
      }
#endif

#ifdef Environment
      // Temperature, humidity , and pressure

      loopcount++;
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
        if (verbose1 == 1) Serial.print(" | T:");
        sprintf(buffer, " %5.2f", temperature);
        Serial.print(buffer);
        if (verbose1 == 1) Serial.print(" DegC; P: ");
        sprintf(buffer, "%6.2f", pressure * 7.50062);
        Serial.print(buffer);
        if (verbose1 == 1) Serial.print(" mm/Hg; H: ");
        sprintf(buffer, "%5.2f", humidity);
        Serial.print(buffer);
      }
#endif

#ifdef Proximity

      // Proximity

      if (APDS.proximityAvailable()) proximity = APDS.readProximity();
      analogWrite(LED_BUILTIN, 230 - proximity);

      if (data1 == 1) {
        if (verbose1 == 1) Serial.print("% | Prox:");
        sprintf(buffer, " %3d", proximity);
        Serial.print(buffer);
      }
#endif

#ifdef Light_Intensity
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
#endif

#ifdef Microphone
      // Microphone

      int i = 0;
      sum = 0;
      Activity_Flags &= !Microphone_Flag;  // Clear microphone flag

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
          else RGB_LED_Color(BLACK, 0.0);
        }
        if (sum >= 25) {
          timeout = timeoutvalue;
          Activity_Flags |= Microphone_Flag;  // Set microphone flag
        }
      }

      if (data1 == 1) {
        if (verbose1 == 1) Serial.print(" | Mic: ");
        sprintf(buffer, "%4d", sum);
        Serial.print(buffer);
      }

#endif

#ifdef Gyroscope
      enKludge();  // Kludge to get fractional resolution without using floating point. ;-)

      Gyro_Roll.writeValue(grint);
      Gyro_Pitch.writeValue(gpint);
      Gyro_Yaw.writeValue(gyint);
#endif

#ifdef Microphone
      level = sum;
      Audio_Level.writeValue(level);
#endif

      SNSend = SN;
      SNSend |= (Send_Flags << 20);
      Sequence_Number.writeValue(SNSend);
      SN++;

      // Optional diagnostic field
      if (senddiag1 == 1) {
        if (data1 == 1) {
          if (verbose1 == 1) Serial.print(" | Diag: ");
          sprintf(buffer, "%4d", diag);
          Serial.print(buffer);
        }
      }

      if (data1 == 1) Serial.println("");

      // Heartbeat: Pulse BUILTIN led if nothing going on
      count++;
      if (count >= timeoutvalue) {
        if ((fabs(grcor) < 1) && (fabs(gpcor) < 1) && (fabs(gycor) < 1) && (proximity > 230) && (Activity_Flags == 0) && (timeout == 0)) {
          digitalWrite(LED_BUILTIN, 1);
          delay(10);
          digitalWrite(LED_BUILTIN, 0);
          count = 0;
          Send_Flags |= Heartbeat_Flag;
        }
      }
      else Send_Flags &= (~Heartbeat_Flag);

#ifdef Microphone
      samplesRead = 0;  // Clear microphone sample buffer
#endif

      // delay(timeoutvalue);
      // digitalWrite(LED_PWR, !(digitalRead(LED_PWR))); // Diagnostic loop rate indicator
    }

    // Connection innactive
    if (data1 == 1) {
      Serial.println();
      Serial.println("Peripheral disconnected");
    }

    digitalWrite(LED_PWR, HIGH);
    digitalWrite(LED_BUILTIN, LOW);
    RGB_LED_Color(BLACK, 0);
  }

  void RGB_LED_Color(int r, int g, int b, float intensity) {
    analogWrite(LEDR, (255 - (r * intensity)));
    analogWrite(LEDG, (255 - (g * intensity)));
    analogWrite(LEDB, (255 - (b * intensity)));
  }

#ifdef Microphone
  void onPDMdata() {
    // query the number of bytes available
    int bytesAvailable = PDM.available();

    // read into the sample buffer
    PDM.read(sampleBuffer, bytesAvailable);

    // 16-bit, 2 bytes per sample
    samplesRead = bytesAvailable / 2;
  }
#endif

  void Do_GyroAutoCal(int Delay) {
    RGB_LED_Color(BLACK, 0);
    while (CalCount > 0) {
      while (!IMU.gyroscopeAvailable())
        ;
      pgr = gr;
      pgp = gp;
      pgy = gy;
      IMU.readGyroscope(gr, gp, gy);
      if (CalCount == CalValues) {
        CalCount--;  // Skip corrupted first value
      } else if (CalCount > 1) {
        delay(Delay);
        if (((fabs(gr - pgr) > 8)) || ((fabs(gp - pgp) > 8)) || ((fabs(gr - pgr) > 8))) {  // Start over if too much gyro activity
          CalCount = CalValues;
          RollOffsetSum = 0;
          PitchOffsetSum = 0;
          YawOffsetSum = 0;
        } else {
          RollOffsetSum += gr;  // Update sums
          PitchOffsetSum += gp;
          YawOffsetSum += gy;
          if ((CalCount & 3) == 2) RGB_LED_Color(GRAY, 1.0);  // Heartbeat while AutoCal in progress
          else RGB_LED_Color(BLACK, 0);
          CalCount--;
        }
      } else if (CalCount == 1) {  // Compute average offsets
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
    } else {
      ledr -= ((axis * Neg_R) / 255);
      ledg -= ((axis * Neg_G) / 255);
      ledb -= ((axis * Neg_B) / 255);
    }
  }

  void RGB_Gyro_Colors(int roll, int pitch, int yaw, float atten) {
    ledr = 0;
    ledg = 0;
    ledb = 0;
    if ((fabs(roll) > 1) || (fabs(pitch) > 1) || (fabs(yaw) > 1)) {  // Update if above threahold
      RGB_Axis_Colors(RED, CYAN, roll);
      RGB_Axis_Colors(GREEN, MAGENTA, pitch);
      RGB_Axis_Colors(BLUE, YELLOW, yaw);
      RGB_LED_Color(ledr, ledg, ledb, atten);
      Activity_Flags |= Gyroscope_Flag;
      timeout = 16;
    } else if (sum <= 25) RGB_LED_Color(BLACK, 0);
    Activity_Flags &= !Gyroscope_Flag;
    if (timeout > 0) timeout--;
  }

  void enKludge() {
    grint = grcor * 1000;
    gpint = gpcor * 1000;
    gyint = gycor * 1000;
  }
